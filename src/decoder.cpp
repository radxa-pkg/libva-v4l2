// SPDX-License-Identifier: MIT
#include "internal.hpp"
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <glob.h>
#include <cerrno>

namespace irisva {
namespace {
constexpr unsigned OUTPUT = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
constexpr unsigned CAPTURE = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
int call(int fd, unsigned long command, void *p) {
    int r;
    do {
        r = ioctl(fd, command, p);
    } while (r < 0 && errno == EINTR);
    return r;
}
void checked(int fd, unsigned long command, void *p, const char *name) {
    if (call(fd, command, p) < 0)
        throw Error(VA_STATUS_ERROR_OPERATION_FAILED, std::string(name) + ": " + strerror(errno));
}
struct QueueBuffer {
    v4l2_plane plane{};
    v4l2_buffer buffer{};
    explicit QueueBuffer(unsigned type, unsigned index = 0) {
        buffer.type = type;
        buffer.index = index;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.length = 1;
        buffer.m.planes = &plane;
    }
};
bool expired(std::chrono::steady_clock::time_point start) {
    return std::chrono::steady_clock::now() - start > std::chrono::seconds(5);
}
} // namespace
Memory::~Memory() {
    fastcv_registration.reset();
    if (mapping)
        munmap(mapping, size);
    if (fd >= 0)
        close(fd);
}
uint8_t *Memory::map() {
    if (!mapping) {
        void *p = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        check(p != MAP_FAILED, "mmap DMA-BUF failed");
        mapping = p;
    }
    return static_cast<uint8_t *>(mapping);
}
std::string find_device() {
    std::vector<std::string> paths;
    if (const char *p = std::getenv("IRIS_VAAPI_DEVICE"))
        paths.push_back(p);
    else {
        glob_t g{};
        if (!glob("/dev/video*", 0, nullptr, &g))
            for (size_t i = 0; i < g.gl_pathc; ++i)
                paths.emplace_back(g.gl_pathv[i]);
        globfree(&g);
    }
    for (const auto &path : paths) {
        int fd = open(path.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0)
            continue;
        v4l2_capability cap{};
        bool match = false;
        if (!call(fd, VIDIOC_QUERYCAP, &cap) &&
            is_supported_stateful_driver(reinterpret_cast<char *>(cap.driver))) {
            unsigned caps =
                cap.capabilities & V4L2_CAP_DEVICE_CAPS ? cap.device_caps : cap.capabilities;
            if (caps & V4L2_CAP_VIDEO_M2M_MPLANE) {
                v4l2_fmtdesc fmt{};
                fmt.type = OUTPUT;
                while (!call(fd, VIDIOC_ENUM_FMT, &fmt)) {
                    if (fmt.pixelformat == V4L2_PIX_FMT_H264)
                        match = true;
                    ++fmt.index;
                }
            }
        }
        close(fd);
        if (match)
            return path;
    }
    throw Error(VA_STATUS_ERROR_OPERATION_FAILED,
                "no supported stateful H264 decoder found");
}
Decoder::Decoder(const std::string &device, unsigned width, unsigned height, unsigned surfaces,
                 VAProfile profile, int render_fd)
    : width_(width), height_(height), pool_size_(std::min(64u, std::max(32u, surfaces + 4))),
      fourcc_(is_10bit(profile) ? VA_FOURCC_P010 : VA_FOURCC_NV12), copier_(render_fd) {
    fd_ = open(device.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
    check(fd_ >= 0, "open Iris decoder");
    try {
        v4l2_event_subscription sub{};
        sub.type = V4L2_EVENT_SOURCE_CHANGE;
        checked(fd_, VIDIOC_SUBSCRIBE_EVENT, &sub, "SUBSCRIBE SOURCE_CHANGE");
        v4l2_format fmt{};
        fmt.type = OUTPUT;
        fmt.fmt.pix_mp.width = width;
        fmt.fmt.pix_mp.height = height;
        fmt.fmt.pix_mp.pixelformat = is_vp9(profile)    ? V4L2_PIX_FMT_VP9
                                     : is_hevc(profile) ? V4L2_PIX_FMT_HEVC
                                                        : V4L2_PIX_FMT_H264;
        unsigned codec = fmt.fmt.pix_mp.pixelformat;
        fmt.fmt.pix_mp.num_planes = 1;
        checked(fd_, VIDIOC_S_FMT, &fmt, "S_FMT OUTPUT");
        check(fmt.fmt.pix_mp.pixelformat == codec, "Iris does not support the requested codec");
        v4l2_control delay{};
        delay.id = V4L2_CID_MPEG_VIDEO_DEC_DISPLAY_DELAY;
        delay.value = 0;
        checked(fd_, VIDIOC_S_CTRL, &delay,
                "S_CTRL DISPLAY_DELAY (Iris decode-order patch required)");
        delay.id = V4L2_CID_MPEG_VIDEO_DEC_DISPLAY_DELAY_ENABLE;
        delay.value = 1;
        checked(fd_, VIDIOC_S_CTRL, &delay, "S_CTRL DISPLAY_DELAY_ENABLE");
        allocate(OUTPUT, 4, output_);
        unsigned type = OUTPUT;
        checked(fd_, VIDIOC_STREAMON, &type, "STREAMON OUTPUT");
        output_on_ = true;
        if (const char *path = std::getenv("IRIS_VAAPI_DUMP"))
            dump_ = fopen(path, "wb");
        trace("Iris session %s %ux%u output-buffers=%zu", device.c_str(), width, height,
              output_.size());
    } catch (...) {
        close(fd_);
        fd_ = -1;
        throw;
    }
}
Decoder::~Decoder() {
    if (dump_)
        fclose(dump_);
    if (fd_ >= 0) {
        unsigned type = OUTPUT;
        if (output_on_)
            call(fd_, VIDIOC_STREAMOFF, &type);
        type = CAPTURE;
        if (capture_on_)
            call(fd_, VIDIOC_STREAMOFF, &type);
        close(fd_);
    }
    for (auto &[token, surface] : pending_) {
        surface->status = VA_STATUS_ERROR_DECODING_ERROR;
        surface->pending = false;
    }
}
void Decoder::allocate(unsigned type, unsigned count, std::vector<std::shared_ptr<Memory>> &pool) {
    v4l2_requestbuffers req{};
    req.type = type;
    req.memory = V4L2_MEMORY_MMAP;
    req.count = count;
    checked(fd_, VIDIOC_REQBUFS, &req, "REQBUFS");
    check(req.count > 0 && req.count <= 64, "invalid allocated buffer count");
    for (unsigned i = 0; i < req.count; ++i) {
        QueueBuffer q(type, i);
        checked(fd_, VIDIOC_QUERYBUF, &q.buffer, "QUERYBUF");
        auto m = std::make_shared<Memory>();
        m->origin = MemoryOrigin::Iris;
        m->index = i;
        m->size = q.plane.length;
        if (type == OUTPUT) {
            void *p = mmap(nullptr, m->size, PROT_READ | PROT_WRITE, MAP_SHARED, fd_,
                           q.plane.m.mem_offset);
            check(p != MAP_FAILED, "mmap OUTPUT");
            m->mapping = p;
        } else {
            v4l2_exportbuffer exp{};
            exp.type = type;
            exp.index = i;
            exp.flags = O_CLOEXEC | O_RDWR;
            checked(fd_, VIDIOC_EXPBUF, &exp, "EXPBUF");
            m->fd = exp.fd;
        }
        pool.push_back(m);
    }
}
void Decoder::configure_capture() {
    check(!capture_on_, "in-context dynamic resolution change not yet implemented",
          VA_STATUS_ERROR_UNIMPLEMENTED);
    v4l2_format fmt{};
    fmt.type = CAPTURE;
    checked(fd_, VIDIOC_G_FMT, &fmt, "G_FMT CAPTURE");
    unsigned pixel_format = fourcc_ == VA_FOURCC_P010 ? V4L2_PIX_FMT_P010 : V4L2_PIX_FMT_NV12;
    fmt.fmt.pix_mp.pixelformat = pixel_format;
    checked(fd_, VIDIOC_S_FMT, &fmt, "S_FMT CAPTURE");
    check(fmt.fmt.pix_mp.pixelformat == pixel_format && fmt.fmt.pix_mp.num_planes == 1,
          "unsupported CAPTURE layout");
    v4l2_control minimum{};
    minimum.id = V4L2_CID_MIN_BUFFERS_FOR_CAPTURE;
    checked(fd_, VIDIOC_G_CTRL, &minimum, "G_CTRL minimum capture buffers");
    // FFmpeg may pass zero render targets and allocate VA surfaces later.
    // The firmware minimum alone leaves no room for surfaces retained in VA's
    // reference/reorder queues. Reserve at least 32 slots and honor larger
    // renderer pools (VLC), within the patched Iris driver's 64-buffer limit.
    trace("CAPTURE pool target=%u firmware-min=%d", pool_size_, minimum.value);
    allocate(CAPTURE, std::max(pool_size_, unsigned(minimum.value)), capture_);
    check(capture_.size() >= pool_size_,
          "CAPTURE pool too small; apply the Iris gen2 capture-pool kernel patch",
          VA_STATUS_ERROR_ALLOCATION_FAILED);
    for (auto &m : capture_) {
        m->stride = fmt.fmt.pix_mp.plane_fmt[0].bytesperline;
        m->storage_height = fmt.fmt.pix_mp.height;
        m->width = fmt.fmt.pix_mp.width;
        m->height = fmt.fmt.pix_mp.height;
        m->fourcc = fourcc_;
    }
    requeue();
    unsigned type = CAPTURE;
    checked(fd_, VIDIOC_STREAMON, &type, "STREAMON CAPTURE");
    capture_on_ = true;
    source_change_ = false;
    trace("CAPTURE %ux%u pitch=%u count=%zu", fmt.fmt.pix_mp.width, fmt.fmt.pix_mp.height,
          fmt.fmt.pix_mp.plane_fmt[0].bytesperline, capture_.size());
}
void Decoder::requeue() {
    if (last_)
        return;
    for (auto &m : capture_) {
        if (!m->queued && m.use_count() == 1) {
            QueueBuffer q(CAPTURE, m->index);
            q.plane.length = m->size;
            checked(fd_, VIDIOC_QBUF, &q.buffer, "QBUF CAPTURE");
            m->queued = true;
        }
    }
}
void Decoder::pump(int timeout_ms) {
    requeue();
    if (timeout_ms) {
        pollfd p{fd_, POLLIN | POLLPRI | POLLOUT, 0};
        int ret;
        do {
            ret = poll(&p, 1, timeout_ms);
        } while (ret < 0 && errno == EINTR);
        check(ret >= 0 && !(p.revents & POLLNVAL), "poll Iris failed");
    }
    for (;;) {
        v4l2_event event{};
        if (call(fd_, VIDIOC_DQEVENT, &event) < 0) {
            check(errno == EAGAIN || errno == ENOENT, "DQEVENT failed");
            break;
        }
        if (event.type == V4L2_EVENT_SOURCE_CHANGE)
            source_change_ = true;
    }
    if (source_change_ && !capture_on_)
        configure_capture();
    for (;;) {
        QueueBuffer q(OUTPUT);
        if (call(fd_, VIDIOC_DQBUF, &q.buffer) < 0) {
            check(errno == EAGAIN, "DQBUF OUTPUT failed");
            break;
        }
        check(q.buffer.index < output_.size(), "invalid OUTPUT index");
        output_[q.buffer.index]->queued = false;
        check(!(q.buffer.flags & V4L2_BUF_FLAG_ERROR), "Iris bitstream error",
              VA_STATUS_ERROR_DECODING_ERROR);
    }
    if (!capture_on_ || last_)
        return;
    for (;;) {
        QueueBuffer q(CAPTURE);
        if (call(fd_, VIDIOC_DQBUF, &q.buffer) < 0) {
            check(errno == EAGAIN || errno == EPIPE, "DQBUF CAPTURE failed");
            break;
        }
        check(q.buffer.index < capture_.size(), "invalid CAPTURE index");
        auto &m = capture_[q.buffer.index];
        m->queued = false;
        trace("CAPTURE index=%u used=%u offset=%u ts=%lld:%lld flags=%x", q.buffer.index,
              q.plane.bytesused, q.plane.data_offset, (long long)q.buffer.timestamp.tv_sec,
              (long long)q.buffer.timestamp.tv_usec, q.buffer.flags);
        if (q.plane.bytesused > q.plane.data_offset) {
            uint64_t token =
                uint64_t(q.buffer.timestamp.tv_sec) * 1000000 + q.buffer.timestamp.tv_usec;
            auto it = pending_.find(token);
            check(it != pending_.end(), "unmatched CAPTURE timestamp",
                  VA_STATUS_ERROR_DECODING_ERROR);
            auto surface = it->second;
            m->data_offset = q.plane.data_offset;
            surface->status = q.buffer.flags & V4L2_BUF_FLAG_ERROR ? VA_STATUS_ERROR_DECODING_ERROR
                                                                   : VA_STATUS_SUCCESS;
            if (surface->status == VA_STATUS_SUCCESS && surface->persistent_export) {
                // Keep the fd/offset/pitch which the client already imported.
                // CAPTURE selection is owned by the stateful decoder, not VA.
                try {
                    check(bool(surface->memory), "persistent export lost its storage");
                    copier_.copy(surface->memory, m, surface->width, surface->height);
                } catch (const Error &e) {
                    surface->status = e.status;
                    surface->pending = false;
                    pending_.erase(it);
                    throw;
                }
            } else if (!surface->persistent_export) {
                surface->memory = m;
            }
            surface->pending = false;
            trace("decoded token=%llu capture=%u flags=%x", (unsigned long long)token,
                  q.buffer.index, q.buffer.flags);
            pending_.erase(it);
        }
        if (q.buffer.flags & V4L2_BUF_FLAG_LAST) {
            last_ = true;
            break;
        }
    }
    check(!source_change_, "in-context dynamic resolution change not yet implemented",
          VA_STATUS_ERROR_UNIMPLEMENTED);
}
void Decoder::submit(const std::vector<uint8_t> &bytes, const std::shared_ptr<Surface> &surface) {
    std::lock_guard<std::mutex> lock(mutex_);
    // VLC allocates its renderer pool before creating the FFmpeg context,
    // which can pass zero render targets to VA. Preserve the allocation hint
    // on each surface so CAPTURE can cover that pool plus in-flight output.
    if (!capture_on_)
        pool_size_ = std::min(64u, std::max(pool_size_, surface->allocation_count + 4));
    pump(0);
    auto start = std::chrono::steady_clock::now();
    std::shared_ptr<Memory> slot;
    while (!slot) {
        for (auto &m : output_)
            if (!m->queued) {
                slot = m;
                break;
            }
        if (slot)
            break;
        check(!expired(start), "OUTPUT queue timeout", VA_STATUS_ERROR_TIMEDOUT);
        pump(20);
    }
    check(!bytes.empty() && bytes.size() <= slot->size, "compressed picture too large",
          VA_STATUS_ERROR_INVALID_BUFFER);
    std::memcpy(slot->mapping, bytes.data(), bytes.size());
    if (dump_) {
        fwrite(bytes.data(), 1, bytes.size(), dump_);
        fflush(dump_);
    }
    uint64_t token = next_token_++;
    QueueBuffer q(OUTPUT, slot->index);
    q.plane.bytesused = bytes.size();
    q.plane.length = slot->size;
    q.buffer.timestamp.tv_sec = token / 1000000;
    q.buffer.timestamp.tv_usec = token % 1000000;
    checked(fd_, VIDIOC_QBUF, &q.buffer, "QBUF OUTPUT");
    slot->queued = true;
    surface->pending = true;
    surface->token = token;
    surface->status = VA_STATUS_SUCCESS;
    pending_[token] = surface;
    trace("submit token=%llu bytes=%zu", (unsigned long long)token, bytes.size());
    if (!capture_on_) {
        while (!capture_on_) {
            check(!expired(start), "initial SOURCE_CHANGE timeout", VA_STATUS_ERROR_TIMEDOUT);
            pump(20);
        }
    }
}
void Decoder::refresh() {
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if (lock.owns_lock())
        pump(0);
}
void Decoder::finish() {
    std::lock_guard<std::mutex> lock(mutex_);
    auto start = std::chrono::steady_clock::now();
    while (!pending_.empty()) {
        check(!expired(start), "pending pictures at context destruction", VA_STATUS_ERROR_TIMEDOUT);
        pump(20);
    }
}
void Decoder::sync(const std::shared_ptr<Surface> &surface) {
    std::unique_lock<std::mutex> lock(mutex_);
    pump(0);
    auto start = std::chrono::steady_clock::now();
    while (surface->pending) {
        if (expired(start)) {
            unsigned queued = 0, retained = 0;
            for (auto &m : capture_) {
                queued += m->queued;
                retained += m.use_count() > 1;
            }
            trace("sync timeout token=%llu pending=%zu capture=%zu queued=%u retained=%u",
                  (unsigned long long)surface->token, pending_.size(), capture_.size(), queued,
                  retained);
            throw Error(VA_STATUS_ERROR_TIMEDOUT, "surface sync timeout");
        }
        // Decode-order output makes each submitted picture independently
        // synchronizable. Never issue STOP based on an idle-time heuristic:
        // vaSyncSurface is not an end-of-stream notification.
        lock.unlock();
        pollfd p{fd_, POLLIN | POLLPRI, 0};
        poll(&p, 1, 10);
        lock.lock();
        pump(0);
        check(!last_ || !surface->pending, "surface missing at LAST",
              VA_STATUS_ERROR_DECODING_ERROR);
    }
    if (surface->status != VA_STATUS_SUCCESS)
        throw Error(surface->status, "surface decode failed");
}
} // namespace irisva
