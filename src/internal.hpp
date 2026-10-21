// SPDX-License-Identifier: MIT
#pragma once
#include <va/va.h>
#include <va/va_backend.h>
#include <va/va_backend_vpp.h>
#include <va/va_drmcommon.h>
#include <va/va_dec_hevc.h>
#include <va/va_dec_vp9.h>
#include <va/va_enc_h264.h>
#include <va/va_enc_hevc.h>
#include <algorithm>
#include <chrono>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <atomic>
#include <type_traits>

extern "C" int irisva_msm_allocate(int drm_fd, uint64_t size, int *coherent);

namespace irisva {
struct Error : std::runtime_error {
    VAStatus status;
    Error(VAStatus s, const std::string &m) : std::runtime_error(m), status(s) {}
};
inline void check(bool b, const char *m, VAStatus s = VA_STATUS_ERROR_OPERATION_FAILED) {
    if (!b)
        throw Error(s, m);
}
inline bool debug() {
    return std::getenv("IRIS_VAAPI_DEBUG") != nullptr;
}
template <class... A> void trace(const char *f, A... a) {
    if (debug()) {
        std::fprintf(stderr, "iris-vaapi: ");
        std::fprintf(stderr, f, a...);
        std::fprintf(stderr, "\n");
    }
}
enum class MemoryOrigin { Unknown, MsmWriteCombined, MsmCoherent, DmaHeap, Imported, Iris };
struct Memory {
    std::shared_ptr<void> fastcv_registration;
    MemoryOrigin origin = MemoryOrigin::Unknown;
    int fd = -1;
    void *mapping = nullptr;
    size_t size = 0;
    unsigned index = 0, stride = 0, storage_height = 0, width = 0, height = 0;
    unsigned data_offset = 0, fourcc = VA_FOURCC_NV12;
    bool queued = false;
    ~Memory();
    uint8_t *map();
};
class Decoder;
class SurfaceCopier {
    // Calls are serialized by the owning decoder or encoder, including lazy setup.
    struct Impl;
    int render_fd_; // Borrowed VA display fd; duplicated when GPU copying is first used.
    std::unique_ptr<Impl> impl_;

  public:
    explicit SurfaceCopier(int render_fd);
    ~SurfaceCopier();
    void copy(const std::shared_ptr<Memory> &destination, const std::shared_ptr<Memory> &source,
              unsigned width, unsigned height);
};
class Encoder;
struct EncodeTask {
    std::weak_ptr<Encoder> encoder;
    bool pending = true;
    VAStatus status = VA_STATUS_SUCCESS;
};
struct Surface {
    std::shared_ptr<EncodeTask> encode_task;
    unsigned width, height, fourcc = VA_FOURCC_NV12;
    unsigned allocation_count = 1;
    std::shared_ptr<Memory> memory;
    // An export before the first picture may be cached by the consumer for
    // this surface's whole lifetime (Chromium's NativePixmap frame pool).
    bool persistent_export = false;
    std::weak_ptr<Decoder> decoder;
    uint64_t token = 0;
    std::atomic<VAStatus> status{VA_STATUS_SUCCESS};
    std::atomic<bool> pending{false};
    unsigned derived_images = 0, mapped_images = 0, acquired_handles = 0;
};
struct Buffer {
    std::shared_ptr<EncodeTask> encode_task;
    VABufferType type;
    unsigned element_size = 0, elements = 0;
    std::vector<uint8_t> data;
    std::shared_ptr<Surface> parent;
    std::shared_ptr<Memory> memory;
    int acquired_fd = -1;
    bool mapped = false;
    VACodedBufferSegment coded{};
    bool coded_ready = false, coded_mapped = false;
    uint32_t context_id = VA_INVALID_ID;
    ~Buffer();
    uint8_t *map();
    void unmap();
};
struct Slice {
    VASliceParameterBufferH264 params{};
    std::vector<uint8_t> bytes;
};
struct Picture {
    VAPictureParameterBufferH264 params{};
    VAIQMatrixBufferH264 iq{};
    bool has_params = false, has_iq = false;
    std::vector<VASliceParameterBufferH264> pending_slices;
    std::vector<Slice> slices;
};
struct H264State {
    std::vector<uint8_t> sps;
    std::map<unsigned, std::vector<uint8_t>> pps;
};
struct HevcSlice {
    VASliceParameterBufferHEVC params{};
    std::vector<uint8_t> bytes;
};
struct HevcPicture {
    VAPictureParameterBufferHEVC params{};
    VAIQMatrixBufferHEVC iq{};
    bool has_params = false, has_iq = false;
    std::vector<VASliceParameterBufferHEVC> pending_slices;
    std::vector<HevcSlice> slices;
};
struct HevcState {
    bool started = false;
    bool no_rasl_output = false;
    std::vector<uint8_t> vps, sps;
    std::map<unsigned, std::vector<uint8_t>> pps;
};
inline bool is_hevc(VAProfile p) {
    return p == VAProfileHEVCMain || p == VAProfileHEVCMain10;
}
inline bool is_vp9(VAProfile p) {
    return p == VAProfileVP9Profile0 || p == VAProfileVP9Profile2;
}
inline bool is_10bit(VAProfile p) {
    return p == VAProfileHEVCMain10 || p == VAProfileVP9Profile2;
}
struct Vp9Slice {
    VASliceParameterBufferVP9 params{};
    std::vector<uint8_t> bytes;
};
struct Vp9Picture {
    VADecPictureParameterBufferVP9 params{};
    bool has_params = false;
    std::vector<VASliceParameterBufferVP9> pending_slices;
    std::vector<Vp9Slice> slices;
};
std::vector<uint8_t> vp9_bitstream(VAProfile profile, const Vp9Picture &picture);
std::vector<uint8_t> h264_bitstream(VAProfile profile, const Picture &picture, H264State &state);
std::vector<uint8_t> hevc_bitstream(VAProfile profile, const HevcPicture &picture,
                                    HevcState &state);

class Decoder {
    int fd_ = -1;
    FILE *dump_ = nullptr;
    std::mutex mutex_;
    bool output_on_ = false, capture_on_ = false;
    bool source_change_ = false, last_ = false;
    unsigned width_, height_, pool_size_, fourcc_;
    uint64_t next_token_ = 1;
    std::vector<std::shared_ptr<Memory>> output_, capture_;
    std::map<uint64_t, std::shared_ptr<Surface>> pending_;
    SurfaceCopier copier_;
    void allocate(unsigned type, unsigned count, std::vector<std::shared_ptr<Memory>> &pool);
    void configure_capture();
    void pump(int timeout_ms);
    void requeue();

  public:
    Decoder(const std::string &device, unsigned width, unsigned height, unsigned surfaces,
            VAProfile profile, int render_fd);
    ~Decoder();
    void submit(const std::vector<uint8_t> &bytes, const std::shared_ptr<Surface> &surface);
    void sync(const std::shared_ptr<Surface> &surface);
    void finish();
    void refresh();
};
std::string find_device();
std::string find_encoder_device();
struct EncodePicture {
    VAEncPictureParameterBufferH264 params{};
    VAEncPictureParameterBufferHEVC hevc_params{};
    bool has_params = false;
    std::vector<VAEncSliceParameterBufferH264> slices;
    std::vector<VAEncSliceParameterBufferHEVC> hevc_slices;
    VABufferID coded_buffer(bool hevc) const {
        return hevc ? hevc_params.coded_buf : params.coded_buf;
    }
    VASurfaceID reconstruction(bool hevc) const {
        return hevc ? hevc_params.decoded_curr_pic.picture_id : params.CurrPic.picture_id;
    }
};
// Supported VAEncMiscParameterBufferQualityLevel range: 1 = best quality,
// VA_ENC_QUALITY_RANGE = fastest, 0 = driver default. See encoder.cpp for how
// the level is mapped without a firmware quality/speed preset.
constexpr unsigned VA_ENC_QUALITY_RANGE = 4;
struct EncodeSettings {
    VAEncSequenceParameterBufferH264 sequence{};
    VAEncSequenceParameterBufferHEVC hevc_sequence{};
    bool hevc = false;
    bool has_sequence = false;
    unsigned rate_control = VA_RC_CQP;
    unsigned bitrate = 0, peak_bitrate = 0, min_qp = 1, max_qp = 51;
    unsigned quality = 0;
    unsigned icq_quality = 0;
    unsigned fps_num = 30, fps_den = 1;
};
void render_encode_buffer(EncodeSettings &settings, EncodePicture &picture, const Buffer &buffer);
class Encoder {
    struct Impl;
    std::unique_ptr<Impl> impl_;

  public:
    Encoder(const std::string &device, unsigned width, unsigned height, VAProfile profile,
            const EncodeSettings &settings, const EncodePicture &picture, int render_fd);
    ~Encoder();
    std::shared_ptr<EncodeTask> encode(const EncodeSettings &settings, const EncodePicture &picture,
                                       const std::shared_ptr<Surface> &input,
                                       const std::shared_ptr<Buffer> &output);
    bool sync(const std::shared_ptr<EncodeTask> &task, uint64_t timeout);
    void refresh();
    void finish();
};
void copy_surface(Memory &destination, Memory &source, unsigned width, unsigned height);
void wait_surface_access(const Memory &memory, short events);
bool vpp_supported(VAProfile profile, VAEntrypoint entrypoint);
void vpp_caps(VAProcPipelineCaps &caps);
struct VppPicture {
    std::shared_ptr<Surface> source;
    VARectangle region{};
};
class Vpp {
    std::shared_ptr<void> runtime_;

  public:
    Vpp();
    void scale(const VppPicture &picture, const std::shared_ptr<Surface> &target);
};
VppPicture vpp_picture(const Buffer &buffer, const std::shared_ptr<Surface> &source,
                       const Surface &target);
struct Context {
    VAProfile profile;
    VAProfile decode_profile;
    unsigned decode_surfaces = 0;
    VAEntrypoint entrypoint = VAEntrypointVLD;
    unsigned width, height;
    std::shared_ptr<Decoder> decoder;
    std::shared_ptr<Encoder> encoder;
    std::unique_ptr<Vpp> vpp;
    VppPicture vpp_picture;
    EncodeSettings encode_settings;
    EncodePicture encode_picture;
    std::shared_ptr<Surface> target;
    Picture picture;
    H264State h264;
    HevcPicture hevc_picture;
    HevcState hevc;
    Vp9Picture vp9_picture;
};
struct Config {
    VAProfile profile;
    VAEntrypoint entrypoint;
    unsigned rate_control = VA_RC_CQP;
};
struct Driver {
    std::recursive_mutex mutex;
    std::string device;
    std::string encoder_device;
    uint32_t next_id = 1;
    std::map<uint32_t, Config> configs;
    std::map<uint32_t, std::shared_ptr<Context>> contexts;
    std::map<uint32_t, std::shared_ptr<Surface>> surfaces;
    std::map<uint32_t, std::shared_ptr<Buffer>> buffers;
    std::map<uint32_t, VAImage> images;
};
template <class T> auto lookup(std::map<uint32_t, T> &table, uint32_t id, VAStatus status) {
    auto it = table.find(id);
    if (it == table.end())
        throw Error(status, "invalid object ID");
    return it->second;
}
} // namespace irisva
