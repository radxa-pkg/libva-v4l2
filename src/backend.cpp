// SPDX-License-Identifier: MIT
#include "internal.hpp"
#include <drm_fourcc.h>
#include <linux/dma-buf.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <climits>

namespace irisva {
Buffer::~Buffer() {
    if (acquired_fd >= 0) {
        close(acquired_fd);
        --parent->acquired_handles;
    }
    if (mapped) {
        dma_buf_sync sync{DMA_BUF_SYNC_END | DMA_BUF_SYNC_RW};
        ioctl(memory->fd, DMA_BUF_IOCTL_SYNC, &sync);
        --parent->mapped_images;
    }
    if (memory)
        --parent->derived_images;
}
uint8_t *Buffer::map() {
    if (type == VAEncCodedBufferType) {
        check(coded_ready, "coded buffer is not ready", VA_STATUS_ERROR_OPERATION_FAILED);
        coded_mapped = true;
        return reinterpret_cast<uint8_t *>(&coded);
    }
    if (!memory)
        return data.data();
    check(!parent->acquired_handles, "surface handle acquired", VA_STATUS_ERROR_SURFACE_BUSY);
    auto pointer = memory->map();
    if (!mapped) {
        dma_buf_sync sync{DMA_BUF_SYNC_START | DMA_BUF_SYNC_RW};
        check(ioctl(memory->fd, DMA_BUF_IOCTL_SYNC, &sync) == 0,
              "DMA-BUF map synchronization failed");
        mapped = true;
        ++parent->mapped_images;
    }
    return pointer;
}
void Buffer::unmap() {
    coded_mapped = false;
    if (!mapped)
        return;
    dma_buf_sync sync{DMA_BUF_SYNC_END | DMA_BUF_SYNC_RW};
    check(ioctl(memory->fd, DMA_BUF_IOCTL_SYNC, &sync) == 0,
          "DMA-BUF unmap synchronization failed");
    mapped = false;
    --parent->mapped_images;
}
struct ImageMapping {
    std::shared_ptr<Buffer> buffer;
    bool was_mapped;
    uint8_t *data;
    explicit ImageMapping(std::shared_ptr<Buffer> b)
        : buffer(std::move(b)), was_mapped(buffer->mapped), data(buffer->map()) {}
    void finish() {
        if (!was_mapped)
            buffer->unmap();
    }
    ~ImageMapping() {
        try {
            finish();
        } catch (...) {
        }
    }
};
template <class F> VAStatus guard(VADriverContextP ctx, F f) noexcept {
    try {
        auto &d = *static_cast<Driver *>(ctx->pDriverData);
        std::lock_guard<std::recursive_mutex> lock(d.mutex);
        return f(d);
    } catch (const Error &e) {
        std::fprintf(stderr, "iris-vaapi: %s (VA status %#x)\n", e.what(), e.status);
        return e.status;
    } catch (const std::bad_alloc &) {
        return VA_STATUS_ERROR_ALLOCATION_FAILED;
    } catch (const std::exception &e) {
        std::fprintf(stderr, "iris-vaapi: %s\n", e.what());
        return VA_STATUS_ERROR_OPERATION_FAILED;
    }
}
#define API(name, args, ...)                                                                       \
    static VAStatus name args {                                                                    \
        return guard(ctx, [&](Driver &d) -> VAStatus { __VA_ARGS__ });                             \
    }
static bool supported(const Driver &d, VAProfile p) {
    return p == VAProfileH264ConstrainedBaseline || p == VAProfileH264Main ||
           p == VAProfileH264High || is_hevc(p) || is_vp9(p) || (is_av1(p) && d.av1);
}
static bool encode_supported(const Driver &d, VAProfile p, VAEntrypoint e) {
    return !d.encoder_device.empty() && e == VAEntrypointEncSlice &&
           (p == VAProfileH264ConstrainedBaseline || p == VAProfileH264Main ||
            p == VAProfileH264High || p == VAProfileHEVCMain);
}
static unsigned encode_attribute(VAConfigAttribType type, VAProfile profile) {
    (void)profile; // HEVC attributes are only available in libva >= 1.13.
    switch (type) {
#if VA_CHECK_VERSION(1, 13, 0)
    case VAConfigAttribEncHEVCFeatures: {
        if (profile != VAProfileHEVCMain)
            return VA_ATTRIB_NOT_SUPPORTED;
        // Iris generates the parameter sets and chooses coding tools itself.
        VAConfigAttribValEncHEVCFeatures features{};
        features.bits.sao = VA_FEATURE_REQUIRED;
        features.bits.temporal_mvp = VA_FEATURE_REQUIRED;
        features.bits.cu_qp_delta = VA_FEATURE_SUPPORTED;
        features.bits.deblocking_filter_disable = VA_FEATURE_SUPPORTED;
        return features.value;
    }
    case VAConfigAttribEncHEVCBlockSizes: {
        if (profile != VAProfileHEVCMain)
            return VA_ATTRIB_NOT_SUPPORTED;
        VAConfigAttribValEncHEVCBlockSizes sizes{};
        sizes.bits.log2_max_coding_tree_block_size_minus3 = 2;
        sizes.bits.log2_min_coding_tree_block_size_minus3 = 2;
        sizes.bits.log2_max_luma_transform_block_size_minus2 = 3;
        return sizes.value;
    }
#endif
    case VAConfigAttribRTFormat:
        return VA_RT_FORMAT_YUV420;
    case VAConfigAttribRateControl:
        return VA_RC_CQP | VA_RC_CBR | VA_RC_VBR;
    case VAConfigAttribEncPackedHeaders:
        // The stateful encoder emits sequence headers in-band, but clients
        // such as FFmpeg only write container extradata (avcC/hvcC) when the
        // driver accepts packed sequence headers.
        return VA_ENC_PACKED_HEADER_SEQUENCE;
    case VAConfigAttribEncInterlaced:
        return VA_ENC_INTERLACED_NONE;
    case VAConfigAttribEncMaxRefFrames:
        return 1; // One past reference, no future references.
    case VAConfigAttribEncMaxSlices:
        return 1;
    case VAConfigAttribEncSliceStructure:
        return VA_ENC_SLICE_STRUCTURE_EQUAL_ROWS;
    case VAConfigAttribEncQualityRange:
        return 1;
    case VAConfigAttribPredictionDirection:
        return VA_PREDICTION_DIRECTION_PREVIOUS;
    case VAConfigAttribMaxPictureWidth:
        return 3840;
    case VAConfigAttribMaxPictureHeight:
        return 2160;
    default:
        return VA_ATTRIB_NOT_SUPPORTED;
    }
}
static unsigned profile_format(VAProfile p) {
    if (p == VAProfileVP9Profile2)
        return VA_RT_FORMAT_YUV420_10;
    // HEVC Main10 and AV1 Main accept both 8-bit and 10-bit surfaces.
    if (p == VAProfileHEVCMain10 || is_av1(p))
        return VA_RT_FORMAT_YUV420 | VA_RT_FORMAT_YUV420_10;
    return VA_RT_FORMAT_YUV420;
}
static unsigned render_formats(VAProfile profile, VAEntrypoint entrypoint) {
    return entrypoint == VAEntrypointVideoProc
               ? (VA_RT_FORMAT_YUV420 | VA_RT_FORMAT_YUV420_10 | VA_RT_FORMAT_RGB32)
               : profile_format(profile);
}
static unsigned profile_fourcc(VAProfile p) {
    return is_10bit(p) || is_av1(p) ? VA_FOURCC_P010 : VA_FOURCC_NV12;
}
static auto surface(Driver &d, VASurfaceID id) {
    return lookup(d.surfaces, id, VA_STATUS_ERROR_INVALID_SURFACE);
}
static auto context(Driver &d, VAContextID id) {
    return lookup(d.contexts, id, VA_STATUS_ERROR_INVALID_CONTEXT);
}
static auto buffer(Driver &d, VABufferID id) {
    return lookup(d.buffers, id, VA_STATUS_ERROR_INVALID_BUFFER);
}
static VAStatus synchronize_encode(const std::shared_ptr<EncodeTask> &task,
                                   uint64_t timeout = VA_TIMEOUT_INFINITE) {
    if (!task)
        return VA_STATUS_SUCCESS;
    if (task->pending) {
        auto encoder = task->encoder.lock();
        check(bool(encoder), "surface encoder no longer exists", VA_STATUS_ERROR_INVALID_CONTEXT);
        if (!encoder->sync(task, timeout))
            return VA_STATUS_ERROR_TIMEDOUT;
    }
    check(task->status == VA_STATUS_SUCCESS, "asynchronous encoding failed", task->status);
    return VA_STATUS_SUCCESS;
}
static void synchronize(Driver &d, const std::shared_ptr<Surface> &s) {
    if (auto decoder = s->decoder.lock()) {
        // Keep the decoder and surface alive while allowing other VA submissions.
        d.mutex.unlock();
        try {
            decoder->sync(s);
        } catch (...) {
            d.mutex.lock();
            throw;
        }
        d.mutex.lock();
    }
    synchronize_encode(s->encode_task);
    check(!s->pending, "surface decoder no longer exists", VA_STATUS_ERROR_INVALID_CONTEXT);
    if (s->status)
        throw Error(s->status, "surface contains a decoding error");
}
static VAImageFormat image_format(unsigned fourcc) {
    VAImageFormat f{};
    f.fourcc = fourcc;
    f.byte_order = VA_LSB_FIRST;
    f.bits_per_pixel = fourcc == VA_FOURCC_P010 ? 24 : 12;
    if (is_rgb(fourcc)) {
        f.bits_per_pixel = f.depth = 32;
        f.red_mask = fourcc == VA_FOURCC_BGRA ? 0xff0000 : 0xff;
        f.green_mask = 0xff00;
        f.blue_mask = fourcc == VA_FOURCC_BGRA ? 0xff : 0xff0000;
        f.alpha_mask = 0xff000000;
    }
    return f;
}
static VAImage create_image(Driver &d, unsigned fourcc, unsigned w, unsigned h) {
    check(w && h && w <= 8192 && h <= 8192, "invalid image dimensions",
          VA_STATUS_ERROR_INVALID_PARAMETER);
    check(fourcc == VA_FOURCC_NV12 || fourcc == VA_FOURCC_I420 || fourcc == VA_FOURCC_P010 ||
              is_rgb(fourcc),
          "unsupported image format", VA_STATUS_ERROR_INVALID_IMAGE_FORMAT);
    VAImage image{};
    image.image_id = d.next_id++;
    image.buf = d.next_id++;
    image.format = image_format(fourcc);
    image.width = w;
    image.height = h;
    image.pitches[0] = ((w + 1) & ~1u) * (fourcc == VA_FOURCC_P010 ? 2 : 1);
    image.offsets[1] = image.pitches[0] * h;
    unsigned chroma_h = (h + 1) / 2;
    if (is_rgb(fourcc)) {
        image.num_planes = 1;
        image.pitches[0] = w * 4;
        image.offsets[1] = 0;
        image.data_size = image.pitches[0] * h;
    } else if (fourcc != VA_FOURCC_I420) {
        image.num_planes = 2;
        image.pitches[1] = image.pitches[0];
        image.data_size = image.offsets[1] + image.pitches[1] * chroma_h;
    } else {
        image.num_planes = 3;
        image.pitches[1] = image.pitches[2] = image.pitches[0] / 2;
        image.offsets[2] = image.offsets[1] + image.pitches[1] * chroma_h;
        image.data_size = image.offsets[2] + image.pitches[2] * chroma_h;
    }
    auto b = std::make_shared<Buffer>();
    b->type = VAImageBufferType;
    b->element_size = image.data_size;
    b->elements = 1;
    b->data.resize(image.data_size);
    d.buffers[image.buf] = b;
    d.images[image.image_id] = image;
    return image;
}

static std::shared_ptr<Memory> allocate_undecoded_surface(VADriverContextP ctx, const Surface &s) {
    // mpv probes GPU import before creating a decoder context. Use an actual
    // linear MSM GEM allocation for these otherwise unbacked VA surfaces.
    // The PRIME fd owns the storage after the temporary GEM handle is closed.
    // If exported before decode, keep this allocation for the surface's
    // lifetime. A consumer may keep its fd without calling export again.
    check(ctx->drm_state, "surface allocation requires a DRM display");
    int fd = static_cast<drm_state *>(ctx->drm_state)->fd;
    return allocate_surface(fd, s.width, s.height, s.fourcc);
}

API(query_profiles, (VADriverContextP ctx, VAProfile *out, int *count), (void)d;
    check(out && count, "null profiles", VA_STATUS_ERROR_INVALID_PARAMETER);
    out[0] = VAProfileH264ConstrainedBaseline; out[1] = VAProfileH264Main;
    out[2] = VAProfileH264High; out[3] = VAProfileHEVCMain; out[4] = VAProfileHEVCMain10;
    out[5] = VAProfileVP9Profile0; out[6] = VAProfileVP9Profile2; *count = 7;
    if (d.av1) out[(*count)++] = VAProfileAV1Profile0;
    if (vpp_supported(VAProfileNone, VAEntrypointVideoProc)) out[(*count)++] = VAProfileNone;
    return VA_STATUS_SUCCESS;)
API(query_entrypoints, (VADriverContextP ctx, VAProfile profile, VAEntrypoint *out, int *count),
    (void)d;
    check(supported(d, profile) || vpp_supported(profile, VAEntrypointVideoProc),
          "unsupported profile", VA_STATUS_ERROR_UNSUPPORTED_PROFILE);
    check(out && count, "null entrypoints", VA_STATUS_ERROR_INVALID_PARAMETER);
    out[0] = profile == VAProfileNone ? VAEntrypointVideoProc : VAEntrypointVLD; *count = 1;
    if (encode_supported(d, profile, VAEntrypointEncSlice)) out[(*count)++] = VAEntrypointEncSlice;
    return VA_STATUS_SUCCESS;)
API(
    get_attributes,
    (VADriverContextP ctx, VAProfile profile, VAEntrypoint entry, VAConfigAttrib *attrs, int count),
    (void)d;
    check(supported(d, profile) || vpp_supported(profile, VAEntrypointVideoProc),
          "unsupported profile", VA_STATUS_ERROR_UNSUPPORTED_PROFILE);
    check((supported(d, profile) && entry == VAEntrypointVLD) || vpp_supported(profile, entry) ||
              encode_supported(d, profile, entry),
          "unsupported entrypoint", VA_STATUS_ERROR_UNSUPPORTED_ENTRYPOINT);
    for (int i = 0; i < count; ++i) {
        if (entry == VAEntrypointVideoProc) {
            attrs[i].value = attrs[i].type == VAConfigAttribRTFormat
                                 ? render_formats(profile, entry)
                             : (attrs[i].type == VAConfigAttribMaxPictureWidth ||
                                attrs[i].type == VAConfigAttribMaxPictureHeight)
                                 ? 8192
                                 : VA_ATTRIB_NOT_SUPPORTED;
            continue;
        }
        if (entry == VAEntrypointEncSlice) {
            attrs[i].value = encode_attribute(attrs[i].type, profile);
            continue;
        }
        switch (attrs[i].type) {
        case VAConfigAttribRTFormat:
            attrs[i].value = profile_format(profile);
            break;
        case VAConfigAttribDecAV1Features:
            attrs[i].value = is_av1(profile) ? 0 : VA_ATTRIB_NOT_SUPPORTED;
            break;
        case VAConfigAttribDecSliceMode:
            attrs[i].value = VA_DEC_SLICE_MODE_NORMAL;
            break;
        case VAConfigAttribMaxPictureWidth:
        case VAConfigAttribMaxPictureHeight:
            attrs[i].value = 8192;
            break;
        default:
            attrs[i].value = VA_ATTRIB_NOT_SUPPORTED;
        }
    } return VA_STATUS_SUCCESS;)
API(
    create_config,
    (VADriverContextP ctx, VAProfile p, VAEntrypoint e, VAConfigAttrib *attrs, int count,
     VAConfigID *id),
    check(supported(d, p) || vpp_supported(p, VAEntrypointVideoProc), "unsupported profile",
          VA_STATUS_ERROR_UNSUPPORTED_PROFILE);
    check((supported(d, p) && e == VAEntrypointVLD) || vpp_supported(p, e) ||
              encode_supported(d, p, e),
          "unsupported entrypoint", VA_STATUS_ERROR_UNSUPPORTED_ENTRYPOINT);
    Config config{p, e, VA_RC_CQP}; check(id, "null config ID", VA_STATUS_ERROR_INVALID_PARAMETER);
    for (int i = 0; i < count; ++i) {
        if (e == VAEntrypointEncSlice) {
            const auto type = attrs[i].type;
            check(type == VAConfigAttribRTFormat || type == VAConfigAttribRateControl ||
                      type == VAConfigAttribEncPackedHeaders || type == VAConfigAttribEncInterlaced,
                  "unsupported encoder config attribute", VA_STATUS_ERROR_ATTR_NOT_SUPPORTED);
            auto value = encode_attribute(type, p);
            check(!(attrs[i].value & ~value) &&
                      (type != VAConfigAttribRTFormat || attrs[i].value == VA_RT_FORMAT_YUV420),
                  "unsupported encoder attribute", VA_STATUS_ERROR_ATTR_NOT_SUPPORTED);
            if (attrs[i].type == VAConfigAttribRateControl) {
                check(attrs[i].value == VA_RC_CQP || attrs[i].value == VA_RC_CBR ||
                          attrs[i].value == VA_RC_VBR,
                      "select one encoder rate control mode", VA_STATUS_ERROR_ATTR_NOT_SUPPORTED);
                config.rate_control = attrs[i].value;
            }
        } else if (attrs[i].type == VAConfigAttribRTFormat)
            check(attrs[i].value && !(attrs[i].value & ~render_formats(p, e)),
                  "unsupported render format", VA_STATUS_ERROR_UNSUPPORTED_RT_FORMAT);
        else if (e == VAEntrypointVLD && attrs[i].type == VAConfigAttribDecSliceMode)
            check(attrs[i].value == VA_DEC_SLICE_MODE_NORMAL, "unsupported slice mode",
                  VA_STATUS_ERROR_ATTR_NOT_SUPPORTED);
        else if (e == VAEntrypointVLD && is_av1(p) && attrs[i].type == VAConfigAttribDecAV1Features)
            check(!attrs[i].value, "AV1 large scale tiles unsupported",
                  VA_STATUS_ERROR_ATTR_NOT_SUPPORTED);
        else
            throw Error(VA_STATUS_ERROR_ATTR_NOT_SUPPORTED, "unsupported config attribute");
    } *id = d.next_id++;
    d.configs[*id] = config; return VA_STATUS_SUCCESS;)
API(destroy_config, (VADriverContextP ctx, VAConfigID id),
    check(d.configs.erase(id), "invalid config", VA_STATUS_ERROR_INVALID_CONFIG);
    return VA_STATUS_SUCCESS;)
API(query_config,
    (VADriverContextP ctx, VAConfigID id, VAProfile *p, VAEntrypoint *e, VAConfigAttrib *a, int *n),
    auto config = lookup(d.configs, id, VA_STATUS_ERROR_INVALID_CONFIG);
    *p = config.profile; *e = config.entrypoint;
    a[0] = {VAConfigAttribRTFormat, render_formats(*p, *e)}; *n = 1;
    if (*e == VAEntrypointEncSlice) a[(*n)++] = {VAConfigAttribRateControl, config.rate_control};
    return VA_STATUS_SUCCESS;)
API(
    query_surfaces, (VADriverContextP ctx, VAConfigID id, VASurfaceAttrib *attrs, unsigned *count),
    auto config = lookup(d.configs, id, VA_STATUS_ERROR_INVALID_CONFIG);
    auto profile = config.profile;
    check(count, "null surface attribute count", VA_STATUS_ERROR_INVALID_PARAMETER);
    VASurfaceAttrib a[9]{};
    VASurfaceAttribType types[] = {VASurfaceAttribPixelFormat, VASurfaceAttribMinWidth,
                                   VASurfaceAttribMinHeight, VASurfaceAttribMaxWidth,
                                   VASurfaceAttribMaxHeight, VASurfaceAttribMemoryType};
    unsigned values[] = {profile_fourcc(profile), 128, 128, 8192, 8192,
                         VA_SURFACE_ATTRIB_MEM_TYPE_VA | VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME |
                             VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2};
    if (config.entrypoint == VAEntrypointVideoProc) {
        values[1] = 2;
        values[2] = 2;
    } if (config.entrypoint == VAEntrypointEncSlice) {
        values[3] = 3840;
        values[4] = 2160;
    } for (unsigned i = 0; i < 6; ++i) {
        a[i].type = types[i];
        a[i].flags = VA_SURFACE_ATTRIB_GETTABLE;
        if (i == 0 || i == 5)
            a[i].flags |= VA_SURFACE_ATTRIB_SETTABLE;
        a[i].value.type = VAGenericValueTypeInteger;
        a[i].value.value.i = values[i];
    } unsigned n = 6;
    if (config.entrypoint == VAEntrypointVideoProc) {
        a[n] = a[0];
        a[n++].value.value.i = VA_FOURCC_P010;
        a[n] = a[0];
        a[n++].value.value.i = VA_FOURCC_BGRA;
        a[n] = a[0];
        a[n++].value.value.i = VA_FOURCC_RGBA;
    } else if (values[0] != VA_FOURCC_NV12 && (profile_format(profile) & VA_RT_FORMAT_YUV420)) {
        a[n] = a[0];
        a[n++].value.value.i = VA_FOURCC_NV12;
    } if (!attrs) {
        *count = n;
        return VA_STATUS_SUCCESS;
    } if (*count < n) {
        *count = n;
        return VA_STATUS_ERROR_MAX_NUM_EXCEEDED;
    } std::memcpy(attrs, a, n * sizeof(*a));
    *count = n; return VA_STATUS_SUCCESS;)
// Normalize the legacy descriptor before applying the shared DMA-BUF checks.
// PRIME1 has no explicit modifier, so only unflagged linear buffers are accepted.
static VADRMPRIMESurfaceDescriptor
prime_descriptor(const VASurfaceAttribExternalBuffers &external) {
    bool rgb = is_rgb(external.pixel_format);
    check(external.num_buffers == 1 && external.buffers && external.buffers[0] <= INT_MAX &&
              external.num_planes == (rgb ? 1u : 2u) && !external.flags,
          "PRIME1 requires one linear DMA-BUF", VA_STATUS_ERROR_INVALID_PARAMETER);
    VADRMPRIMESurfaceDescriptor desc{};
    desc.fourcc = external.pixel_format;
    desc.width = external.width;
    desc.height = external.height;
    desc.num_objects = desc.num_layers = 1;
    desc.objects[0].fd = static_cast<int>(external.buffers[0]);
    desc.objects[0].size = external.data_size;
    desc.objects[0].drm_format_modifier = DRM_FORMAT_MOD_LINEAR;
    auto &layer = desc.layers[0];
    layer.drm_format = external.pixel_format == VA_FOURCC_P010 ? DRM_FORMAT_P010 : DRM_FORMAT_NV12;
    if (rgb)
        layer.drm_format =
            external.pixel_format == VA_FOURCC_BGRA ? DRM_FORMAT_ARGB8888 : DRM_FORMAT_ABGR8888;
    layer.num_planes = rgb ? 1 : 2;
    for (unsigned i = 0; i < layer.num_planes; ++i) {
        layer.offset[i] = external.offsets[i];
        layer.pitch[i] = external.pitches[i];
    }
    return desc;
}
static std::shared_ptr<Memory> import_prime(const VADRMPRIMESurfaceDescriptor &desc, unsigned width,
                                            unsigned height, unsigned fourcc) {
    check(desc.fourcc == fourcc && desc.width == width && desc.height == height &&
              desc.num_objects == 1 && desc.objects[0].fd >= 0 &&
              desc.objects[0].drm_format_modifier == DRM_FORMAT_MOD_LINEAR,
          "import requires matching single-object linear surface",
          VA_STATUS_ERROR_INVALID_PARAMETER);
    bool p010 = fourcc == VA_FOURCC_P010;
    unsigned offsets[2]{}, pitches[2]{};
    bool rgb = is_rgb(fourcc);
    if (rgb) {
        const auto &l = desc.layers[0];
        check(desc.num_layers == 1 && l.num_planes == 1 && !l.object_index[0] &&
                  l.drm_format ==
                      (fourcc == VA_FOURCC_BGRA ? DRM_FORMAT_ARGB8888 : DRM_FORMAT_ABGR8888) &&
                  !(l.offset[0] & 3) && !(l.pitch[0] & 3) && l.pitch[0] >= width * 4 &&
                  uint64_t(l.offset[0]) + uint64_t(height - 1) * l.pitch[0] + width * 4 <=
                      desc.objects[0].size,
              "invalid RGB DMA-BUF layout", VA_STATUS_ERROR_INVALID_PARAMETER);
        offsets[0] = l.offset[0];
        pitches[0] = l.pitch[0];
    } else if (desc.num_layers == 1) {
        const auto &l = desc.layers[0];
        check(l.drm_format == (p010 ? DRM_FORMAT_P010 : DRM_FORMAT_NV12) && l.num_planes == 2 &&
                  !l.object_index[0] && !l.object_index[1],
              "invalid composed DMA-BUF layer", VA_STATUS_ERROR_INVALID_PARAMETER);
        for (unsigned i = 0; i < 2; ++i) {
            offsets[i] = l.offset[i];
            pitches[i] = l.pitch[i];
        }
    } else {
        check(desc.num_layers == 2 &&
                  desc.layers[0].drm_format == (p010 ? DRM_FORMAT_R16 : DRM_FORMAT_R8) &&
                  desc.layers[1].drm_format == (p010 ? DRM_FORMAT_GR1616 : DRM_FORMAT_GR88),
              "invalid separate DMA-BUF layers", VA_STATUS_ERROR_INVALID_PARAMETER);
        for (unsigned i = 0; i < 2; ++i) {
            const auto &l = desc.layers[i];
            check(l.num_planes == 1 && !l.object_index[0], "invalid DMA-BUF plane",
                  VA_STATUS_ERROR_INVALID_PARAMETER);
            offsets[i] = l.offset[0];
            pitches[i] = l.pitch[0];
        }
    }
    unsigned sample_bytes = p010 ? 2 : 1;
    unsigned row_bytes = ((width + 1) & ~1u) * sample_bytes;
    check(rgb || (pitches[0] >= row_bytes && pitches[0] == pitches[1] &&
                  pitches[0] % sample_bytes == 0 && offsets[0] % sample_bytes == 0 &&
                  offsets[1] >= offsets[0] && (offsets[1] - offsets[0]) % pitches[0] == 0 &&
                  (offsets[1] - offsets[0]) / pitches[0] >= height &&
                  uint64_t(offsets[1]) + uint64_t((height + 1) / 2 - 1) * pitches[1] + row_bytes <=
                      desc.objects[0].size),
          "unsupported DMA-BUF plane layout", VA_STATUS_ERROR_INVALID_PARAMETER);
    auto m = std::make_shared<Memory>();
    m->fd = fcntl(desc.objects[0].fd, F_DUPFD_CLOEXEC, 0);
    check(m->fd >= 0, "duplicate imported DMA-BUF");
    // DMA-BUF llseek reports the real allocation length. Do not trust a descriptor
    // that would let a subsequent mmap/DSP access extend beyond the allocation.
    auto size = lseek(m->fd, 0, SEEK_END);
    check(size >= 0 && uint64_t(size) >= desc.objects[0].size,
          "DMA-BUF allocation smaller than descriptor", VA_STATUS_ERROR_INVALID_PARAMETER);
    m->size = desc.objects[0].size;
    m->fourcc = fourcc;
    m->width = width;
    m->height = height;
    m->stride = pitches[0];
    m->storage_height = rgb ? height : (offsets[1] - offsets[0]) / pitches[0];
    m->data_offset = offsets[0];
    m->origin = MemoryOrigin::Imported;
    return m;
}
API(
    create_surfaces,
    (VADriverContextP ctx, unsigned format, unsigned w, unsigned h, VASurfaceID *ids,
     unsigned count, VASurfaceAttrib *attrs, unsigned nattrs),
    check(format == VA_RT_FORMAT_YUV420 || format == VA_RT_FORMAT_YUV420_10 ||
              format == VA_RT_FORMAT_RGB32,
          "unsupported surface RT format", VA_STATUS_ERROR_UNSUPPORTED_RT_FORMAT);
    check(w && h && w <= 8192 && h <= 8192 && ids && count <= 256, "invalid surfaces",
          VA_STATUS_ERROR_INVALID_PARAMETER);
    check(!nattrs || attrs, "null surface attributes", VA_STATUS_ERROR_INVALID_PARAMETER);
    unsigned fourcc = format == VA_RT_FORMAT_RGB32       ? VA_FOURCC_BGRA
                      : format == VA_RT_FORMAT_YUV420_10 ? VA_FOURCC_P010
                                                         : VA_FOURCC_NV12;
    unsigned memory_type = VA_SURFACE_ATTRIB_MEM_TYPE_VA; const void *external = nullptr;
    bool explicit_modifier = false; for (unsigned i = 0; i < nattrs; ++i) {
        if (!(attrs[i].flags & VA_SURFACE_ATTRIB_SETTABLE))
            continue;
#if VA_CHECK_VERSION(1, 13, 0)
        if (attrs[i].type == VASurfaceAttribDRMFormatModifiers) {
            check(attrs[i].value.type == VAGenericValueTypePointer && attrs[i].value.value.p,
                  "invalid modifier list", VA_STATUS_ERROR_INVALID_PARAMETER);
            const auto &list =
                *static_cast<const VADRMFormatModifierList *>(attrs[i].value.value.p);
            check(list.num_modifiers && list.modifiers &&
                      std::find(list.modifiers, list.modifiers + list.num_modifiers,
                                DRM_FORMAT_MOD_LINEAR) != list.modifiers + list.num_modifiers,
                  "only linear surface allocation is supported",
                  VA_STATUS_ERROR_ATTR_NOT_SUPPORTED);
            explicit_modifier = true;
            continue;
        }
#endif
        if (attrs[i].type == VASurfaceAttribExternalBufferDescriptor) {
            check(attrs[i].value.type == VAGenericValueTypePointer && attrs[i].value.value.p,
                  "invalid external descriptor", VA_STATUS_ERROR_INVALID_PARAMETER);
            external = attrs[i].value.value.p;
            continue;
        }
        check(attrs[i].value.type == VAGenericValueTypeInteger, "unsupported surface attribute",
              VA_STATUS_ERROR_ATTR_NOT_SUPPORTED);
        if (attrs[i].type == VASurfaceAttribPixelFormat) {
            unsigned requested = attrs[i].value.value.i;
            check(format == VA_RT_FORMAT_RGB32 ? is_rgb(requested) : requested == fourcc,
                  "surface pixel format does not match RT format",
                  VA_STATUS_ERROR_UNSUPPORTED_RT_FORMAT);
            fourcc = requested;
        } else if (attrs[i].type == VASurfaceAttribMemoryType)
            memory_type = attrs[i].value.value.i;
        else if (attrs[i].type != VASurfaceAttribUsageHint)
            throw Error(VA_STATUS_ERROR_ATTR_NOT_SUPPORTED, "unsupported surface attribute");
    } check(memory_type == VA_SURFACE_ATTRIB_MEM_TYPE_VA ||
                memory_type == VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME ||
                memory_type == VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2,
            "unsupported surface memory type", VA_STATUS_ERROR_UNSUPPORTED_MEMORY_TYPE);
    std::shared_ptr<Memory> imported; if (memory_type != VA_SURFACE_ATTRIB_MEM_TYPE_VA) {
        check(external && !explicit_modifier && count == 1,
              "PRIME import requires one descriptor/surface", VA_STATUS_ERROR_INVALID_PARAMETER);
        auto desc =
            memory_type == VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME
                ? prime_descriptor(*static_cast<const VASurfaceAttribExternalBuffers *>(external))
                : *static_cast<const VADRMPRIMESurfaceDescriptor *>(external);
        imported = import_prime(desc, w, h, fourcc);
    } else check(!external, "external descriptor without PRIME memory type",
                 VA_STATUS_ERROR_INVALID_PARAMETER);
    for (unsigned i = 0; i < count; ++i) {
        auto s = std::make_shared<Surface>();
        s->memory = imported;
        s->persistent_export = bool(imported);
        s->width = w;
        s->height = h;
        s->allocation_count = count;
        s->fourcc = fourcc;
        ids[i] = d.next_id++;
        d.surfaces[ids[i]] = s;
    } return VA_STATUS_SUCCESS;)
API(create_surfaces_legacy,
    (VADriverContextP ctx, int w, int h, int format, int n, VASurfaceID *ids), (void)d;
    return create_surfaces(ctx, format, w, h, ids, n, nullptr, 0);)
API(destroy_surfaces, (VADriverContextP ctx, VASurfaceID *ids, int count),
    for (int i = 0; i < count; ++i) check(!surface(d, ids[i])->acquired_handles,
                                          "surface handle acquired", VA_STATUS_ERROR_SURFACE_BUSY);
    for (int i = 0; i < count; ++i) d.surfaces.erase(ids[i]); return VA_STATUS_SUCCESS;)
API(
    create_context,
    (VADriverContextP ctx, VAConfigID id, int w, int h, int flags, VASurfaceID *targets, int count,
     VAContextID *result),
    (void)flags;
    auto config = lookup(d.configs, id, VA_STATUS_ERROR_INVALID_CONFIG);
    bool processing = config.entrypoint == VAEntrypointVideoProc;
    check(((processing && w >= 0 && h >= 0) || (w >= 128 && h >= 128)) && w <= 8192 && h <= 8192 &&
              result,
          "invalid context dimensions", VA_STATUS_ERROR_RESOLUTION_NOT_SUPPORTED);
    auto c = std::make_shared<Context>(); c->profile = config.profile;
    c->entrypoint = config.entrypoint; c->encode_settings.rate_control = config.rate_control;
    c->encode_settings.hevc = is_hevc(c->profile); if (c->entrypoint == VAEntrypointEncSlice)
        check(w <= 3840 && h <= 2160 && !(w & 1) && !(h & 1), "unsupported encoder dimensions",
              VA_STATUS_ERROR_RESOLUTION_NOT_SUPPORTED);
    for (int i = 0; i < count; ++i) surface(d, targets[i]); c->width = w; c->height = h;
    c->internal_profile = c->profile; c->decode_surfaces = std::max(0, count); if (processing) {
        check(ctx->drm_state, "VPP requires a DRM display", VA_STATUS_ERROR_INVALID_DISPLAY);
        c->vpp = std::make_unique<Vpp>(static_cast<drm_state *>(ctx->drm_state)->fd);
    } *result = d.next_id++;
    d.contexts[*result] = c; return VA_STATUS_SUCCESS;)
API(
    destroy_context, (VADriverContextP ctx, VAContextID id), auto c = context(d, id);
    d.contexts.erase(id); if (c->encoder) c->encoder->finish();
    if (!c->decoder) return VA_STATUS_SUCCESS;
    // FFmpeg can destroy its decoder context while downstream still owns the
    // last VA frames. Complete pending pictures before closing the V4L2 session;
    // exported DMA-BUF storage remains owned by those surfaces afterwards.
    d.mutex.unlock(); try { c->decoder->finish(); } catch (...) {
        d.mutex.lock();
        throw;
    } d.mutex.lock();
    return VA_STATUS_SUCCESS;)
API(create_buffer,
    (VADriverContextP ctx, VAContextID id, VABufferType type, unsigned size, unsigned count,
     void *data, VABufferID *result),
    auto c = context(d, id);
    check(result && size && count && uint64_t(size) * count <= 64 * 1024 * 1024,
          "invalid buffer size", VA_STATUS_ERROR_INVALID_PARAMETER);
    bool encoding = c->entrypoint == VAEntrypointEncSlice;
    check(c->entrypoint == VAEntrypointVideoProc ? (type == VAProcPipelineParameterBufferType ||
                                                    type == VAProcFilterParameterBufferType)
          : encoding
              ? (type == VAEncSequenceParameterBufferType ||
                 type == VAEncPictureParameterBufferType || type == VAEncSliceParameterBufferType ||
                 type == VAEncMiscParameterBufferType || type == VAEncCodedBufferType ||
                 type == VAEncPackedHeaderParameterBufferType ||
                 type == VAEncPackedHeaderDataBufferType)
              : (type == VAPictureParameterBufferType || type == VAIQMatrixBufferType ||
                 type == VASliceParameterBufferType || type == VASliceDataBufferType),
          "unsupported buffer type", VA_STATUS_ERROR_UNSUPPORTED_BUFFERTYPE);
    auto b = std::make_shared<Buffer>(); b->context_id = id; b->type = type; b->element_size = size;
    b->elements = count; b->data.resize(size_t(size) * count);
    if (data) std::memcpy(b->data.data(), data, b->data.size()); *result = d.next_id++;
    d.buffers[*result] = b; return VA_STATUS_SUCCESS;)
API(set_elements, (VADriverContextP ctx, VABufferID id, unsigned count), auto b = buffer(d, id);
    check(uint64_t(b->element_size) * count <= b->data.size(), "buffer elements exceed allocation",
          VA_STATUS_ERROR_INVALID_PARAMETER);
    b->elements = count; return VA_STATUS_SUCCESS;)
API(map_buffer, (VADriverContextP ctx, VABufferID id, void **data),
    check(data, "null mapped pointer", VA_STATUS_ERROR_INVALID_PARAMETER);
    auto b = buffer(d, id); synchronize_encode(b->encode_task); *data = b->map();
    return VA_STATUS_SUCCESS;)
API(sync_buffer, (VADriverContextP ctx, VABufferID id, uint64_t timeout),
    if (d.buffers.find(id) == d.buffers.end()) return VA_STATUS_ERROR_INVALID_BUFFER;
    auto b = buffer(d, id);
    check(b->type == VAEncCodedBufferType, "only coded buffers can be synchronized",
          VA_STATUS_ERROR_UNSUPPORTED_BUFFERTYPE);
    auto status = synchronize_encode(b->encode_task, timeout);
    if (status != VA_STATUS_SUCCESS) return status;
    check(b->coded_ready, "coded buffer has no completed picture",
          VA_STATUS_ERROR_OPERATION_FAILED);
    return VA_STATUS_SUCCESS;)
API(unmap_buffer, (VADriverContextP ctx, VABufferID id), buffer(d, id)->unmap();
    return VA_STATUS_SUCCESS;)
API(destroy_buffer, (VADriverContextP ctx, VABufferID id),
    check(buffer(d, id)->acquired_fd < 0, "buffer handle acquired", VA_STATUS_ERROR_SURFACE_BUSY);
    check(d.buffers.erase(id), "invalid buffer", VA_STATUS_ERROR_INVALID_BUFFER);
    return VA_STATUS_SUCCESS;)
API(buffer_info,
    (VADriverContextP ctx, VABufferID id, VABufferType *type, unsigned *size, unsigned *n),
    auto b = buffer(d, id);
    *type = b->type; *size = b->element_size; *n = b->elements; return VA_STATUS_SUCCESS;)
API(
    begin_picture, (VADriverContextP ctx, VAContextID id, VASurfaceID target),
    auto c = context(d, id);
    check(!c->target, "picture already open", VA_STATUS_ERROR_OPERATION_FAILED);
    auto s = surface(d, target);
    // Main10 and AV1 permit NV12 and P010. Select the decoder's bit depth
    // from the first target, including clients that create a context without
    // render targets. Keep it fixed for the lifetime of the decoder.
    if (c->entrypoint == VAEntrypointVLD && c->profile == VAProfileHEVCMain10 && !c->decoder)
        c->internal_profile = s->fourcc == VA_FOURCC_NV12 ? VAProfileHEVCMain : VAProfileHEVCMain10;
    if (c->entrypoint == VAEntrypointVLD && !c->decoder) {
        c->decode_fourcc = is_av1(c->profile) ? s->fourcc : profile_fourcc(c->internal_profile);
        check(c->decode_fourcc == VA_FOURCC_NV12 || c->decode_fourcc == VA_FOURCC_P010,
              "unsupported decoder surface format", VA_STATUS_ERROR_UNSUPPORTED_RT_FORMAT);
    }
    check(c->entrypoint == VAEntrypointVideoProc
              ? (s->fourcc == VA_FOURCC_NV12 || s->fourcc == VA_FOURCC_P010)
              : s->fourcc == (c->entrypoint == VAEntrypointVLD ? c->decode_fourcc
                                                            : profile_fourcc(c->internal_profile)),
          "surface format does not match context", VA_STATUS_ERROR_UNSUPPORTED_RT_FORMAT);
    // Defer decoder creation until the target surface format is known.
    if (c->entrypoint == VAEntrypointVLD && !c->decoder) {
        unsigned rt_format =
            s->fourcc == VA_FOURCC_NV12 ? VA_RT_FORMAT_YUV420 : VA_RT_FORMAT_YUV420_10;
        int render_fd = ctx->drm_state ? static_cast<drm_state *>(ctx->drm_state)->fd : -1;
        c->decoder = std::make_shared<Decoder>(d.device, c->width, c->height, c->decode_surfaces,
                                               c->internal_profile, rt_format, render_fd);
    } if (s->pending || (s->encode_task && s->encode_task->pending)) synchronize(d, s);
    check(!s->derived_images, "surface has derived images", VA_STATUS_ERROR_SURFACE_BUSY);
    if (c->entrypoint == VAEntrypointEncSlice) {
        synchronize(d, s);
        check(bool(s->memory), "encode input has no image data", VA_STATUS_ERROR_INVALID_SURFACE);
        c->target = s;
        c->encode_picture = EncodePicture{};
        return VA_STATUS_SUCCESS;
    } if (c->entrypoint == VAEntrypointVideoProc) {
        check(!s->acquired_handles && !s->mapped_images, "VPP target is in use",
              VA_STATUS_ERROR_SURFACE_BUSY);
        s->status = VA_STATUS_SUCCESS;
        c->target = s;
        c->vpp_pictures.clear();
        c->vpp_failed = false;
        return VA_STATUS_SUCCESS;
    } if (!s->persistent_export) s->memory.reset();
    s->status = VA_STATUS_SUCCESS; s->decoder = c->decoder; c->target = s; c->picture = Picture{};
    c->hevc_picture = HevcPicture{}; c->vp9_picture = Vp9Picture{};
    c->av1_picture = Av1Picture{}; return VA_STATUS_SUCCESS;)
template <class P> static void render_buffer(P &p, const std::shared_ptr<Buffer> &b) {
    using Parameter = typename std::decay_t<decltype(p.pending_slices)>::value_type;
    using SliceType = typename std::decay_t<decltype(p.slices)>::value_type;
    if (b->type == VAPictureParameterBufferType) {
        check(b->element_size >= sizeof(p.params) && b->elements == 1, "invalid picture parameters",
              VA_STATUS_ERROR_INVALID_BUFFER);
        std::memcpy(&p.params, b->data.data(), sizeof(p.params));
        p.has_params = true;
    } else if (b->type == VAIQMatrixBufferType) {
        if constexpr (std::is_same_v<P, Vp9Picture> || std::is_same_v<P, Av1Picture>) {
            throw Error(VA_STATUS_ERROR_UNSUPPORTED_BUFFERTYPE, "codec has no IQ matrix buffer");
        } else {
            check(b->element_size >= sizeof(p.iq) && b->elements == 1, "invalid IQ matrix",
                  VA_STATUS_ERROR_INVALID_BUFFER);
            std::memcpy(&p.iq, b->data.data(), sizeof(p.iq));
            p.has_iq = true;
        }
    } else if (b->type == VASliceParameterBufferType) {
        check(b->element_size >= sizeof(Parameter), "invalid slice parameters",
              VA_STATUS_ERROR_INVALID_BUFFER);
        for (unsigned j = 0; j < b->elements; ++j) {
            Parameter s;
            std::memcpy(&s, b->data.data() + size_t(j) * b->element_size, sizeof(s));
            p.pending_slices.push_back(s);
        }
    } else if (b->type == VASliceDataBufferType) {
        check(!p.pending_slices.empty(), "slice data missing parameters",
              VA_STATUS_ERROR_INVALID_BUFFER);
        for (const auto &s : p.pending_slices) {
            check(s.slice_data_flag == VA_SLICE_DATA_FLAG_ALL, "fragmented slice not implemented",
                  VA_STATUS_ERROR_UNIMPLEMENTED);
            size_t off = s.slice_data_offset, n = s.slice_data_size;
            check(off <= b->data.size() && n <= b->data.size() - off, "slice data range",
                  VA_STATUS_ERROR_INVALID_BUFFER);
            SliceType slice;
            slice.params = s;
            slice.bytes.assign(b->data.begin() + off, b->data.begin() + off + n);
            if (!std::is_same_v<P, Vp9Picture> && !std::is_same_v<P, Av1Picture> &&
                slice.bytes.size() >= 4 && !slice.bytes[0] && !slice.bytes[1]) {
                unsigned prefix = slice.bytes[2] == 1                      ? 3
                                  : !slice.bytes[2] && slice.bytes[3] == 1 ? 4
                                                                           : 0;
                slice.bytes.erase(slice.bytes.begin(), slice.bytes.begin() + prefix);
            }
            p.slices.push_back(std::move(slice));
        }
        p.pending_slices.clear();
    }
}
static std::shared_ptr<Buffer> processing_filter(Driver &driver, VAContextID id,
                                                 const VABufferID *filters, unsigned count) {
    check(count <= 1, "VPP supports one HDR filter", VA_STATUS_ERROR_UNSUPPORTED_FILTER);
    if (!count)
        return {};
    check(filters, "null VPP filter list", VA_STATUS_ERROR_INVALID_PARAMETER);
    auto filter = buffer(driver, filters[0]);
    check(filter->context_id == id, "VPP filter belongs to another context",
          VA_STATUS_ERROR_INVALID_BUFFER);
    vpp_filter(*filter);
    check(context(driver, id)->vpp->tone_mapping_supported(), "HDR tone mapping unavailable",
          VA_STATUS_ERROR_UNSUPPORTED_FILTER);
    return filter;
}
API(
    render_picture, (VADriverContextP ctx, VAContextID id, VABufferID *ids, int count),
    auto c = context(d, id);
    check(bool(c->target), "no open picture"); try {
        check(count >= 0 && (!count || ids), "invalid picture buffer list",
              VA_STATUS_ERROR_INVALID_PARAMETER);
        if (c->entrypoint == VAEntrypointVideoProc)
            check(!c->vpp_failed, "invalid VPP submission", VA_STATUS_ERROR_INVALID_PARAMETER);
        std::shared_ptr<Buffer> deferred_av1_data;
        if (c->entrypoint == VAEntrypointVLD && is_av1(c->profile) &&
            c->av1_picture.pending_slices.empty()) {
            // Chromium submits one AV1 data buffer before its tile parameters.
            // Consume it last, but preserve parameter/data pairing when a call
            // contains multiple data buffers (each has its own slice offsets).
            for (int i = 0; i < count; ++i) {
                auto b = buffer(d, ids[i]);
                if (b->type == VASliceParameterBufferType && !deferred_av1_data)
                    break;
                if (b->type != VASliceDataBufferType)
                    continue;
                if (deferred_av1_data) {
                    deferred_av1_data.reset();
                    break;
                }
                deferred_av1_data = b;
            }
        }
        for (int i = 0; i < count; ++i) {
            auto b = buffer(d, ids[i]);
            if (b == deferred_av1_data)
                continue;
            if (c->entrypoint == VAEntrypointVideoProc) {
                check(b->context_id == id && b->type == VAProcPipelineParameterBufferType &&
                          b->element_size >= sizeof(VAProcPipelineParameterBuffer) &&
                          b->elements == 1,
                      "invalid VPP pipeline buffer", VA_STATUS_ERROR_INVALID_BUFFER);
                check(c->vpp_pictures.size() < 16, "VPP supports at most 16 layers",
                      VA_STATUS_ERROR_UNIMPLEMENTED);
                VAProcPipelineParameterBuffer params{};
                std::memcpy(&params, b->data.data(), sizeof(params));
                auto source = surface(d, params.surface);
                auto filter = processing_filter(d, id, params.filters, params.num_filters);
                c->vpp_pictures.push_back(
                    vpp_picture(*b, source, *c->target, filter.get(), !c->vpp_pictures.empty()));
            } else if (c->entrypoint == VAEntrypointEncSlice) {
                check(b->context_id == id, "encode buffer belongs to another context",
                      VA_STATUS_ERROR_INVALID_BUFFER);
                render_encode_buffer(c->encode_settings, c->encode_picture, *b);
            } else if (is_av1(c->profile))
                render_buffer(c->av1_picture, b);
            else if (is_vp9(c->profile))
                render_buffer(c->vp9_picture, b);
            else if (is_hevc(c->profile))
                render_buffer(c->hevc_picture, b);
            else
                render_buffer(c->picture, b);
        }
        if (deferred_av1_data)
            render_buffer(c->av1_picture, deferred_av1_data);
    } catch (...) {
        if (c->entrypoint == VAEntrypointVideoProc) {
            c->vpp_pictures.clear();
            c->vpp_failed = true;
        }
        throw;
    } return VA_STATUS_SUCCESS;)
API(
    end_picture, (VADriverContextP ctx, VAContextID id), auto c = context(d, id);
    check(bool(c->target), "no open picture"); auto target = c->target; c->target.reset();
    if (c->entrypoint == VAEntrypointVideoProc) {
        auto pictures = std::move(c->vpp_pictures);
        c->vpp_pictures.clear();
        check(!c->vpp_failed && !pictures.empty(), "VPP pipeline parameters missing or rejected",
              VA_STATUS_ERROR_INVALID_PARAMETER);
        for (const auto &picture : pictures) {
            synchronize(d, picture.source);
            check(!picture.source->mapped_images && !picture.source->acquired_handles,
                  "VPP source is in use", VA_STATUS_ERROR_SURFACE_BUSY);
        }
        synchronize(d, target);
        check(!target->derived_images && !target->acquired_handles, "VPP surface is in use",
              VA_STATUS_ERROR_SURFACE_BUSY);
        try {
            c->vpp->process(pictures, target);
        } catch (const Error &e) {
            target->status = e.status;
            throw;
        }
        return VA_STATUS_SUCCESS;
    } if (c->entrypoint == VAEntrypointEncSlice) {
        check(c->encode_picture.has_params, "missing encode picture",
              VA_STATUS_ERROR_INVALID_PARAMETER);
        auto coded = buffer(d, c->encode_picture.coded_buffer(c->encode_settings.hevc));
        check(coded->type == VAEncCodedBufferType && coded->context_id == id,
              "invalid encode output buffer", VA_STATUS_ERROR_INVALID_BUFFER);
        auto reconstruction = surface(d, c->encode_picture.reconstruction(c->encode_settings.hevc));
        synchronize_encode(reconstruction->encode_task);
        if (!c->encoder) {
            check(ctx->drm_state, "encoding requires a DRM display");
            c->encoder = std::make_shared<Encoder>(
                d.encoder_device,
                c->encode_settings.hevc ? std::min(c->width, target->width) : c->width,
                c->encode_settings.hevc ? std::min(c->height, target->height) : c->height,
                c->profile, c->encode_settings, c->encode_picture,
                static_cast<drm_state *>(ctx->drm_state)->fd);
        }
        auto task = c->encoder->encode(c->encode_settings, c->encode_picture, target, coded);
        task->encoder = c->encoder;
        target->encode_task = task;
        reconstruction->encode_task = task;
        return VA_STATUS_SUCCESS;
    } try {
        // Do not cache parameter sets from a rejected picture: the firmware
        // has not received them. Commit only after successful submission.
        if (is_av1(c->profile)) {
            check(c->av1_picture.has_params, "AV1 picture parameters missing",
                  VA_STATUS_ERROR_INVALID_BUFFER);
            check(surface(d, c->av1_picture.params.current_frame) == target,
                  "AV1 current frame does not match target", VA_STATUS_ERROR_INVALID_SURFACE);
            unsigned fourcc = c->av1_picture.params.bit_depth_idx == 1 ? VA_FOURCC_P010 : VA_FOURCC_NV12;
            check(fourcc == target->fourcc,
                  "AV1 bit depth does not match target", VA_STATUS_ERROR_UNSUPPORTED_RT_FORMAT);
            auto next_state = c->av1;
            auto bytes = av1_bitstream(c->av1_picture, next_state, c->width, c->height);
            c->decoder->submit(bytes, target);
            c->av1 = std::move(next_state);
        } else if (is_vp9(c->profile)) {
            auto bytes = vp9_bitstream(c->profile, c->vp9_picture);
            c->decoder->submit(bytes, target);
        } else if (is_hevc(c->profile)) {
            auto next_state = c->hevc;
            auto bytes = hevc_bitstream(c->internal_profile, c->hevc_picture, next_state);
            c->decoder->submit(bytes, target);
            c->hevc = std::move(next_state);
        } else {
            auto next_state = c->h264;
            auto bytes = h264_bitstream(c->profile, c->picture, next_state);
            c->decoder->submit(bytes, target);
            c->h264 = std::move(next_state);
        }
        // Chromium's cached NativePixmap output does not call vaSyncSurface
        // after submission. Complete decode and copy before publishing success;
        // in particular, no completion fence exists for the CPU override.
        if (target->persistent_export)
            synchronize(d, target);
    } catch (const Error &e) {
        target->status = e.status;
        throw;
    } catch (...) {
        target->status = VA_STATUS_ERROR_DECODING_ERROR;
        throw;
    } return VA_STATUS_SUCCESS;)
API(sync_surface, (VADriverContextP ctx, VASurfaceID id), synchronize(d, surface(d, id));
    return VA_STATUS_SUCCESS;)
API(query_status, (VADriverContextP ctx, VASurfaceID id, VASurfaceStatus *status),
    check(status, "null surface status", VA_STATUS_ERROR_INVALID_PARAMETER);
    auto s = surface(d, id);
    if (s->pending) if (auto decoder = s->decoder.lock()) decoder->refresh();
    if (s->encode_task &&
        s->encode_task->pending) if (auto encoder = s->encode_task->encoder.lock())
        encoder->refresh();
    *status = s->pending || (s->encode_task && s->encode_task->pending) ? VASurfaceRendering
                                                                        : VASurfaceReady;
    return VA_STATUS_SUCCESS;)
API(query_error, (VADriverContextP ctx, VASurfaceID id, VAStatus status, void **info),
    surface(d, id);
    (void)status; *info = nullptr; return VA_STATUS_SUCCESS;)
API(query_images, (VADriverContextP ctx, VAImageFormat *out, int *count), (void)d;
    out[0] = image_format(VA_FOURCC_NV12); out[1] = image_format(VA_FOURCC_I420);
    out[2] = image_format(VA_FOURCC_P010); out[3] = image_format(VA_FOURCC_BGRA);
    out[4] = image_format(VA_FOURCC_RGBA); *count = 5; return VA_STATUS_SUCCESS;)
API(new_image, (VADriverContextP ctx, VAImageFormat *f, int w, int h, VAImage *out),
    *out = create_image(d, f->fourcc, w, h);
    return VA_STATUS_SUCCESS;)
API(destroy_image, (VADriverContextP ctx, VAImageID id),
    auto image = lookup(d.images, id, VA_STATUS_ERROR_INVALID_IMAGE);
    check(buffer(d, image.buf)->acquired_fd < 0, "image handle acquired",
          VA_STATUS_ERROR_SURFACE_BUSY);
    d.buffers.erase(image.buf); d.images.erase(id); return VA_STATUS_SUCCESS;)
API(
    derive_image, (VADriverContextP ctx, VASurfaceID id, VAImage *image),
    check(image, "null derived image", VA_STATUS_ERROR_INVALID_PARAMETER);
    auto s = surface(d, id); synchronize(d, s);
    check(!s->acquired_handles, "surface handle acquired", VA_STATUS_ERROR_SURFACE_BUSY);
    if (!s->memory) s->memory = allocate_undecoded_surface(ctx, *s); auto m = s->memory;
    VAImage result{}; result.image_id = d.next_id++; result.buf = d.next_id++;
    result.format = image_format(m->fourcc); result.width = s->width; result.height = s->height;
    result.data_size = m->size; result.num_planes = is_rgb(m->fourcc) ? 1 : 2;
    result.pitches[0] = result.pitches[1] = m->stride; result.offsets[0] = m->data_offset;
    result.offsets[1] = m->data_offset + m->stride * m->storage_height; if (is_rgb(m->fourcc)) {
        result.pitches[1] = 0;
        result.offsets[1] = 0;
    } auto b = std::make_shared<Buffer>();
    b->type = VAImageBufferType; b->element_size = result.data_size; b->elements = 1; b->parent = s;
    b->memory = m; ++s->derived_images; d.buffers[result.buf] = b;
    try { d.images[result.image_id] = result; } catch (...) {
        d.buffers.erase(result.buf);
        throw;
    } *image = result;
    return VA_STATUS_SUCCESS;)
API(
    put_image,
    (VADriverContextP ctx, VASurfaceID id, VAImageID image_id, int sx, int sy, unsigned sw,
     unsigned sh, int dx, int dy, unsigned dw, unsigned dh),
    auto s = surface(d, id);
    synchronize(d, s); auto image = lookup(d.images, image_id, VA_STATUS_ERROR_INVALID_IMAGE);
    check(sw == dw && sh == dh, "image scaling unsupported", VA_STATUS_ERROR_UNIMPLEMENTED);
    check(sw && sh && sx >= 0 && sy >= 0 && dx >= 0 && dy >= 0 &&
              (is_rgb(s->fourcc) || !((sx | sy | dx | dy | sw | sh) & 1)) &&
              uint64_t(sx) + sw <= image.width && uint64_t(sy) + sh <= image.height &&
              uint64_t(dx) + dw <= s->width && uint64_t(dy) + dh <= s->height,
          "invalid upload region", VA_STATUS_ERROR_INVALID_PARAMETER);
    check(!s->derived_images && !s->acquired_handles, "surface has derived images or handles",
          VA_STATUS_ERROR_SURFACE_BUSY);
    check((is_rgb(s->fourcc) || is_rgb(image.format.fourcc))
              ? s->fourcc == image.format.fourcc
              : (s->fourcc == VA_FOURCC_P010) == (image.format.fourcc == VA_FOURCC_P010),
          "image bit depth conversion unsupported", VA_STATUS_ERROR_INVALID_IMAGE_FORMAT);
    if (!s->memory) s->memory = allocate_undecoded_surface(ctx, *s); auto m = s->memory;
    auto source_buffer = buffer(d, image.buf); ImageMapping source(source_buffer);
    auto destination_buffer = std::make_shared<Buffer>(); destination_buffer->parent = s;
    destination_buffer->memory = m; ++s->derived_images;
    ImageMapping destination(destination_buffer);
    unsigned bytes = is_rgb(s->fourcc)             ? 4
                     : s->fourcc == VA_FOURCC_P010 ? 2
                                                   : 1;
    for (unsigned row = 0; row < sh; ++row)
        std::memcpy(destination.data + m->data_offset + (dy + row) * m->stride + dx * bytes,
                    source.data + image.offsets[0] + (sy + row) * image.pitches[0] + sx * bytes,
                    sw *bytes);
    for (unsigned row = 0; !is_rgb(s->fourcc) && row < sh / 2; ++row) {
        auto uv = destination.data + m->data_offset +
                  (m->storage_height + dy / 2 + row) * m->stride + dx * bytes;
        if (image.format.fourcc == VA_FOURCC_I420) {
            for (unsigned col = 0; col < sw / 2; ++col) {
                uv[2 * col] =
                    source
                        .data[image.offsets[1] + (sy / 2 + row) * image.pitches[1] + sx / 2 + col];
                uv[2 * col + 1] =
                    source
                        .data[image.offsets[2] + (sy / 2 + row) * image.pitches[2] + sx / 2 + col];
            }
        } else {
            std::memcpy(
                uv, source.data + image.offsets[1] + (sy / 2 + row) * image.pitches[1] + sx * bytes,
                sw * bytes);
        }
    } destination.finish();
    source.finish(); return VA_STATUS_SUCCESS;)
API(acquire_buffer_handle, (VADriverContextP ctx, VABufferID id, VABufferInfo *info),
    check(info, "null buffer handle info", VA_STATUS_ERROR_INVALID_PARAMETER);
    auto b = buffer(d, id);
    check(bool(b->memory), "only derived image buffers have DMA-BUF handles",
          VA_STATUS_ERROR_UNSUPPORTED_BUFFERTYPE);
    check(!info->mem_type || (info->mem_type & VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME),
          "buffer handle requires DRM PRIME", VA_STATUS_ERROR_UNSUPPORTED_MEMORY_TYPE);
    synchronize(d, b->parent); check(!b->parent->acquired_handles && !b->parent->mapped_images,
                                     "image is mapped or acquired", VA_STATUS_ERROR_SURFACE_BUSY);
    b->acquired_fd = fcntl(b->memory->fd, F_DUPFD_CLOEXEC, 0);
    check(b->acquired_fd >= 0, "duplicate image DMA-BUF fd"); ++b->parent->acquired_handles;
    if (!b->parent->token) b->parent->persistent_export = true; *info = {};
    info->handle = b->acquired_fd; info->type = b->type;
    info->mem_type = VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME; info->mem_size = b->memory->size;
    return VA_STATUS_SUCCESS;)
API(
    release_buffer_handle, (VADriverContextP ctx, VABufferID id), auto b = buffer(d, id);
    if (b->acquired_fd >= 0) {
        close(b->acquired_fd);
        b->acquired_fd = -1;
        --b->parent->acquired_handles;
    } return VA_STATUS_SUCCESS;)
API(
    get_image,
    (VADriverContextP ctx, VASurfaceID id, int x, int y, unsigned w, unsigned h,
     VAImageID image_id),
    auto s = surface(d, id);
    synchronize(d, s);
    check(!s->derived_images, "surface has derived images", VA_STATUS_ERROR_SURFACE_BUSY);
    auto image = lookup(d.images, image_id, VA_STATUS_ERROR_INVALID_IMAGE);
    check(x >= 0 && y >= 0 && (is_rgb(s->fourcc) || (!(x & 1) && !(y & 1))) && w <= image.width &&
              h <= image.height && uint64_t(x) + w <= s->width && uint64_t(y) + h <= s->height,
          "invalid image region", VA_STATUS_ERROR_INVALID_PARAMETER);
    auto out = buffer(d, image.buf); ImageMapping destination(out); if (!s->memory) {
        std::memset(destination.data, 0, image.data_size);
        destination.finish();
        return VA_STATUS_SUCCESS;
    } auto m = s->memory;
    check((is_rgb(m->fourcc) || is_rgb(image.format.fourcc))
              ? m->fourcc == image.format.fourcc
              : (m->fourcc == VA_FOURCC_P010) == (image.format.fourcc == VA_FOURCC_P010),
          "image bit depth conversion unsupported", VA_STATUS_ERROR_INVALID_IMAGE_FORMAT);
    unsigned bytes_per_sample = is_rgb(m->fourcc)             ? 4
                                : m->fourcc == VA_FOURCC_P010 ? 2
                                                              : 1;
    check(uint64_t(x) + w <= m->width && uint64_t(y) + h <= m->height,
          "image exceeds CAPTURE layout");
    uint8_t *base = m->map() + m->data_offset;
    dma_buf_sync sync{DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ};
    check(ioctl(m->fd, DMA_BUF_IOCTL_SYNC, &sync) == 0, "DMA-BUF read synchronization failed");
    for (unsigned row = 0; row < h; ++row)
        std::memcpy(destination.data + image.offsets[0] + row * image.pitches[0],
                    base + (y + row) * m->stride + x * bytes_per_sample, w *bytes_per_sample);
    uint8_t *uv = base + m->stride * m->storage_height + (y / 2) * m->stride + x * bytes_per_sample;
    for (unsigned row = 0; !is_rgb(m->fourcc) && row < (h + 1) / 2; ++row) {
        if (image.format.fourcc != VA_FOURCC_I420)
            std::memcpy(destination.data + image.offsets[1] + row * image.pitches[1],
                        uv + row * m->stride, ((w + 1) & ~1u) * bytes_per_sample);
        else
            for (unsigned col = 0; col < (w + 1) / 2; ++col) {
                destination.data[image.offsets[1] + row * image.pitches[1] + col] =
                    uv[row * m->stride + 2 * col];
                destination.data[image.offsets[2] + row * image.pitches[2] + col] =
                    uv[row * m->stride + 2 * col + 1];
            }
    } sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ;
    check(ioctl(m->fd, DMA_BUF_IOCTL_SYNC, &sync) == 0, "DMA-BUF read synchronization end failed");
    destination.finish(); return VA_STATUS_SUCCESS;)
API(
    export_surface,
    (VADriverContextP ctx, VASurfaceID id, uint32_t type, uint32_t flags, void *out),
    check(out, "null surface descriptor", VA_STATUS_ERROR_INVALID_PARAMETER);
    check(type == VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2, "export requires DRM PRIME2",
          VA_STATUS_ERROR_UNSUPPORTED_MEMORY_TYPE);
    auto s = surface(d, id); synchronize(d, s);
    // GPU producers such as Sunshine render into exported encoder input.
    // Independent storage is writable: Iris CAPTURE allocations
    // may still be decoder references and have cached CPU aliases.
    check(!(flags & VA_EXPORT_SURFACE_WRITE_ONLY) || !s->memory ||
              s->memory->origin == MemoryOrigin::MsmCoherent ||
              s->memory->origin == MemoryOrigin::MsmWriteCombined ||
              s->memory->origin == MemoryOrigin::DmaHeap ||
              s->memory->origin == MemoryOrigin::Imported,
          "writable export requires independent storage", VA_STATUS_ERROR_UNIMPLEMENTED);
    if (!s->memory) s->memory = allocate_undecoded_surface(ctx, *s); auto m = s->memory;
    auto &desc = *static_cast<VADRMPRIMESurfaceDescriptor *>(out); desc = {};
    desc.fourcc = m->fourcc; desc.width = s->width; desc.height = s->height; desc.num_objects = 1;
    desc.objects[0].fd = fcntl(m->fd, F_DUPFD_CLOEXEC, 0);
    check(desc.objects[0].fd >= 0, "duplicate DMA-BUF fd"); if (!s->token) {
        if (!s->persistent_export)
            trace("surface=%#x persistent GEM export (copy if later decoded)", id);
        s->persistent_export = true;
    } desc.objects[0]
                                                                .size = m->size;
    desc.objects[0].drm_format_modifier = DRM_FORMAT_MOD_LINEAR; if (is_rgb(m->fourcc)) {
        desc.num_layers = 1;
        desc.layers[0].drm_format =
            m->fourcc == VA_FOURCC_BGRA ? DRM_FORMAT_ARGB8888 : DRM_FORMAT_ABGR8888;
        desc.layers[0].num_planes = 1;
        desc.layers[0].pitch[0] = m->stride;
        desc.layers[0].offset[0] = m->data_offset;
    } else if (!(flags & VA_EXPORT_SURFACE_COMPOSED_LAYERS)) {
        desc.num_layers = 2;
        desc.layers[0].drm_format = m->fourcc == VA_FOURCC_P010 ? DRM_FORMAT_R16 : DRM_FORMAT_R8;
        desc.layers[1].drm_format =
            m->fourcc == VA_FOURCC_P010 ? DRM_FORMAT_GR1616 : DRM_FORMAT_GR88;
        for (unsigned i = 0; i < 2; ++i) {
            desc.layers[i].num_planes = 1;
            desc.layers[i].pitch[0] = m->stride;
            desc.layers[i].offset[0] = m->data_offset + (i ? m->stride * m->storage_height : 0);
        }
    } else {
        desc.num_layers = 1;
        desc.layers[0].drm_format = m->fourcc == VA_FOURCC_P010 ? DRM_FORMAT_P010 : DRM_FORMAT_NV12;
        desc.layers[0].num_planes = 2;
        desc.layers[0].pitch[0] = desc.layers[0].pitch[1] = m->stride;
        desc.layers[0].offset[0] = m->data_offset;
        desc.layers[0].offset[1] = m->data_offset + m->stride * m->storage_height;
    } return VA_STATUS_SUCCESS;)

API(
    query_vpp_filters,
    (VADriverContextP ctx, VAContextID id, VAProcFilterType *filters, unsigned *count),
    check(context(d, id)->entrypoint == VAEntrypointVideoProc, "not a VPP context",
          VA_STATUS_ERROR_INVALID_CONTEXT);
    check(count, "null filter count", VA_STATUS_ERROR_INVALID_PARAMETER);
    unsigned capacity = *count; *count = context(d, id)->vpp->tone_mapping_supported() ? 1 : 0;
    check(capacity >= *count, "filter list too small", VA_STATUS_ERROR_MAX_NUM_EXCEEDED);
    if (*count) {
        check(filters, "null filter list", VA_STATUS_ERROR_INVALID_PARAMETER);
        filters[0] = VAProcFilterHighDynamicRangeToneMapping;
    } return VA_STATUS_SUCCESS;)
API(query_vpp_filter_caps,
    (VADriverContextP ctx, VAContextID id, VAProcFilterType type, void *caps, unsigned *count),
    check(context(d, id)->entrypoint == VAEntrypointVideoProc, "not a VPP context",
          VA_STATUS_ERROR_INVALID_CONTEXT);
    check(count, "null filter caps count", VA_STATUS_ERROR_INVALID_PARAMETER);
    unsigned capacity = *count; *count = 0;
    check(type == VAProcFilterHighDynamicRangeToneMapping &&
              context(d, id)->vpp->tone_mapping_supported(),
          "unsupported VPP filter", VA_STATUS_ERROR_UNSUPPORTED_FILTER);
    *count = 1; check(capacity, "filter caps list too small", VA_STATUS_ERROR_MAX_NUM_EXCEEDED);
    check(caps, "null filter caps", VA_STATUS_ERROR_INVALID_PARAMETER);
    VAProcFilterCapHighDynamicRange hdr{}; hdr.metadata_type = VAProcHighDynamicRangeMetadataHDR10;
    hdr.caps_flag = VA_TONE_MAPPING_HDR_TO_SDR; std::memcpy(caps, &hdr, sizeof(hdr));
    return VA_STATUS_SUCCESS;)
API(query_vpp_caps,
    (VADriverContextP ctx, VAContextID id, VABufferID *filters, unsigned count,
     VAProcPipelineCaps *caps),
    check(context(d, id)->entrypoint == VAEntrypointVideoProc, "not a VPP context",
          VA_STATUS_ERROR_INVALID_CONTEXT);
    check(caps, "null pipeline caps", VA_STATUS_ERROR_INVALID_PARAMETER);
    auto filter = processing_filter(d, id, filters, count); vpp_caps(*caps, bool(filter));
    return VA_STATUS_SUCCESS;)

static VAStatus terminate(VADriverContextP ctx) {
    delete static_cast<Driver *>(ctx->pDriverData);
    ctx->pDriverData = nullptr;
    return VA_STATUS_SUCCESS;
}
} // namespace irisva

extern "C" VAStatus __vaDriverInit_1_0(VADriverContextP ctx) {
    using namespace irisva;
    try {
        auto d = std::make_unique<Driver>();
        d->device = find_device(d->av1);
        d->encoder_device = find_encoder_device();
        ctx->version_major = VA_MAJOR_VERSION;
        ctx->version_minor = VA_MINOR_VERSION;
        ctx->max_profiles = 9;
        ctx->max_entrypoints = 2;
        ctx->max_attributes = 16;
        // libva validates nonzero allocation bounds even when queries return no entries.
        ctx->max_image_formats = 5;
        ctx->max_subpic_formats = 1;
        ctx->max_display_attributes = 1;
        ctx->str_vendor = "Iris V4L2 stateful VA-API backend";
        auto &v = *ctx->vtable;
        if (ctx->vtable_vpp && vpp_supported(VAProfileNone, VAEntrypointVideoProc)) {
            auto &vpp = *ctx->vtable_vpp;
            vpp.version = VA_DRIVER_VTABLE_VPP_VERSION;
            vpp.vaQueryVideoProcFilters = query_vpp_filters;
            vpp.vaQueryVideoProcFilterCaps = query_vpp_filter_caps;
            vpp.vaQueryVideoProcPipelineCaps = query_vpp_caps;
        }
        v.vaTerminate = terminate;
        v.vaQueryConfigProfiles = query_profiles;
        v.vaQueryConfigEntrypoints = query_entrypoints;
        v.vaGetConfigAttributes = get_attributes;
        v.vaCreateConfig = create_config;
        v.vaDestroyConfig = destroy_config;
        v.vaQueryConfigAttributes = query_config;
        v.vaQuerySurfaceAttributes = query_surfaces;
        v.vaCreateSurfaces2 = create_surfaces;
        v.vaCreateSurfaces = create_surfaces_legacy;
        v.vaDestroySurfaces = destroy_surfaces;
        v.vaCreateContext = create_context;
        v.vaDestroyContext = destroy_context;
        v.vaCreateBuffer = create_buffer;
        v.vaBufferSetNumElements = set_elements;
        v.vaMapBuffer = map_buffer;
        v.vaUnmapBuffer = unmap_buffer;
        v.vaDestroyBuffer = destroy_buffer;
        v.vaBufferInfo = buffer_info;
        v.vaBeginPicture = begin_picture;
        v.vaRenderPicture = render_picture;
        v.vaEndPicture = end_picture;
        v.vaSyncSurface = sync_surface;
        v.vaSyncBuffer = sync_buffer;
        v.vaQuerySurfaceStatus = query_status;
        v.vaQuerySurfaceError = query_error;
        v.vaQueryImageFormats = query_images;
        v.vaCreateImage = new_image;
        v.vaDestroyImage = destroy_image;
        v.vaDeriveImage = derive_image;
        v.vaGetImage = get_image;
        v.vaExportSurfaceHandle = export_surface;
        v.vaAcquireBufferHandle = acquire_buffer_handle;
        v.vaReleaseBufferHandle = release_buffer_handle;
        v.vaPutImage = put_image;
        v.vaPutSurface = [](VADriverContextP, VASurfaceID, void *, short, short, unsigned short,
                            unsigned short, short, short, unsigned short, unsigned short,
                            VARectangle *, unsigned,
                            unsigned) { return VA_STATUS_ERROR_UNIMPLEMENTED; };
        v.vaQuerySubpictureFormats = [](VADriverContextP, VAImageFormat *, unsigned *,
                                        unsigned *n) {
            *n = 0;
            return VA_STATUS_SUCCESS;
        };
        v.vaQueryDisplayAttributes = [](VADriverContextP, VADisplayAttribute *, int *n) {
            *n = 0;
            return VA_STATUS_SUCCESS;
        };
        v.vaGetDisplayAttributes = [](VADriverContextP, VADisplayAttribute *, int) {
            return VA_STATUS_ERROR_ATTR_NOT_SUPPORTED;
        };
        v.vaSetDisplayAttributes = [](VADriverContextP, VADisplayAttribute *, int) {
            return VA_STATUS_ERROR_ATTR_NOT_SUPPORTED;
        };
        v.vaSetImagePalette = [](VADriverContextP, VAImageID, unsigned char *) {
            return VA_STATUS_ERROR_UNIMPLEMENTED;
        };
        v.vaCreateSubpicture = [](VADriverContextP, VAImageID, VASubpictureID *) {
            return VA_STATUS_ERROR_UNIMPLEMENTED;
        };
        v.vaDestroySubpicture = [](VADriverContextP, VASubpictureID) {
            return VA_STATUS_ERROR_UNIMPLEMENTED;
        };
        v.vaSetSubpictureImage = [](VADriverContextP, VASubpictureID, VAImageID) {
            return VA_STATUS_ERROR_UNIMPLEMENTED;
        };
        v.vaSetSubpictureChromakey = [](VADriverContextP, VASubpictureID, unsigned, unsigned,
                                        unsigned) { return VA_STATUS_ERROR_UNIMPLEMENTED; };
        v.vaSetSubpictureGlobalAlpha = [](VADriverContextP, VASubpictureID, float) {
            return VA_STATUS_ERROR_UNIMPLEMENTED;
        };
        v.vaAssociateSubpicture = [](VADriverContextP, VASubpictureID, VASurfaceID *, int, short,
                                     short, unsigned short, unsigned short, short, short,
                                     unsigned short, unsigned short,
                                     unsigned) { return VA_STATUS_ERROR_UNIMPLEMENTED; };
        v.vaDeassociateSubpicture = [](VADriverContextP, VASubpictureID, VASurfaceID *, int) {
            return VA_STATUS_ERROR_UNIMPLEMENTED;
        };
        trace("initialized on %s", d->device.c_str());
        ctx->pDriverData = d.release();
        return VA_STATUS_SUCCESS;
    } catch (const std::exception &e) {
        std::fprintf(stderr, "iris-vaapi: init: %s\n", e.what());
        return VA_STATUS_ERROR_OPERATION_FAILED;
    }
}
