// SPDX-License-Identifier: MIT
#include "internal.hpp"
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <glob.h>
#include <poll.h>
#include <cerrno>
#include <limits>
#include <array>

namespace irisva {
namespace {
constexpr unsigned OUTPUT = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
constexpr unsigned CAPTURE = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
int call(int fd, unsigned long command, void *arg) {
    int ret;
    do {
        ret = ioctl(fd, command, arg);
    } while (ret < 0 && errno == EINTR);
    return ret;
}
void checked(int fd, unsigned long command, void *arg, const char *name) {
    if (call(fd, command, arg) < 0)
        throw Error(VA_STATUS_ERROR_ENCODING_ERROR, std::string(name) + ": " + strerror(errno));
}
struct QueueBuffer {
    v4l2_plane plane{};
    v4l2_buffer buffer{};
    explicit QueueBuffer(unsigned type, unsigned index = 0, bool import = false) {
        buffer.type = type;
        buffer.index = index;
        buffer.memory = type == OUTPUT && import ? V4L2_MEMORY_DMABUF : V4L2_MEMORY_MMAP;
        buffer.length = 1;
        buffer.m.planes = &plane;
    }
};
template <class T> T parameter(const Buffer &b, size_t offset = 0) {
    check(b.elements == 1 && b.data.size() >= offset + sizeof(T), "short encode parameter buffer",
          VA_STATUS_ERROR_INVALID_BUFFER);
    T value;
    std::memcpy(&value, b.data.data() + offset, sizeof(value));
    return value;
}
int level_control(unsigned idc) {
    const unsigned levels[] = {10, 9,  11, 12, 13, 20, 21, 22, 30,
                               31, 32, 40, 41, 42, 50, 51, 52, 60};
    for (unsigned i = 0; i < std::size(levels); ++i)
        if (levels[i] == idc)
            return i;
    throw Error(VA_STATUS_ERROR_INVALID_PARAMETER, "unsupported H264 encode level");
}
int hevc_level_control(unsigned idc) {
    const unsigned levels[] = {30, 60, 63, 90, 93, 120, 123, 150, 153, 156, 180, 183, 186};
    for (unsigned i = 0; i < std::size(levels); ++i)
        if (levels[i] == idc)
            return i;
    throw Error(VA_STATUS_ERROR_INVALID_PARAMETER, "unsupported HEVC encode level");
}
std::array<int, 3> deblocking(const EncodePicture &p, bool hevc) {
    if (hevc) {
        const auto &slice = p.hevc_slices.front();
        bool disabled = slice.slice_fields.bits.slice_deblocking_filter_disabled_flag;
        int mode = slice.slice_fields.bits.slice_loop_filter_across_slices_enabled_flag ? 0 : 2;
        return {disabled ? 1 : mode, disabled ? 0 : slice.slice_beta_offset_div2,
                disabled ? 0 : slice.slice_tc_offset_div2};
    }
    const auto &slice = p.slices.front();
    // Offsets are absent from the slice header when filtering is disabled.
    return {slice.disable_deblocking_filter_idc,
            slice.disable_deblocking_filter_idc == 1 ? 0 : slice.slice_alpha_c0_offset_div2,
            slice.disable_deblocking_filter_idc == 1 ? 0 : slice.slice_beta_offset_div2};
}
void validate_h264(const EncodeSettings &s, const EncodePicture &p, unsigned width,
                   unsigned height) {
    check(s.has_sequence && p.has_params && p.slices.size() == 1,
          "encoding requires a sequence, picture and one slice", VA_STATUS_ERROR_INVALID_PARAMETER);
    const auto &seq = s.sequence;
    const auto &pic = p.params;
    const auto &slice = p.slices.front();
    check(!pic.pic_fields.bits.idr_pic_flag || slice.slice_type % 5 == 2, "IDR requires an I slice",
          VA_STATUS_ERROR_UNIMPLEMENTED);
    check(seq.picture_width_in_mbs == (width + 15) / 16 &&
              seq.picture_height_in_mbs == (height + 15) / 16,
          "encode sequence dimensions do not match context", VA_STATUS_ERROR_INVALID_PARAMETER);
    check(seq.seq_fields.bits.frame_mbs_only_flag && seq.seq_fields.bits.chroma_format_idc == 1 &&
              !seq.bit_depth_luma_minus8 && !seq.bit_depth_chroma_minus8 &&
              !seq.seq_fields.bits.seq_scaling_matrix_present_flag && seq.ip_period <= 1 &&
              seq.max_num_ref_frames <= 1,
          "only progressive 8-bit H264 I/P encoding is supported", VA_STATUS_ERROR_UNIMPLEMENTED);
    check(slice.macroblock_address == 0 &&
              slice.num_macroblocks ==
                  unsigned(seq.picture_width_in_mbs) * seq.picture_height_in_mbs &&
              (slice.slice_type % 5 == 0 || slice.slice_type % 5 == 2),
          "only one full-picture I or P slice is supported", VA_STATUS_ERROR_UNIMPLEMENTED);
    check(!pic.pic_fields.bits.weighted_pred_flag && !pic.pic_fields.bits.weighted_bipred_idc &&
              !pic.pic_fields.bits.pic_scaling_matrix_present_flag && !pic.chroma_qp_index_offset &&
              !pic.second_chroma_qp_index_offset && !pic.num_ref_idx_l0_active_minus1 &&
              !slice.num_ref_idx_l0_active_minus1 && !slice.cabac_init_idc,
          "unsupported H264 encode picture features", VA_STATUS_ERROR_UNIMPLEMENTED);
    const auto filter = deblocking(p, false);
    check(filter[0] <= 2 && filter[1] >= -6 && filter[1] <= 6 && filter[2] >= -6 && filter[2] <= 6,
          "invalid H264 encode deblocking request", VA_STATUS_ERROR_INVALID_PARAMETER);
    check(!seq.frame_crop_left_offset && !seq.frame_crop_top_offset &&
              (!seq.frame_cropping_flag ||
               (uint64_t(width) + uint64_t(seq.frame_crop_right_offset) * 2 ==
                    unsigned(seq.picture_width_in_mbs) * 16 &&
                uint64_t(height) + uint64_t(seq.frame_crop_bottom_offset) * 2 ==
                    unsigned(seq.picture_height_in_mbs) * 16)),
          "unsupported encode cropping", VA_STATUS_ERROR_INVALID_PARAMETER);
}
void validate_hevc(const EncodeSettings &s, const EncodePicture &p, unsigned width,
                   unsigned height) {
    check(s.has_sequence && p.has_params && p.hevc_slices.size() == 1,
          "encoding requires a sequence, picture and one slice", VA_STATUS_ERROR_INVALID_PARAMETER);
    const auto &seq = s.hevc_sequence;
    const auto &pic = p.hevc_params;
    const auto &slice = p.hevc_slices.front();
    check(seq.general_profile_idc == 1 && seq.seq_fields.bits.chroma_format_idc == 1 &&
              !seq.seq_fields.bits.bit_depth_luma_minus8 &&
              !seq.seq_fields.bits.bit_depth_chroma_minus8 &&
              !seq.seq_fields.bits.separate_colour_plane_flag &&
              !seq.seq_fields.bits.pcm_enabled_flag && !seq.seq_fields.bits.hierachical_flag &&
              !seq.vui_fields.bits.field_seq_flag && seq.ip_period <= 1,
          "only progressive 8-bit HEVC Main I/P encoding is supported",
          VA_STATUS_ERROR_UNIMPLEMENTED);
    check(seq.log2_min_luma_coding_block_size_minus3 <= 3 &&
              seq.log2_diff_max_min_luma_coding_block_size <= 3 &&
              seq.log2_min_luma_coding_block_size_minus3 +
                      seq.log2_diff_max_min_luma_coding_block_size <=
                  3,
          "invalid HEVC coding block sizes", VA_STATUS_ERROR_INVALID_PARAMETER);
    trace("HEVC sequence=%ux%u visible=%ux%u min-cb-log2=%u ctu-diff=%u",
          seq.pic_width_in_luma_samples, seq.pic_height_in_luma_samples, width, height,
          seq.log2_min_luma_coding_block_size_minus3 + 3,
          seq.log2_diff_max_min_luma_coding_block_size);
    unsigned min_cb = 1u << (seq.log2_min_luma_coding_block_size_minus3 + 3);
    unsigned ctu = min_cb << seq.log2_diff_max_min_luma_coding_block_size;
    // HEVC VA sequence parameters lack a conformance window. The input surface
    // and context supply visible dimensions; the sequence may include CU padding.
    // Older FFmpeg clients align to 16 even when their SPS declares 8-pixel CUs.
    check(seq.pic_width_in_luma_samples >= width &&
              seq.pic_width_in_luma_samples - width < std::max(16u, min_cb) &&
              seq.pic_height_in_luma_samples >= height &&
              seq.pic_height_in_luma_samples - height < std::max(16u, min_cb),
          "HEVC sequence dimensions do not match input", VA_STATUS_ERROR_INVALID_PARAMETER);
    check(slice.slice_segment_address == 0 &&
              slice.num_ctu_in_slice == ((seq.pic_width_in_luma_samples + ctu - 1) / ctu) *
                                            ((seq.pic_height_in_luma_samples + ctu - 1) / ctu) &&
              (slice.slice_type == 1 || slice.slice_type == 2) &&
              (!pic.pic_fields.bits.idr_pic_flag || slice.slice_type == 2) &&
              pic.pic_fields.bits.coding_type == (slice.slice_type == 2 ? 1u : 2u),
          "only one full-picture HEVC I or P slice is supported", VA_STATUS_ERROR_UNIMPLEMENTED);
    check(!pic.pic_fields.bits.tiles_enabled_flag && !pic.num_tile_columns_minus1 &&
              !pic.num_tile_rows_minus1 && !pic.pic_fields.bits.entropy_coding_sync_enabled_flag &&
              !pic.pic_fields.bits.dependent_slice_segments_enabled_flag &&
              !pic.pic_fields.bits.weighted_pred_flag &&
              !pic.pic_fields.bits.weighted_bipred_flag &&
              !pic.pic_fields.bits.transquant_bypass_enabled_flag &&
              !pic.pic_fields.bits.scaling_list_data_present_flag &&
              !pic.num_ref_idx_l0_default_active_minus1 && !slice.num_ref_idx_l0_active_minus1 &&
              !pic.pps_cb_qp_offset && !pic.pps_cr_qp_offset && !slice.slice_cb_qp_offset &&
              !slice.slice_cr_qp_offset && !slice.slice_fields.bits.dependent_slice_segment_flag &&
              !slice.slice_fields.bits.colour_plane_id && !pic.ctu_max_bitsize_allowed,
          "unsupported HEVC encode picture features", VA_STATUS_ERROR_UNIMPLEMENTED);
    check(slice.slice_fields.bits.slice_deblocking_filter_disabled_flag <= 1 &&
              slice.slice_beta_offset_div2 >= -6 && slice.slice_beta_offset_div2 <= 6 &&
              slice.slice_tc_offset_div2 >= -6 && slice.slice_tc_offset_div2 <= 6 &&
              seq.general_tier_flag <= 1 &&
              (!seq.general_tier_flag || seq.general_level_idc >= 120),
          "invalid HEVC filter or tier", VA_STATUS_ERROR_INVALID_PARAMETER);
}
void validate(const EncodeSettings &s, const EncodePicture &p, unsigned width, unsigned height) {
    if (s.hevc)
        validate_hevc(s, p, width, height);
    else
        validate_h264(s, p, width, height);
    check(s.fps_num && s.fps_den && s.fps_num >= s.fps_den &&
              uint64_t(s.fps_num) <= uint64_t(s.fps_den) * 240,
          "unsupported encode frame rate", VA_STATUS_ERROR_INVALID_PARAMETER);
    check(s.min_qp >= 1 && s.max_qp <= 51 && s.min_qp <= s.max_qp, "invalid encode QP limits",
          VA_STATUS_ERROR_INVALID_PARAMETER);
    if (s.rate_control == VA_RC_ICQ)
        check(s.icq_quality >= 1 && s.icq_quality <= 51, "invalid ICQ quality factor",
              VA_STATUS_ERROR_INVALID_PARAMETER);
    if (s.rate_control != VA_RC_CQP && s.rate_control != VA_RC_ICQ)
        check(s.bitrate && s.bitrate <= s.peak_bitrate && s.peak_bitrate <= 245000000,
              "invalid encode bitrate", VA_STATUS_ERROR_INVALID_PARAMETER);
}
// VA quality levels: 1 = best .. VA_ENC_QUALITY_RANGE = fastest, 0 = default.
// Iris firmware has no speed/quality preset, so approximate the trade-off by
// capping the maximum QP the rate controller may reach. Only meaningful with
// rate control enabled; CQP pictures name their QP explicitly.
unsigned quality_max_qp(unsigned level, unsigned user_max_qp) {
    if (!level || level > VA_ENC_QUALITY_RANGE)
        return user_max_qp;
    unsigned ceiling = 51 - 7 * (VA_ENC_QUALITY_RANGE - level);
    return std::min(user_max_qp, ceiling);
}
// CQP and ICQ leave the firmware rate controller off (fixed/seeded QP).
bool qp_mode(unsigned rate_control) {
    return rate_control == VA_RC_CQP || rate_control == VA_RC_ICQ;
}
// AVBR is an average-bitrate mode with no peak constraint: closest firmware
// behavior is CBR. VBR and QVBR use the firmware VBR mode.
bool cbr_mode(unsigned rate_control) {
    return rate_control == VA_RC_CBR || rate_control == VA_RC_AVBR;
}
} // namespace

std::string find_encoder_device() {
    std::vector<std::string> paths;
    if (const char *p = std::getenv("IRIS_VAAPI_ENCODER_DEVICE"))
        paths.emplace_back(p);
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
            !strcmp(reinterpret_cast<char *>(cap.driver), "iris_driver") &&
            (cap.device_caps & V4L2_CAP_VIDEO_M2M_MPLANE)) {
            v4l2_fmtdesc fmt{};
            fmt.type = CAPTURE;
            while (!call(fd, VIDIOC_ENUM_FMT, &fmt)) {
                if (fmt.pixelformat == V4L2_PIX_FMT_H264)
                    match = true;
                ++fmt.index;
            }
        }
        close(fd);
        if (match)
            return path;
    }
    return {};
}

void render_encode_buffer(EncodeSettings &s, EncodePicture &p, const Buffer &b) {
    switch (b.type) {
    case VAEncSequenceParameterBufferType:
        if (s.hevc) {
            s.hevc_sequence = parameter<VAEncSequenceParameterBufferHEVC>(b);
            s.has_sequence = true;
            const auto &seq = s.hevc_sequence;
            if (seq.vui_parameters_present_flag &&
                seq.vui_fields.bits.vui_timing_info_present_flag) {
                s.fps_num = seq.vui_time_scale;
                s.fps_den = seq.vui_num_units_in_tick;
            }
            if (!s.bitrate)
                s.bitrate = s.peak_bitrate = seq.bits_per_second;
            break;
        }
        s.sequence = parameter<VAEncSequenceParameterBufferH264>(b);
        s.has_sequence = true;
        if (s.sequence.vui_parameters_present_flag &&
            s.sequence.vui_fields.bits.timing_info_present_flag) {
            check(s.sequence.num_units_in_tick <= std::numeric_limits<unsigned>::max() / 2,
                  "invalid encoder timing", VA_STATUS_ERROR_INVALID_PARAMETER);
            s.fps_num = s.sequence.time_scale;
            s.fps_den = s.sequence.num_units_in_tick * 2;
        }
        if (!s.bitrate)
            s.bitrate = s.peak_bitrate = s.sequence.bits_per_second;
        break;
    case VAEncPictureParameterBufferType:
        if (s.hevc)
            p.hevc_params = parameter<VAEncPictureParameterBufferHEVC>(b);
        else
            p.params = parameter<VAEncPictureParameterBufferH264>(b);
        p.has_params = true;
        break;
    case VAEncSliceParameterBufferType:
        if (s.hevc) {
            check(b.element_size >= sizeof(VAEncSliceParameterBufferHEVC),
                  "short HEVC encode slice", VA_STATUS_ERROR_INVALID_BUFFER);
            for (unsigned i = 0; i < b.elements; ++i) {
                VAEncSliceParameterBufferHEVC slice;
                std::memcpy(&slice, b.data.data() + size_t(i) * b.element_size, sizeof(slice));
                p.hevc_slices.push_back(slice);
            }
            break;
        }
        check(b.element_size >= sizeof(VAEncSliceParameterBufferH264), "short encode slice",
              VA_STATUS_ERROR_INVALID_BUFFER);
        for (unsigned i = 0; i < b.elements; ++i) {
            VAEncSliceParameterBufferH264 slice;
            std::memcpy(&slice, b.data.data() + size_t(i) * b.element_size, sizeof(slice));
            p.slices.push_back(slice);
        }
        break;
    case VAEncMiscParameterBufferType: {
        auto misc = parameter<VAEncMiscParameterBuffer>(b);
        const size_t off = offsetof(VAEncMiscParameterBuffer, data);
        switch (misc.type) {
        case VAEncMiscParameterTypeRateControl: {
            auto rc = parameter<VAEncMiscParameterRateControl>(b, off);
            check(!rc.rc_flags.bits.temporal_id && !rc.rc_flags.bits.reset &&
                      !rc.rc_flags.bits.enable_parallel_brc &&
                      !rc.rc_flags.bits.enable_dynamic_scaling && !rc.target_frame_size,
                  "unsupported encode rate control feature", VA_STATUS_ERROR_UNIMPLEMENTED);
            check(rc.target_percentage <= 100, "invalid target bitrate percentage",
                  VA_STATUS_ERROR_INVALID_PARAMETER);
            s.peak_bitrate = rc.bits_per_second;
            s.bitrate = (s.rate_control == VA_RC_VBR || s.rate_control == VA_RC_QVBR) &&
                                rc.target_percentage
                            ? uint64_t(rc.bits_per_second) * rc.target_percentage / 100
                            : rc.bits_per_second;
            s.min_qp = rc.min_qp ? rc.min_qp : 1;
            s.max_qp = rc.max_qp ? rc.max_qp : 51;
            // QVBR names its quality target as a QP ceiling: the encoder stops
            // spending bits once the target is met, so bitrate may overshoot.
            if (s.rate_control == VA_RC_QVBR && rc.quality_factor >= 1 &&
                rc.quality_factor <= 51)
                s.max_qp = std::min(s.max_qp, rc.quality_factor);
            // ICQ carries its target as an initial quality factor; this firmware
            // has no adaptive quality mode, so it seeds a fixed QP (see encode()).
            s.icq_quality = rc.ICQ_quality_factor;
            break;
        }
        case VAEncMiscParameterTypeFrameRate: {
            auto fr = parameter<VAEncMiscParameterFrameRate>(b, off);
            check(!fr.framerate_flags.bits.temporal_id, "temporal layers unsupported",
                  VA_STATUS_ERROR_UNIMPLEMENTED);
            s.fps_num = fr.framerate & 0xffff;
            s.fps_den = std::max(1u, fr.framerate >> 16);
            break;
        }
        case VAEncMiscParameterTypeQualityLevel: {
            auto quality = parameter<VAEncMiscParameterBufferQualityLevel>(b, off);
            check(quality.quality_level <= VA_ENC_QUALITY_RANGE, "unsupported encoder quality level",
                  VA_STATUS_ERROR_INVALID_PARAMETER);
            s.quality = quality.quality_level;
            break;
        }
        case VAEncMiscParameterTypeHRD:
            // Iris has no CPB size/fullness control. The firmware owns the
            // rate-control buffer; HRD values are advisory for this backend.
            (void)parameter<VAEncMiscParameterHRD>(b, off);
            break;
        default:
            throw Error(VA_STATUS_ERROR_UNIMPLEMENTED, "unsupported encode misc parameter");
        }
        break;
    }
    default:
        throw Error(VA_STATUS_ERROR_UNSUPPORTED_BUFFERTYPE, "unsupported encode buffer");
    }
}

struct Encoder::Impl {
    int fd = -1, render_fd = -1;
    bool import = false;
    struct Job {
        std::shared_ptr<EncodeTask> task;
        std::shared_ptr<Buffer> output;
        std::shared_ptr<Memory> memory;
        bool input_done = false, frame_done = false;
    };
    std::vector<std::shared_ptr<Memory>> staging;
    std::vector<std::shared_ptr<Job>> slots;
    std::map<uint64_t, std::shared_ptr<Job>> jobs;
    int current_qp = -1;
    std::unique_ptr<SurfaceCopier> copier;
    unsigned width, height, stride = 0, storage_height = 0, input_size = 0, input_count = 0;
    bool output_on = false, capture_on = false, failed = false;
    uint64_t token = 0;
    VASurfaceID last_reference = VA_INVALID_SURFACE;
    EncodeSettings settings;
    unsigned entropy;
    std::array<int, 3> filter{};
    std::vector<std::shared_ptr<Memory>> capture, raw;

    Impl(unsigned w, unsigned h, const EncodeSettings &s, unsigned cabac)
        : width(w), height(h), settings(s), entropy(cabac) {}
    ~Impl() {
        stop(VA_STATUS_ERROR_INVALID_CONTEXT);
        if (fd >= 0)
            close(fd);
        copier.reset();
        if (render_fd >= 0)
            close(render_fd);
    }
    void control(unsigned id, int value) {
        v4l2_control ctrl{};
        ctrl.id = id;
        ctrl.value = value;
        trace("encoder control id=%#x value=%d", id, value);
        checked(fd, VIDIOC_S_CTRL, &ctrl, "encoder S_CTRL");
    }
    void configure_deblocking() {
        const int ids[] = {settings.hevc ? V4L2_CID_MPEG_VIDEO_HEVC_LOOP_FILTER_MODE
                                         : V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_MODE,
                           settings.hevc ? V4L2_CID_MPEG_VIDEO_HEVC_LF_BETA_OFFSET_DIV2
                                         : V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_ALPHA,
                           settings.hevc ? V4L2_CID_MPEG_VIDEO_HEVC_LF_TC_OFFSET_DIV2
                                         : V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_BETA};
        bool supported = true;
        for (auto id : ids) {
            v4l2_queryctrl query{};
            query.id = id;
            if (call(fd, VIDIOC_QUERYCTRL, &query) < 0) {
                check(errno == EINVAL, "query encoder deblocking controls",
                      VA_STATUS_ERROR_ENCODING_ERROR);
                supported = false;
            } else if (query.flags & V4L2_CTRL_FLAG_DISABLED) {
                supported = false;
            }
        }
        if (!supported) {
            // Older Iris kernels only expose firmware's enabled, 0/0 default.
            // Never silently replace a non-default request with that default.
            check((filter[0] == 0 || (settings.hevc && filter[0] == 2)) && !filter[1] && !filter[2],
                  "deblocking request requires Iris loop-filter controls",
                  VA_STATUS_ERROR_UNIMPLEMENTED);
            return;
        }
        // Firmware can override slice offsets: on the tested SC8280XP, CQP
        // retains 0/0 and CBR/VBR can adjust them per frame. Forward the request
        // without rewriting headers that describe the firmware's actual filtering.
        const int modes[] = {V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_ENABLED,
                             V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_DISABLED,
                             V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_DISABLED_AT_SLICE_BOUNDARY};
        const int hevc_modes[] = {V4L2_MPEG_VIDEO_HEVC_LOOP_FILTER_MODE_ENABLED,
                                  V4L2_MPEG_VIDEO_HEVC_LOOP_FILTER_MODE_DISABLED,
                                  V4L2_MPEG_VIDEO_HEVC_LOOP_FILTER_MODE_DISABLED_AT_SLICE_BOUNDARY};
        control(ids[0], settings.hevc ? hevc_modes[filter[0]] : modes[filter[0]]);
        control(ids[1], filter[1]);
        control(ids[2], filter[2]);
        trace("encoder deblocking mode=%d alpha=%d beta=%d via V4L2", filter[0], filter[1],
              filter[2]);
    }
    void queue_capture(unsigned index) {
        QueueBuffer q(CAPTURE, index);
        q.plane.length = capture[index]->size;
        checked(fd, VIDIOC_QBUF, &q.buffer, "encoder QBUF CAPTURE");
    }
    void stop(VAStatus status) noexcept {
        if (fd >= 0) {
            unsigned type = OUTPUT;
            if (output_on)
                call(fd, VIDIOC_STREAMOFF, &type);
            output_on = false;
            type = CAPTURE;
            if (capture_on)
                call(fd, VIDIOC_STREAMOFF, &type);
            capture_on = false;
        }
        failed = true;
        // STREAMOFF must precede release of imported input storage.
        for (auto &[token, job] : jobs) {
            job->task->status = status;
            job->task->pending = false;
            job->output->coded_ready = false;
        }
        jobs.clear();
        for (auto &slot : slots)
            slot.reset();
    }
    void refresh() {
        if (jobs.empty())
            return;
        try {
            for (;;) {
                QueueBuffer q(OUTPUT, 0, import);
                if (call(fd, VIDIOC_DQBUF, &q.buffer) < 0) {
                    check(errno == EAGAIN, "encoder DQBUF OUTPUT failed",
                          VA_STATUS_ERROR_ENCODING_ERROR);
                    break;
                }
                check(q.buffer.index < slots.size() && slots[q.buffer.index] &&
                          !slots[q.buffer.index]->input_done &&
                          !(q.buffer.flags & V4L2_BUF_FLAG_ERROR),
                      "invalid encoder input completion", VA_STATUS_ERROR_ENCODING_ERROR);
                slots[q.buffer.index]->input_done = true;
                // OUTPUT DQBUF returns ownership of the raw slot even if the
                // corresponding coded output is still in the hardware pipeline.
                slots[q.buffer.index].reset();
            }
            for (;;) {
                QueueBuffer q(CAPTURE);
                if (call(fd, VIDIOC_DQBUF, &q.buffer) < 0) {
                    check(errno == EAGAIN, "encoder DQBUF CAPTURE failed",
                          VA_STATUS_ERROR_ENCODING_ERROR);
                    break;
                }
                check(q.buffer.index < capture.size() && !(q.buffer.flags & V4L2_BUF_FLAG_ERROR),
                      "encoder CAPTURE error", VA_STATUS_ERROR_ENCODING_ERROR);
                const auto &memory = capture[q.buffer.index];
                check(q.plane.data_offset <= q.plane.bytesused && q.plane.bytesused <= memory->size,
                      "invalid encoder bitstream range", VA_STATUS_ERROR_ENCODING_ERROR);
                unsigned size = q.plane.bytesused - q.plane.data_offset;
                if (size) {
                    // Recover the frame token from the timestamp written at QBUF.
                    // Timestamps carry the real frame period (see encode()); round
                    // to the nearest token because µs truncation loses <1µs.
                    const uint64_t ts_us = uint64_t(q.buffer.timestamp.tv_sec) * 1000000 +
                                           q.buffer.timestamp.tv_usec;
                    const uint64_t token =
                        (ts_us * settings.fps_num + 500000ull * settings.fps_den) /
                        (1000000ull * settings.fps_den);
                    auto it = jobs.find(token);
                    check(it != jobs.end() && !it->second->frame_done, "encoder timestamp mismatch",
                          VA_STATUS_ERROR_ENCODING_ERROR);
                    auto &job = *it->second;
                    auto &output = *job.output;
                    if (size > output.data.size()) {
                        output.coded.status = VA_CODED_BUF_STATUS_SLICE_OVERFLOW_MASK;
                        throw Error(VA_STATUS_ERROR_NOT_ENOUGH_BUFFER, "coded buffer too small");
                    }
                    std::memcpy(output.data.data(), memory->map() + q.plane.data_offset, size);
                    output.coded.size = size;
                    output.coded.buf = output.data.data();
                    job.frame_done = true;
                }
                queue_capture(q.buffer.index);
            }
            for (auto it = jobs.begin(); it != jobs.end();) {
                auto &job = *it->second;
                if (job.input_done && job.frame_done) {
                    job.output->coded_ready = true;
                    job.task->pending = false;
                    trace("encoded token=%llu bytes=%u", (unsigned long long)it->first,
                          job.output->coded.size);
                    it = jobs.erase(it);
                } else {
                    ++it;
                }
            }
        } catch (const Error &error) {
            stop(error.status);
            throw;
        } catch (...) {
            stop(VA_STATUS_ERROR_ENCODING_ERROR);
            throw;
        }
    }
    bool wait(const std::shared_ptr<EncodeTask> &task, uint64_t timeout) {
        const auto start = std::chrono::steady_clock::now();
        // Bound active waits, not time since submission: an idle client can
        // leave CAPTURE full until its next VA call requeues those buffers.
        // Also avoid overflowing time_point for VA_TIMEOUT_INFINITE.
        const auto end = start + std::chrono::nanoseconds(std::min<uint64_t>(timeout, 5000000000));
        while (task->pending) {
            refresh();
            if (!task->pending)
                break;
            const auto now = std::chrono::steady_clock::now();
            if (now >= end) {
                if (timeout <= 5000000000)
                    return false; // Caller timeout does not cancel or poison work.
                stop(VA_STATUS_ERROR_ENCODING_ERROR);
                throw Error(VA_STATUS_ERROR_ENCODING_ERROR, "encoder timed out");
            }
            bool input_pending = false;
            for (const auto &[token, job] : jobs)
                input_pending |= !job->input_done;
            auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(end - now);
            pollfd p{fd, short(POLLIN | (input_pending ? POLLOUT : 0)), 0};
            int ret = poll(&p, 1, std::min<int64_t>(100, remaining.count() + 1));
            if ((ret < 0 && errno != EINTR) ||
                (ret > 0 && (p.revents & (POLLERR | POLLHUP | POLLNVAL)))) {
                stop(VA_STATUS_ERROR_ENCODING_ERROR);
                throw Error(VA_STATUS_ERROR_ENCODING_ERROR, "encoder poll failed");
            }
        }
        check(task->status == VA_STATUS_SUCCESS, "asynchronous encoding failed", task->status);
        return true;
    }
    void finish() {
        while (!jobs.empty()) {
            auto task = jobs.begin()->second->task;
            check(wait(task, VA_TIMEOUT_INFINITE), "encoder timed out",
                  VA_STATUS_ERROR_ENCODING_ERROR);
        }
    }
};

Encoder::Encoder(const std::string &device, unsigned width, unsigned height, VAProfile profile,
                 const EncodeSettings &s, const EncodePicture &p, int render_fd) {
    // VA contexts may use macroblock-aligned dimensions (FFmpeg does this).
    // The sequence crop determines the visible dimensions sent to Iris.
    if (!s.hevc) {
        const auto &seq = s.sequence;
        unsigned coded_width = unsigned(seq.picture_width_in_mbs) * 16;
        unsigned coded_height = unsigned(seq.picture_height_in_mbs) * 16;
        check(s.has_sequence && coded_width >= width && coded_width - width < 16 &&
                  coded_height >= height && coded_height - height < 16 &&
                  uint64_t(seq.frame_crop_right_offset) * 2 < coded_width &&
                  uint64_t(seq.frame_crop_bottom_offset) * 2 < coded_height,
              "invalid encoder sequence dimensions", VA_STATUS_ERROR_INVALID_PARAMETER);
        if (seq.frame_cropping_flag) {
            coded_width -= seq.frame_crop_right_offset * 2;
            coded_height -= seq.frame_crop_bottom_offset * 2;
        }
        check(coded_width <= width && coded_height <= height && coded_width >= 128 &&
                  coded_height >= 128,
              "encode crop exceeds context", VA_STATUS_ERROR_INVALID_PARAMETER);
        width = coded_width;
        height = coded_height;
    }
    check(width >= 128 && height >= 128 && width <= 3840 && height <= 2160 && !(width & 1) &&
              !(height & 1),
          "unsupported visible encoder dimensions", VA_STATUS_ERROR_RESOLUTION_NOT_SUPPORTED);
    validate(s, p, width, height);
    auto impl = std::make_unique<Impl>(
        width, height, s, s.hevc ? 1 : p.params.pic_fields.bits.entropy_coding_mode_flag);
    auto &e = *impl;
    e.filter = deblocking(p, s.hevc);
    e.render_fd = fcntl(render_fd, F_DUPFD_CLOEXEC, 0);
    check(e.render_fd >= 0, "duplicate encoder DRM fd");
    int coherent = 0;
    int probe = irisva_msm_allocate(e.render_fd, 4096, &coherent);
    check(probe >= 0, "probe encoder shared memory", VA_STATUS_ERROR_ALLOCATION_FAILED);
    close(probe);
    e.import = coherent;
    e.fd = open(device.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
    check(e.fd >= 0, "open Iris encoder");
    v4l2_format fmt{};
    fmt.type = CAPTURE;
    fmt.fmt.pix_mp.width = width;
    fmt.fmt.pix_mp.height = height;
    fmt.fmt.pix_mp.pixelformat = s.hevc ? V4L2_PIX_FMT_HEVC : V4L2_PIX_FMT_H264;
    checked(e.fd, VIDIOC_S_FMT, &fmt, "encoder S_FMT coded format");
    check(fmt.fmt.pix_mp.pixelformat == (s.hevc ? V4L2_PIX_FMT_HEVC : V4L2_PIX_FMT_H264),
          "Iris does not support the requested encode format", VA_STATUS_ERROR_UNSUPPORTED_PROFILE);
    fmt = {};
    fmt.type = OUTPUT;
    fmt.fmt.pix_mp.width = width;
    fmt.fmt.pix_mp.height = height;
    fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
    fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
    checked(e.fd, VIDIOC_S_FMT, &fmt, "encoder S_FMT NV12");
    check(fmt.fmt.pix_mp.pixelformat == V4L2_PIX_FMT_NV12 && fmt.fmt.pix_mp.num_planes == 1,
          "unsupported Iris encoder input layout");
    e.stride = fmt.fmt.pix_mp.plane_fmt[0].bytesperline;
    e.storage_height = fmt.fmt.pix_mp.height;
    e.input_size = fmt.fmt.pix_mp.plane_fmt[0].sizeimage;
    v4l2_selection crop{};
    crop.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    crop.target = V4L2_SEL_TGT_CROP;
    crop.r.width = width;
    crop.r.height = height;
    checked(e.fd, VIDIOC_S_SELECTION, &crop, "encoder S_SELECTION");
    if (s.hevc) {
        e.control(V4L2_CID_MPEG_VIDEO_HEVC_PROFILE, V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN);
        e.control(V4L2_CID_MPEG_VIDEO_HEVC_LEVEL,
                  hevc_level_control(s.hevc_sequence.general_level_idc));
        v4l2_queryctrl tier{};
        tier.id = V4L2_CID_MPEG_VIDEO_HEVC_TIER;
        int ret = call(e.fd, VIDIOC_QUERYCTRL, &tier);
        check(ret == 0 || errno == EINVAL, "query HEVC tier control",
              VA_STATUS_ERROR_ENCODING_ERROR);
        if (ret == 0 && !(tier.flags & V4L2_CTRL_FLAG_DISABLED))
            e.control(tier.id, s.hevc_sequence.general_tier_flag);
        else
            check(!s.hevc_sequence.general_tier_flag, "HEVC High tier requires Iris tier control",
                  VA_STATUS_ERROR_UNIMPLEMENTED);
    } else {
        e.control(V4L2_CID_MPEG_VIDEO_H264_PROFILE,
                  profile == VAProfileH264High ? V4L2_MPEG_VIDEO_H264_PROFILE_HIGH
                  : profile == VAProfileH264Main
                      ? V4L2_MPEG_VIDEO_H264_PROFILE_MAIN
                      : V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE);
        e.control(V4L2_CID_MPEG_VIDEO_H264_LEVEL, level_control(s.sequence.level_idc));
        e.control(V4L2_CID_MPEG_VIDEO_H264_ENTROPY_MODE, e.entropy);
    }
    e.control(V4L2_CID_MPEG_VIDEO_B_FRAMES, 0);
    // Keyframes follow VA picture requests; prevent autonomous periodic IDRs.
    e.control(V4L2_CID_MPEG_VIDEO_GOP_SIZE, std::numeric_limits<int>::max());
    e.control(V4L2_CID_MPEG_VIDEO_HEADER_MODE, V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_1ST_FRAME);
    e.control(V4L2_CID_MPEG_VIDEO_PREPEND_SPSPPS_TO_IDR, 1);
    e.control(V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE, !qp_mode(s.rate_control));
    if (!qp_mode(s.rate_control)) {
        e.control(V4L2_CID_MPEG_VIDEO_BITRATE_MODE, cbr_mode(s.rate_control)
                                                        ? V4L2_MPEG_VIDEO_BITRATE_MODE_CBR
                                                        : V4L2_MPEG_VIDEO_BITRATE_MODE_VBR);
        e.control(V4L2_CID_MPEG_VIDEO_BITRATE, s.bitrate);
        e.control(V4L2_CID_MPEG_VIDEO_BITRATE_PEAK, s.peak_bitrate);
    }
    e.control(s.hevc ? V4L2_CID_MPEG_VIDEO_HEVC_MIN_QP : V4L2_CID_MPEG_VIDEO_H264_MIN_QP, s.min_qp);
    unsigned max_qp =
        qp_mode(s.rate_control) ? s.max_qp : quality_max_qp(s.quality, s.max_qp);
    if (max_qp != s.max_qp)
        trace("encoder quality level %u caps max QP %u -> %u", s.quality, s.max_qp, max_qp);
    e.control(s.hevc ? V4L2_CID_MPEG_VIDEO_HEVC_MAX_QP : V4L2_CID_MPEG_VIDEO_H264_MAX_QP, max_qp);
    e.configure_deblocking();
    for (unsigned type : {CAPTURE, OUTPUT}) {
        v4l2_streamparm parm{};
        parm.type = type;
        parm.parm.output.timeperframe.numerator = s.fps_den;
        parm.parm.output.timeperframe.denominator = s.fps_num;
        checked(e.fd, VIDIOC_S_PARM, &parm, "encoder S_PARM");
    }
    // Iris sizes coded buffers differently when rate control is disabled.
    // Refresh CAPTURE after controls, before allocating either queue.
    fmt = {};
    fmt.type = CAPTURE;
    fmt.fmt.pix_mp.width = width;
    fmt.fmt.pix_mp.height = height;
    fmt.fmt.pix_mp.pixelformat = s.hevc ? V4L2_PIX_FMT_HEVC : V4L2_PIX_FMT_H264;
    checked(e.fd, VIDIOC_S_FMT, &fmt, "encoder refresh CAPTURE format");
    for (unsigned type : {OUTPUT, CAPTURE}) {
        v4l2_requestbuffers req{};
        req.type = type;
        req.memory = type == OUTPUT && e.import ? V4L2_MEMORY_DMABUF : V4L2_MEMORY_MMAP;
        req.count = 4;
        checked(e.fd, VIDIOC_REQBUFS, &req, "encoder REQBUFS");
        check(req.count && req.count <= 64, "invalid encoder buffer count");
        if (type == OUTPUT) {
            e.input_count = req.count;
            if (e.import)
                continue;
        }
        for (unsigned i = 0; i < req.count; ++i) {
            QueueBuffer q(type, i);
            checked(e.fd, VIDIOC_QUERYBUF, &q.buffer, "encoder QUERYBUF");
            auto m = std::make_shared<Memory>();
            m->origin = MemoryOrigin::Iris;
            m->size = q.plane.length;
            void *ptr = mmap(nullptr, m->size, PROT_READ | PROT_WRITE, MAP_SHARED, e.fd,
                             q.plane.m.mem_offset);
            check(ptr != MAP_FAILED, "mmap encoder buffer");
            m->mapping = ptr;
            if (type == OUTPUT) {
                m->stride = e.stride;
                m->storage_height = e.storage_height;
                m->width = e.width;
                m->height = e.height;
                v4l2_exportbuffer exp{};
                exp.type = OUTPUT;
                exp.index = i;
                exp.flags = O_CLOEXEC | O_RDWR;
                checked(e.fd, VIDIOC_EXPBUF, &exp, "encoder EXPBUF OUTPUT");
                m->fd = exp.fd;
                e.raw.push_back(m);
            } else {
                e.capture.push_back(m);
                e.queue_capture(i);
            }
        }
    }
    e.slots.resize(e.input_count);
    e.staging.resize(e.input_count);
    unsigned type = OUTPUT;
    checked(e.fd, VIDIOC_STREAMON, &type, "encoder STREAMON OUTPUT");
    e.output_on = true;
    type = CAPTURE;
    checked(e.fd, VIDIOC_STREAMON, &type, "encoder STREAMON CAPTURE");
    e.capture_on = true;
    trace("encoder %s %ux%u stride=%u storage-height=%u size=%u RC=%u queue=%s", device.c_str(),
          width, height, e.stride, e.storage_height, e.input_size, s.rate_control,
          e.import ? "dmabuf" : "mmap");
    impl_ = std::move(impl);
}
Encoder::~Encoder() = default;

std::shared_ptr<EncodeTask> Encoder::encode(const EncodeSettings &s, const EncodePicture &p,
                                            const std::shared_ptr<Surface> &input,
                                            const std::shared_ptr<Buffer> &coded) {
    auto &output = *coded;
    auto &e = *impl_;
    check(!e.failed, "encoder session failed; create a new context",
          VA_STATUS_ERROR_ENCODING_ERROR);
    validate(s, p, e.width, e.height);
    check(deblocking(p, s.hevc) == e.filter, "changing deblocking requires a new context",
          VA_STATUS_ERROR_UNIMPLEMENTED);
    check((s.hevc
               ? s.hevc_sequence.general_level_idc == e.settings.hevc_sequence.general_level_idc &&
                     s.hevc_sequence.general_tier_flag == e.settings.hevc_sequence.general_tier_flag
               : s.sequence.level_idc == e.settings.sequence.level_idc &&
                     p.params.pic_fields.bits.entropy_coding_mode_flag == e.entropy) &&
              s.fps_num == e.settings.fps_num && s.fps_den == e.settings.fps_den &&
              s.bitrate == e.settings.bitrate && s.peak_bitrate == e.settings.peak_bitrate &&
              s.min_qp == e.settings.min_qp && s.max_qp == e.settings.max_qp &&
              s.quality == e.settings.quality && s.icq_quality == e.settings.icq_quality,
          "changing encoder settings requires a new context", VA_STATUS_ERROR_UNIMPLEMENTED);
    const bool idr =
        s.hevc ? p.hevc_params.pic_fields.bits.idr_pic_flag : p.params.pic_fields.bits.idr_pic_flag;
    const bool intra =
        s.hevc ? p.hevc_slices.front().slice_type == 2 : p.slices.front().slice_type % 5 == 2;
    check(e.token || idr, "encoder must start with IDR", VA_STATUS_ERROR_INVALID_PARAMETER);
    // Clients with multiple GOPs per IDR (including Sunshine) submit non-IDR
    // I pictures / HEVC CRA. Iris only exposes FORCE_KEY_FRAME, which emits IDR.
    // Promote these requests to IDR and drain before changing the control below.
    // This is safe with our I/P-only, immediately preceding reference contract:
    // no later picture can refer across this reset. Firmware owns headers/POC;
    // never relabel an inter-coded picture or rewrite the firmware's NAL header.
    if (!intra) {
        VASurfaceID reference;
        bool valid;
        if (s.hevc) {
            const auto &ref = p.hevc_slices.front().ref_pic_list0[0];
            reference = ref.picture_id;
            valid = !(ref.flags & (VA_PICTURE_HEVC_INVALID | VA_PICTURE_HEVC_LONG_TERM_REFERENCE));
        } else {
            const auto &ref = p.slices.front().RefPicList0[0];
            reference = ref.picture_id;
            valid = !(ref.flags & (VA_PICTURE_H264_INVALID | VA_PICTURE_H264_LONG_TERM_REFERENCE));
        }
        check(valid && reference == e.last_reference && e.last_reference != VA_INVALID_SURFACE,
              "encoder only supports the immediately preceding reference",
              VA_STATUS_ERROR_UNIMPLEMENTED);
    }
    check(s.hevc ? p.hevc_params.pic_fields.bits.reference_pic_flag
                 : p.params.pic_fields.bits.reference_pic_flag,
          "non-reference encoding unsupported", VA_STATUS_ERROR_UNIMPLEMENTED);
    auto memory = input->memory;
    check(memory && memory->fourcc == VA_FOURCC_NV12 && memory->width >= e.width &&
              memory->height >= e.height,
          "invalid encoder input surface", VA_STATUS_ERROR_INVALID_SURFACE);
    check(!output.coded_mapped, "coded buffer is mapped", VA_STATUS_ERROR_SURFACE_BUSY);
    // ICQ clients leave picture QP at the FFmpeg dummy default; the quality
    // factor from the rate-control buffer names the intended target instead.
    int qp = s.rate_control == VA_RC_ICQ
                 ? int(s.icq_quality)
                 : s.hevc ? int(p.hevc_params.pic_init_qp) + p.hevc_slices.front().slice_qp_delta
                          : int(p.params.pic_init_qp) + p.slices.front().slice_qp_delta;
    check(qp >= 1 && qp <= 51, "invalid encode picture QP", VA_STATUS_ERROR_INVALID_PARAMETER);
    e.refresh();
    check(!output.encode_task || !output.encode_task->pending, "coded buffer is pending",
          VA_STATUS_ERROR_SURFACE_BUSY);
    try {
        // Scalar V4L2 controls have no per-request association. Drain before a
        // QP change or forced IDR so they cannot affect an earlier queued frame.
        if ((qp_mode(s.rate_control) && qp != e.current_qp) || (intra && e.token))
            e.finish();
        if (e.jobs.size() == e.input_count + e.capture.size()) {
            auto task = e.jobs.begin()->second->task;
            check(e.wait(task, VA_TIMEOUT_INFINITE), "encoder timed out",
                  VA_STATUS_ERROR_ENCODING_ERROR);
        }
        unsigned index;
        const auto slot_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        for (;;) {
            for (index = 0; index < e.slots.size() && e.slots[index]; ++index) {
            }
            if (index < e.slots.size())
                break;
            check(std::chrono::steady_clock::now() < slot_deadline, "encoder input queue timed out",
                  VA_STATUS_ERROR_ENCODING_ERROR);
            pollfd ready{e.fd, short(POLLIN | POLLOUT), 0};
            int ret = poll(&ready, 1, 100);
            check((ret >= 0 || errno == EINTR) && !(ready.revents & (POLLERR | POLLHUP | POLLNVAL)),
                  "encoder input queue poll failed", VA_STATUS_ERROR_ENCODING_ERROR);
            e.refresh();
        }
        const bool direct =
            e.import && memory->fd >= 0 &&
            (memory->origin == MemoryOrigin::MsmCoherent || memory->origin == MemoryOrigin::Iris ||
             memory->origin == MemoryOrigin::DmaHeap) &&
            memory->stride == e.stride && memory->storage_height == e.storage_height &&
            !memory->data_offset && memory->size >= e.input_size;
        if (!e.import) {
            auto raw = e.raw[index];
            copy_surface(*raw, *memory, e.width, e.height);
            memory = raw;
        } else if (!direct) {
            // Never send write-combined GEM directly to a coherent VPU, or
            // GPU-write an Iris MMAP allocation with cached CPU aliases.
            if (!e.staging[index]) {
                auto m = std::make_shared<Memory>();
                m->width = e.width;
                m->height = e.height;
                m->stride = e.stride;
                m->storage_height = e.storage_height;
                m->size = e.input_size;
                int coherent = 0;
                m->fd = irisva_msm_allocate(e.render_fd, m->size, &coherent);
                check(m->fd >= 0 && coherent, "allocate coherent encoder staging",
                      VA_STATUS_ERROR_ALLOCATION_FAILED);
                m->origin = MemoryOrigin::MsmCoherent;
                e.staging[index] = m;
                if (!e.copier)
                    e.copier = std::make_unique<SurfaceCopier>(e.render_fd);
            }
            e.copier->copy(e.staging[index], memory, e.width, e.height);
            memory = e.staging[index];
        }
        // Retain the input through both DQBUFs, and STREAMOFF before releasing
        // it if a job fails. Cache coherency does not replace lifetime/fences.
        wait_surface_access(*memory, POLLIN);
        trace("encoder input path=%s origin=%u",
              direct     ? "direct"
              : e.import ? "staging"
                         : "cpu-mmap",
              unsigned(memory->origin));
        if (qp_mode(s.rate_control) && qp != e.current_qp) {
            e.control(s.hevc ? V4L2_CID_MPEG_VIDEO_HEVC_I_FRAME_QP
                             : V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP,
                      qp);
            e.control(s.hevc ? V4L2_CID_MPEG_VIDEO_HEVC_P_FRAME_QP
                             : V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP,
                      qp);
            e.current_qp = qp;
        }
        if (intra && e.token)
            e.control(V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME, 0);
        QueueBuffer q(OUTPUT, index, e.import);
        if (e.import)
            q.plane.m.fd = memory->fd;
        q.plane.length = memory->size;
        q.plane.bytesused = e.input_size;
        const uint64_t token = e.token + 1;
        // Firmware rate control derives frame intervals from buffer timestamps
        // (iris forwards them to the HFI buffer). Stamp the real frame period;
        // token-sized microseconds starve CBR/VBR and force max QP.
        const uint64_t ts_us =
            uint64_t(token) * s.fps_den * 1000000 / s.fps_num;
        q.buffer.timestamp.tv_sec = ts_us / 1000000;
        q.buffer.timestamp.tv_usec = ts_us % 1000000;
        auto job = std::make_shared<Impl::Job>();
        job->task = std::make_shared<EncodeTask>();
        job->output = coded;
        job->memory = memory;
        e.jobs.emplace(token, job);
        e.slots[index] = job;
        output.encode_task = job->task;
        output.coded_ready = false;
        output.coded = {};
        checked(e.fd, VIDIOC_QBUF, &q.buffer, "encoder QBUF OUTPUT");
        e.token = token;
        e.last_reference = p.reconstruction(s.hevc);
        trace("encoder queued token=%llu depth=%zu", (unsigned long long)token, e.jobs.size());
        return job->task;
    } catch (const Error &error) {
        e.stop(error.status);
        throw;
    } catch (...) {
        e.stop(VA_STATUS_ERROR_ENCODING_ERROR);
        throw;
    }
}
bool Encoder::sync(const std::shared_ptr<EncodeTask> &task, uint64_t timeout) {
    return impl_->wait(task, timeout);
}
void Encoder::refresh() {
    impl_->refresh();
}
void Encoder::finish() {
    impl_->finish();
}
} // namespace irisva
