# libva-v4l2

A VA-API backend for Qualcomm Iris and Venus, using the Linux V4L2 stateful M2M
interface.
It provides hardware video decoding and H.264/HEVC encoding on **SC8280XP**, tested on
**Radxa Dragon Q8B**. The project is experimental and targets this platform;
it is not a generic backend for all V4L2 devices.

The device discovery accepts both the original Iris kernel driver name
(`iris_driver`) and the upstream Qualcomm Venus driver name (`qcom-venus`).
Venus support is intended for kernels exposing the same stateful controls and
buffer semantics; codec support still has to be validated on each firmware
revision.
On Venus kernels that omit the optional Iris `PREPEND_SPSPPS_TO_IDR` control,
the backend relies on the required joined sequence-header mode.

## Support

| Codec | Decoding | Encoding |
| --- | --- | --- |
| H.264 / AVC | Constrained Baseline, Main, High (8-bit) | Same profiles (8-bit) |
| H.265 / HEVC | Main, Main10 (8/10-bit) | Main (8-bit) |
| VP9 | Profiles 0 and 2 (8/10-bit) | — |

Supported video is progressive 4:2:0. Decoding has been tested with FFmpeg,
GStreamer, mpv, VLC, Chromium, and Kodi; H.264 encoding with FFmpeg, GStreamer,
and Sunshine. HEVC encoding is tested with FFmpeg and GStreamer.

- H.264 and HEVC encoding support CQP, CBR, and VBR, even dimensions from 128×128 to
  3840×2160, and I/P frames with one reference. I-frame requests produce IDR frames; B frames
  are unsupported.
  HEVC encoding currently uses Main tier; Main10 encoding is not implemented.
- Optional FastCV VPP provides CDSP scaling for linear NV12 with DMA-BUF support.
  Widths must be multiples of 16 and heights even. Color/bit-depth conversion
  and other VPP filters are unsupported.
- Compatible DMA-BUF paths avoid raw-frame copies. Applications that cache
  surfaces before decoding use a GPU copy by default.

## Performance

HEVC Main (8-bit) throughput on **Radxa Dragon Q8B — SC8280XP, Iris HFI Gen2**,
using FFmpeg 9.0.1 and GStreamer 1.28.6. Values are average frames per second.

| Pipeline | Operation | 720p (1280×720) | 1080p (1920×1080) | 4K (3840×2160) |
| --- | --- | ---: | ---: | ---: |
| FFmpeg VA-API | Decode | 1111 | 806 | 297 |
| FFmpeg VA-API | Encode | 1289 | 714 | 206 |
| GStreamer VA | Decode | 1088 | 823 | 278 |
| GStreamer VA | Encode | 1285 | 709 | 206 |
| GStreamer direct V4L2 | Decode | 1199 | 882 | 330 |
| GStreamer direct V4L2 | Encode | 1287 | 724 | 210 |

Synthetic I/P streams, GOP 60, VBR at 5 / 10 / 20 Mbps respectively. Encoding
includes CPU NV12 upload; FFmpeg encoding uses **`async_depth=4`**. Decoding waits
for completed frames without CPU download or rendering. Results are three-run
means, except 4K encoding, which uses one completed run; each run processes
9,000 / 6,000 / 2,400 frames respectively. Actual performance depends on content
and pipeline configuration.

## Requirements

- Linux with the Qualcomm Iris or Venus stateful driver and MSM DRM support.
- The changes supplied in [patches/](patches/), unless already present in your
  kernel: decode-order output, larger CAPTURE pools, and encoder deblocking
  controls. The patches are references to integrate into your kernel.
- Access to the Iris video devices and DRM render node.
- A C++17 compiler, Meson ≥ 0.61, Ninja, pkg-config, and development libraries
  for libva, libdrm, EGL, GLES 3.2, and GBM.
- For VPP: system FastCV and FastRPC libraries with a working CDSP runtime.

## Build and install

The Debian package CI builds one ARM64 package on Ubuntu Noble, then checks
installation and driver loading on Debian Trixie, Ubuntu Noble, and Ubuntu
Resolute. Tag builds publish the same package and debug symbols to a draft
GitHub Release after all checks pass. Hardware codec testing is separate.

On Arch Linux:

```sh
sudo pacman -S --needed base-devel meson ninja pkgconf libva libdrm mesa libglvnd
python3 packaging/make-dist.py
cd packaging/arch
makepkg -si
```

The [PKGBUILD](packaging/arch/PKGBUILD) packages the userspace driver. After
editing packaged sources, regenerate the archive, update `sha256sums` using
`makepkg -g`, and refresh `.SRCINFO` with `makepkg --printsrcinfo > .SRCINFO`.

Alternatively, build and install with Meson:

```sh
meson setup build --buildtype=release --prefix=/usr -Dlibdir=lib
meson compile -C build
sudo meson install -C build
```

FastCV VPP defaults to `-Dfastcv=disabled`. Use `-Dfastcv=auto` to enable it
when dependencies are available or `-Dfastcv=enabled` to require build dependencies.
Debian / Ubuntu packages explicitly disable FastCV for consistent distribution builds.
FastCV is loaded dynamically; missing runtime libraries disable only VPP.

Adjust `libdir` if your system uses a driver directory other than `/usr/lib/dri`.
Installation adds the MSM driver alias, so **`LIBVA_DRIVER_NAME` is not required**
on the target platform.

To check the installation (`vainfo` comes from `libva-utils` on Arch):

```sh
vainfo --display drm --device /dev/dri/renderD128
```

## Usage

Play a video with mpv:

```sh
mpv --hwdec=vaapi input.mp4
```

Encode to H.264 with FFmpeg:

```sh
ffmpeg -vaapi_device /dev/dri/renderD128 -i input.mp4 -an \
  -vf format=nv12,hwupload -c:v h264_vaapi -profile:v high \
  -bf 0 -g 60 -rc_mode CBR -b:v 6M -async_depth 4 output.mp4
```

For HEVC, use `-c:v hevc_vaapi -profile:v main` in the command above.

For hardware decoding and encoding of a supported 8-bit input:

```sh
ffmpeg -hwaccel vaapi -hwaccel_device /dev/dri/renderD128 \
  -hwaccel_output_format vaapi -i input.mp4 -an \
  -c:v h264_vaapi -bf 0 -qp 24 -async_depth 4 output.mp4
```

MP4 and raw H.264 output are supported; for Matroska, encode to raw H.264 first
and remux. FFmpeg may warn about unsupported packed headers because the firmware
produces the headers. Custom VUI/SEI metadata is not forwarded.

Scale a decoded NV12 video on CDSP:

```sh
ffmpeg -hwaccel vaapi -hwaccel_device /dev/dri/renderD128 \
  -hwaccel_output_format vaapi -i input.mp4 -an \
  -vf scale_vaapi=w=1920:h=1080 -c:v h264_vaapi -bf 0 -qp 24 \
  -async_depth 4 output.mp4
```

For GStreamer, set `GST_VA_ALL_DRIVERS=1` to enable this driver's `va` elements:

```sh
GST_VA_ALL_DRIVERS=1 gst-launch-1.0 -e \
  videotestsrc num-buffers=120 ! \
  video/x-raw,format=NV12,width=1920,height=1080,framerate=30/1 ! \
  vah264enc rate-control=vbr bitrate=6000 target-percentage=100 \
    b-frames=0 ref-frames=1 target-usage=1 key-int-max=60 ! \
  'video/x-h264,profile=high' ! h264parse ! mp4mux ! filesink location=output.mp4
```

For Sunshine/Moonlight, use **H.264 / SDR**. HEVC and AV1 encoder probes may fail
while H.264 remains available.

### Qualcomm Venus

On kernels exposing the stateful Qualcomm Venus driver as `qcom-venus`, the
backend discovers the decoder and encoder automatically. Verify the DRM path
with:

```sh
vainfo --display drm --device /dev/dri/renderD128
```

Set `LIBVA_DRIVERS_PATH` when the driver is installed outside libva's default
search path (for example `/usr/lib64/dri` on distributions with a lib64
layout).

For mpv, direct VAAPI surfaces work with a GPU video output in a graphical
Wayland session:

```sh
mpv --vo=gpu-next --gpu-api=vulkan \
  --hwdec=vaapi --vaapi-device=/dev/dri/renderD128 input.mp4
```

Use `--hwdec=vaapi-copy` when the video output cannot import VAAPI surfaces;
decoding remains hardware accelerated, while frames are copied back to system
memory before rendering. On the TCL B220G, H.264, HEVC, and VP9 decode and
H.264/HEVC encode were validated with the Venus firmware. Container muxing of
newly encoded HEVC still needs explicit codec-extradata handling; raw HEVC and
decode round-trips are valid.

Until HEVC packed-header support is added, remux a raw HEVC stream with
`extract_extradata`:

```sh
ffmpeg -f hevc -i encoded.hevc -c copy -bsf:v extract_extradata \
  -f matroska encoded.mkv
```

## Configuration and development

| Variable | Purpose |
| --- | --- |
| `LIBVA_DRIVER_NAME=v4l2` | Explicitly select this backend. |
| `LIBVA_DRIVERS_PATH=/path/to/build` | Load an uninstalled build. |
| `IRIS_VAAPI_DEVICE` / `IRIS_VAAPI_ENCODER_DEVICE` | Override automatic video-device discovery. |
| `IRIS_VAAPI_COPY=gpu` or `cpu` | Select the copy path; default is `gpu`. |
| `IRIS_VAAPI_DEBUG=1` | Enable diagnostics. |

Use clang-format 22 and the checked-in `.clang-format`. Meson provides `format`
and `format-check` targets when clang-format is available; `-Dwerror=true`
enables compiler warnings as errors. Hardware tests and their run instructions
are in [tests/](tests/). Local research and historical results live in ignored
`tmp/` and are not required to build the driver.

## License

The backend is [MIT licensed](LICENSE). Bundled Linux kernel patches are
GPL-2.0-only; external test frameworks and media retain their own licenses.
