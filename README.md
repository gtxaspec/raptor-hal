# raptor-hal

Hardware Abstraction Layer for the Raptor Streaming System (RSS). Provides a
unified C API over the Ingenic IMP SDK, abstracting away the differences between
three SDK generations so that RSS daemons (RVD, RAD, RIC) can target all 8
Ingenic T-series SoCs from a single codebase.

## Supported Platforms

| SoC | SDK Generation | Notes |
|-----|---------------|-------|
| T20 | Old | XBurst1, H.264 only |
| T21 | Old | XBurst1 |
| T23 | Old | XBurst1, extended OSD |
| T30 | Old | XBurst1 |
| T31 | New | XBurst1, primary dev target |
| T32 | New (hybrid) | XBurst2, multi-sensor, old-style type names |
| T40 | IMPVI | XBurst2, multi-sensor |
| T41 | IMPVI | XBurst2, multi-sensor |

SDK generation differences (encoder struct layout, RC mode enums, ISP function
signatures, ring buffer semantics) are handled internally via compile-time
`HAL_OLD_SDK` / `HAL_NEW_SDK` / `HAL_IMPVI_SDK` macros.

## Build

Produces a static library `libraptor_hal.a`. Requires the
[ingenic-headers](https://github.com/thingino/ingenic-headers) repo.

```
make PLATFORM=T31 CROSS_COMPILE=mipsel-linux-
```

### Variables

| Variable | Required | Default | Description |
|----------|----------|---------|-------------|
| `PLATFORM` | yes | -- | Target SoC: T20, T21, T23, T30, T31, T32, T40, T41 |
| `CROSS_COMPILE` | yes | -- | Toolchain prefix (e.g. `mipsel-linux-`) |
| `INGENIC_HEADERS` | no | `../ingenic-headers` | Path to SDK header repo |
| `INGENIC_LIB` | no | `../ingenic-lib` | Path to SDK library repo |
| `DEBUG` | no | 0 | Set to 1 for `-O0 -g` and `HAL_DEBUG` logging |
| `V` | no | 0 | Set to 1 for verbose build output |

```
make PLATFORM=T31 clean    # clean build artifacts
make PLATFORM=T31 info     # print resolved build config
```

## API Overview

The public API is a single header: `include/raptor_hal.h`. All Ingenic SDK types
are abstracted behind `rss_*` types. The HAL exposes an operations vtable
(`rss_hal_ops_t`) through an opaque context.

### Factory

```c
rss_hal_ctx_t  *rss_hal_create(void);
void            rss_hal_destroy(rss_hal_ctx_t *ctx);
const rss_hal_ops_t *rss_hal_get_ops(rss_hal_ctx_t *ctx);
```

### Calling Convention

All vtable functions are invoked through `RSS_HAL_CALL`, which returns
`RSS_ERR_NOTSUP` if the function pointer is NULL (capability not available on
the target SoC).

```c
RSS_HAL_CALL(ops, isp_set_brightness, ctx, 128);
```

### Subsystems

| Subsystem | Prefix | Description |
|-----------|--------|-------------|
| System | `init`, `deinit`, `bind`, `unbind`, `sys_*` | Lifecycle, pipeline binding, register access |
| Framesource | `fs_*` | Channel create/destroy/enable, raw frame capture, crop, scaler |
| Encoder | `enc_*` | H.264/H.265/JPEG encode, rate control, GOP, IDR, NAL framing |
| ISP | `isp_*` | Image tuning (BCSH, flip, WB, AE/AWB/AF, gamma, noise reduction, WDR) |
| OSD | `osd_*` | Overlay regions (picture, cover), alpha blending, per-group control |
| ISP OSD | `isp_osd_*` | Hardware overlay in ISP pipeline (T23/T32/T40/T41) |
| Audio In | `audio_*` | AI device, PCM capture, NS/HPF/AGC/AEC, custom encoder registration |
| Audio Enc | `aenc_*` | Audio encoding pipeline |
| Audio Dec | `adec_*` | Audio decoding pipeline (backchannel) |
| Audio Out | `ao_*` | Speaker playback |
| DMIC | `dmic_*` | Digital microphone (T30/T31/T32/T40/T41) |
| GPIO | `gpio_*`, `ircut_*` | Pin control, IR-cut filter |
| IVS | `ivs_*` | Motion detection / intelligent video |
| Memory | `mem_*` | DMA alloc/free, cache flush, phys/virt translation, pool management |
| Capabilities | `get_caps` | Per-SoC feature/limit query (`rss_hal_caps_t`) |

### Key Types

- `rss_frame_t` -- encoded video frame with parsed NAL units
- `rss_audio_frame_t` -- PCM audio frame
- `rss_video_config_t` -- encoder channel configuration (codec, resolution, RC, GOP)
- `rss_fs_config_t` -- framesource channel configuration (resolution, crop, scaler)
- `rss_hal_caps_t` -- per-SoC capability flags and limits
- `rss_sensor_config_t` -- sensor driver configuration (I2C, GPIO, VIN type)

### Source Layout

```
include/raptor_hal.h     Public API (only header consumers include)
src/hal_internal.h       SDK generation macros, type shims, logging, context struct
src/hal_common.c         Vtable, factory, system lifecycle, bind/unbind
src/hal_caps.c           Per-SoC capability structs
src/hal_encoder.c        Video encoder (H.264/H.265/JPEG, NAL parsing)
src/hal_framesource.c    Framesource channels
src/hal_isp.c            ISP tuning
src/hal_audio.c          Audio input/output/encoding/decoding
src/hal_osd.c            OSD regions
src/hal_gpio.c           GPIO and IR-cut
src/hal_ivs.c            Intelligent video / motion detection
src/hal_dmic.c           Digital microphone
src/hal_memory.c         DMA memory management
```

## License

Licensed under the GNU General Public License v3.0.
