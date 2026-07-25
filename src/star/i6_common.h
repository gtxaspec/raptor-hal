/*
 * star/i6_common.h -- SigmaStar MI common types, Infinity6E
 *
 * Vendored from OpenIPC divinus, src/hal/star/i6_common.h (MIT), which is
 * why this file carries divinus's licence rather than raptor's. The type
 * layouts here describe the ABI of prebuilt vendor .so files: SigmaStar
 * publishes no redistributable headers, and a wrong layout corrupts memory
 * rather than failing to compile, so validated declarations are worth far
 * more than freshly derived ones. divinus's are exercised on this exact
 * silicon, and waybeam_venc's independently maintained sigmastar_types.h
 * agrees with them field for field.
 *
 * Keep the i6_* names and field order as upstream so fixes stay diffable
 * against divinus. Adaptations applied to every vendored i6_*.h in this
 * directory, and nothing else:
 *
 *   1. divinus's "../symbols.h" and "../types.h" includes are dropped.
 *      hal_symbol_load() moves here (it is the only thing the i6_* headers
 *      took from symbols.h); nothing in types.h was ever referenced by
 *      them.
 *   2. HAL_ERROR(mod, ...) becomes HAL_LOG_ERR(...) plus an explicit
 *      return, since HAL_ERROR's hidden `return EXIT_FAILURE` is not a
 *      convention raptor uses.
 *   3. EXIT_SUCCESS/EXIT_FAILURE become RSS_OK/RSS_ERR_*: RSS_ERR_NOENT
 *      when a library is absent, RSS_ERR_NOTSUP when a library is present
 *      but lacks a symbol. Callers can then tell "SDK not installed" from
 *      "SDK too old" without parsing logs.
 *   4. The loaders are `static inline`, not `static`. raptor-hal builds
 *      with -Werror, and an unused `static` function in a header is a
 *      -Wunused-function error; `static inline` is exempt.
 *
 * MI is reached through dlopen rather than by linking -lmi_*, so no MI
 * library is needed at build time and the binary binds to whatever the
 * device itself carries -- which matters because the MI stack is coupled to
 * the running 4.9.84 kernel.
 *
 * Copyright (c) 2024 OpenIPC
 * SPDX-License-Identifier: MIT
 */

#ifndef STAR_I6_COMMON_H
#define STAR_I6_COMMON_H

#include "hal_internal.h"

#include <dlfcn.h>

/*
 * hal_symbol_load -- dlsym with a diagnostic, divinus's symbols.h helper.
 *
 * Returns NULL on failure, so call sites read as
 *   if (!(lib->fnFoo = (cast)hal_symbol_load(mod, lib->handle, "MI_Foo")))
 *       return RSS_ERR_NOTSUP;
 * Naming the module in the message matters: with nine libraries loaded, the
 * symbol alone does not say which one came up short.
 */
static inline void *hal_symbol_load(const char *module, void *handle, const char *symbol)
{
    void *function = dlsym(handle, symbol);

    if (!function) {
        HAL_LOG_ERR("%s: failed to acquire symbol %s", module, symbol);
        return NULL;
    }

    return function;
}

typedef enum {
    I6_BAYER_RG,
    I6_BAYER_GR,
    I6_BAYER_BG,
    I6_BAYER_GB,
    I6_BAYER_R0,
    I6_BAYER_G0,
    I6_BAYER_B0,
    I6_BAYER_G1,
    I6_BAYER_G2,
    I6_BAYER_I0,
    I6_BAYER_G3,
    I6_BAYER_I1,
    I6_BAYER_END
} i6_common_bayer;

typedef enum {
    I6_COMPR_NONE,
    I6_COMPR_SEG,
    I6_COMPR_LINE,
    I6_COMPR_FRAME,
    // Valid on infinity6e only
    I6_COMPR_8BIT,
    I6_COMPR_END
} i6_common_compr;

typedef enum {
    I6_EDGE_SINGLE_UP,
    I6_EDGE_SINGLE_DOWN,
    I6_EDGE_DOUBLE,
    I6_EDGE_END
} i6_common_edge;

typedef enum {
    I6_HDR_OFF,
    I6_HDR_VC,
    I6_HDR_DOL,
    I6_HDR_EMBED,
    I6_HDR_LI,
    I6_HDR_END
} i6_common_hdr;

typedef enum {
    I6_INPUT_VUVU = 0,
    I6_INPUT_UVUV,
    I6_INPUT_UYVY = 0,
    I6_INPUT_VYUY,
    I6_INPUT_YUYV,
    I6_INPUT_YVYU,
    I6_INPUT_END
} i6_common_input;

typedef enum {
    I6_INTF_BT656,
    I6_INTF_DIGITAL_CAMERA,
    I6_INTF_BT1120_STANDARD,
    I6_INTF_BT1120_INTERLEAVED,
    I6_INTF_MIPI,
    I6_INTF_END
} i6_common_intf;

typedef enum {
    I6_PREC_8BPP,
    I6_PREC_10BPP,
    I6_PREC_12BPP,
    I6_PREC_14BPP,
    I6_PREC_16BPP,
    I6_PREC_END
} i6_common_prec;

typedef enum {
    I6_PIXFMT_YUV422_YUYV,
    I6_PIXFMT_ARGB8888,
    I6_PIXFMT_ABGR8888,
    I6_PIXFMT_BGRA8888,
    I6_PIXFMT_RGB565,
    I6_PIXFMT_ARGB1555,
    I6_PIXFMT_ARGB4444,
    I6_PIXFMT_I2,
    I6_PIXFMT_I4,
    I6_PIXFMT_I8,
    I6_PIXFMT_YUV422SP,
    I6_PIXFMT_YUV420SP,
    I6_PIXFMT_YUV420SP_NV21,
    I6_PIXFMT_YUV420_MST,
    I6_PIXFMT_YUV422_UYVY,
    I6_PIXFMT_YUV422_YVYU,
    I6_PIXFMT_YUV422_VYUY,
    I6_PIXFMT_YC420_MSTITLE1_H264,
    I6_PIXFMT_YC420_MSTITLE2_H265,
    I6_PIXFMT_YC420_MSTITLE3_H265,
    I6_PIXFMT_RGB_BAYER,
    I6_PIXFMT_RGB_BAYER_END =
        I6_PIXFMT_RGB_BAYER + I6_PREC_END * I6_BAYER_END - 1,
    I6_PIXFMT_RGB888,
    I6_PIXFMT_BGR888,
    I6_PIXFMT_GRAY8,
    I6_PIXFMT_RGB101010,
    I6_PIXFMT_RGB888P,
    I6_PIXFMT_END
} i6_common_pixfmt;

typedef struct {
    unsigned short width;
    unsigned short height;
} i6_common_dim;

typedef struct {
    unsigned short x;
    unsigned short y;
    unsigned short width;
    unsigned short height;
} i6_common_rect;

typedef struct {
    int vsyncInv;
    int hsyncInv;
    int pixclkInv;
    unsigned int vsyncDelay;
    unsigned int hsyncDelay;
    unsigned int pixclkDelay;
} i6_common_sync;

#endif /* STAR_I6_COMMON_H */
