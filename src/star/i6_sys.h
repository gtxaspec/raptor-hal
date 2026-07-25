/*
 * star/i6_sys.h -- MI_SYS bindings, Infinity6E
 *
 * Vendored from OpenIPC divinus, src/hal/star/i6_sys.h. See i6_common.h for
 * why these headers are vendored and for the four adaptations applied.
 *
 * libcam_os_wrapper is loaded first and RTLD_GLOBAL: every libmi_* depends
 * on it, and its absence is not fatal here on purpose -- some firmware
 * builds satisfy those symbols another way, and failing the whole load for a
 * library that may be unnecessary would be worse than letting the libmi_sys
 * dlopen report the real problem.
 *
 * NOT bound here, and deliberately so: MI_SYS_ChnOutputPortGetBuf /
 * PutBuf, MI_SYS_FlushInvCache, MI_SYS_Va2Pa and friends. divinus never
 * touches raw frames -- it reads encoded streams out of VENC -- so upstream
 * has no validated MI_SYS_BufInfo layout to vendor, and guessing one would
 * reintroduce exactly the risk this file exists to avoid. Phase 2b/2c needs
 * them for the VIF/VPE frame path; take the layout from waybeam_venc, which
 * does use them on this silicon (`StabBufInfo_t` in src/maruko_framing_stab.c
 * and `IyBufInfo_t` in src/star6e_ipu_yolo.c), rather than deriving it.
 *
 * Copyright (c) 2024 OpenIPC
 * SPDX-License-Identifier: MIT
 */

#ifndef STAR_I6_SYS_H
#define STAR_I6_SYS_H

#include "i6_common.h"

#define I6_SYS_API "1.0"

typedef enum {
    I6_SYS_LINK_FRAMEBASE = 0x1,
    I6_SYS_LINK_LOWLATENCY = 0x2,
    I6_SYS_LINK_REALTIME = 0x4,
    I6_SYS_LINK_AUTOSYNC = 0x8,
    I6_SYS_LINK_RING = 0x10
} i6_sys_link;

typedef enum {
    I6_SYS_MOD_IVE,
    I6_SYS_MOD_VDF,
    I6_SYS_MOD_VENC,
    I6_SYS_MOD_RGN,
    I6_SYS_MOD_AI,
    I6_SYS_MOD_AO,
    I6_SYS_MOD_VIF,
    I6_SYS_MOD_VPE,
    I6_SYS_MOD_VDEC,
    I6_SYS_MOD_SYS,
    I6_SYS_MOD_FB,
    I6_SYS_MOD_HDMI,
    I6_SYS_MOD_DIVP,
    I6_SYS_MOD_GFX,
    I6_SYS_MOD_VDISP,
    I6_SYS_MOD_DISP,
    I6_SYS_MOD_OS,
    I6_SYS_MOD_IAE,
    I6_SYS_MOD_MD,
    I6_SYS_MOD_OD,
    I6_SYS_MOD_SHADOW,
    I6_SYS_MOD_WARP,
    I6_SYS_MOD_UAC,
    I6_SYS_MOD_LDC,
    I6_SYS_MOD_SD,
    I6_SYS_MOD_PANEL,
    I6_SYS_MOD_CIPHER,
    I6_SYS_MOD_SNR,
    I6_SYS_MOD_WLAN,
    I6_SYS_MOD_IPU,
    I6_SYS_MOD_MIPITX,
    I6_SYS_MOD_GYRO,
    I6_SYS_MOD_JPD,
    I6_SYS_MOD_ISP,
    I6_SYS_MOD_SCL,
    I6_SYS_MOD_WBC,
    I6_SYS_MOD_DSP,
    I6_SYS_MOD_PCIE,
    I6_SYS_MOD_DUMMY,
    I6_SYS_MOD_NIR,
    I6_SYS_MOD_DPU,
    I6_SYS_MOD_END,
} i6_sys_mod;

typedef struct {
    i6_sys_mod module;
    unsigned int device;
    unsigned int channel;
    unsigned int port;
} i6_sys_bind;

typedef struct {
    unsigned char version[128];
} i6_sys_ver;

typedef struct {
    void *handle, *handleCamOsWrapper;

    int (*fnExit)(void);
    int (*fnGetVersion)(i6_sys_ver *version);
    int (*fnInit)(void);

    int (*fnBind)(i6_sys_bind *source, i6_sys_bind *dest,
        unsigned int srcFps, unsigned int dstFps);
    int (*fnBindExt)(i6_sys_bind *source, i6_sys_bind *dest, unsigned int srcFps,
        unsigned int dstFps, i6_sys_link link, unsigned int linkParam);
    int (*fnSetOutputDepth)(i6_sys_bind *bind, unsigned int usrDepth, unsigned int bufDepth);
    int (*fnUnbind)(i6_sys_bind *source, i6_sys_bind *dest);

    /*
     * Media-clock access. Not from divinus -- it binds neither -- but needed
     * by rss_hal_ops_t's sys_get_timestamp/sys_rebase_timestamp, which
     * rvd_frame_loop.c uses to publish the media-clock-to-UTC mapping for SEI
     * timecodes.
     *
     * The signatures are read off libmi_sys.so rather than guessed, because
     * getting the arity wrong here writes through a bogus pointer instead of
     * failing. Each of these userspace entry points is a thin ioctl wrapper
     * that spills its arguments and then stores {payload size, user address}
     * for the kernel, so the disassembly states both the argument count and
     * the size of the pointee:
     *
     *   MI_SYS_Init         no spills at all               -> 0 args, and
     *                       matches fnInit(void) above, which confirms the
     *                       method reads true
     *   MI_SYS_GetVersion   one spill, size field 128       -> 1 pointer to
     *                       128 bytes == sizeof(i6_sys_ver), and this call is
     *                       known-good on hardware
     *   MI_SYS_GetCurPts    one spill, size field 8         -> 1 pointer to
     *                       8 bytes, i.e. unsigned long long *
     *   MI_SYS_InitPtsBase  strd r0,r1 -> a register pair   -> one u64 passed
     *   MI_SYS_SyncPts      strd r0,r1 -> a register pair       by value
     *
     * Note this differs by SoC family: waybeam calls MI_SYS_GetCurPts with a
     * leading device argument (maruko_framing_stab.c:626), which is Mercury6.
     * On Infinity6E that form would pass 0 as the output pointer.
     */
    int (*fnGetCurrentPts)(unsigned long long *pts);
    int (*fnInitPtsBase)(unsigned long long ptsBase);
    int (*fnSyncPts)(unsigned long long pts);
} i6_sys_impl;

static inline int i6_sys_load(i6_sys_impl *sys_lib)
{
    sys_lib->handleCamOsWrapper = dlopen("libcam_os_wrapper.so", RTLD_LAZY | RTLD_GLOBAL);

    if (!(sys_lib->handle = dlopen("libmi_sys.so", RTLD_LAZY | RTLD_GLOBAL))) {
        HAL_LOG_ERR("i6_sys: failed to load library: %s", dlerror());
        return RSS_ERR_NOENT;
    }

    if (!(sys_lib->fnExit = (int(*)(void))
        hal_symbol_load("i6_sys", sys_lib->handle, "MI_SYS_Exit")))
        return RSS_ERR_NOTSUP;

    if (!(sys_lib->fnGetVersion = (int(*)(i6_sys_ver *version))
        hal_symbol_load("i6_sys", sys_lib->handle, "MI_SYS_GetVersion")))
        return RSS_ERR_NOTSUP;

    if (!(sys_lib->fnInit = (int(*)(void))
        hal_symbol_load("i6_sys", sys_lib->handle, "MI_SYS_Init")))
        return RSS_ERR_NOTSUP;

    if (!(sys_lib->fnBind = (int(*)(i6_sys_bind *source, i6_sys_bind *dest,
        unsigned int srcFps, unsigned int dstFps))
        hal_symbol_load("i6_sys", sys_lib->handle, "MI_SYS_BindChnPort")))
        return RSS_ERR_NOTSUP;

    if (!(sys_lib->fnBindExt = (int(*)(i6_sys_bind *source, i6_sys_bind *dest, unsigned int srcFps,
        unsigned int dstFps, i6_sys_link link, unsigned int linkParam))
        hal_symbol_load("i6_sys", sys_lib->handle, "MI_SYS_BindChnPort2")))
        return RSS_ERR_NOTSUP;

    if (!(sys_lib->fnSetOutputDepth = (int(*)(i6_sys_bind *bind, unsigned int usrDepth,
        unsigned int bufDepth))
        hal_symbol_load("i6_sys", sys_lib->handle, "MI_SYS_SetChnOutputPortDepth")))
        return RSS_ERR_NOTSUP;

    if (!(sys_lib->fnUnbind = (int(*)(i6_sys_bind *source, i6_sys_bind *dest))
        hal_symbol_load("i6_sys", sys_lib->handle, "MI_SYS_UnBindChnPort")))
        return RSS_ERR_NOTSUP;

    /*
     * Optional: missing media-clock entry points cost SEI timecodes, not
     * streaming, so resolve them with a bare dlsym and let the callers
     * null-check. Failing the whole load over them would trade a cosmetic
     * loss for a dead pipeline.
     */
    sys_lib->fnGetCurrentPts = (int(*)(unsigned long long *pts))
        dlsym(sys_lib->handle, "MI_SYS_GetCurPts");
    sys_lib->fnInitPtsBase = (int(*)(unsigned long long ptsBase))
        dlsym(sys_lib->handle, "MI_SYS_InitPtsBase");
    sys_lib->fnSyncPts = (int(*)(unsigned long long pts))
        dlsym(sys_lib->handle, "MI_SYS_SyncPts");

    return RSS_OK;
}

static inline void i6_sys_unload(i6_sys_impl *sys_lib)
{
    if (sys_lib->handle)
        dlclose(sys_lib->handle);
    sys_lib->handle = NULL;
    if (sys_lib->handleCamOsWrapper)
        dlclose(sys_lib->handleCamOsWrapper);
    sys_lib->handleCamOsWrapper = NULL;
    memset(sys_lib, 0, sizeof(*sys_lib));
}

#endif /* STAR_I6_SYS_H */
