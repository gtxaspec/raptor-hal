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
 * The frame-buffer entry points (MI_SYS_ChnOutputPortGetBuf / PutBuf,
 * MI_SYS_GetFd / CloseFd, MI_SYS_FlushInvCache, MI_SYS_Va2Pa) have no divinus
 * counterpart -- it reads encoded streams out of VENC and never touches a raw
 * frame -- so their types and signatures come from waybeam_venc plus
 * libmi_sys.so's disassembly. See the i6_sys_bufinfo comment below.
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

/*
 * Output-port frame buffers.
 *
 * Not from divinus -- it reads encoded streams out of VENC and never touches a
 * raw frame -- so the layout comes from waybeam_venc, which does read VIF/VPE
 * output on this exact silicon: StabSysBufInfo_t in src/star6e_framing_stab.c
 * (Infinity6E) and IyBufInfo_t in src/star6e_ipu_yolo.c (Infinity6E), plus
 * StabBufInfo_t in src/maruko_framing_stab.c (i6c). Three transcriptions that
 * agree, two of them for this SoC, all three exercised on hardware.
 *
 * libmi_sys.so's own MI_SYS_ChnOutputPortGetBuf corroborates it. The wrapper
 * memcpy's the 16-byte port descriptor into its ioctl payload, and after the
 * call copies 272 bytes back out to argument 2 and the buffer handle from
 * payload+288 to argument 3. So the kernel's view of this struct is 272 bytes,
 * meaning the vendor union is 232 -- not the 512 waybeam reserves, and not the
 * 104 that the frame-data member alone needs.
 *
 * Reserving 512 anyway is deliberate. It is what the proven code does, it
 * cannot under-allocate for a 272-byte copy-out even if another firmware
 * revision grows the union, and every field we read sits in the first 136
 * bytes regardless. The static assertions below pin the copy size and the
 * offsets the kernel writes through, so a later edit cannot quietly shrink
 * this below what MI_SYS memcpy's into it.
 *
 * The 272 figure also settles the bool width: MI_BOOL is a C99 bool (1 byte,
 * per waybeam include/star6e.h), which puts the union at offset 32 and yields
 * align8(32 + 232 + 1) == 272. Four-byte bools would give 280 and every field
 * from bEndOfStream on would be misplaced.
 */
typedef enum {
    I6_SYS_BUFDATA_RAW = 0,
    I6_SYS_BUFDATA_FRAME = 1,
    I6_SYS_BUFDATA_META = 2,
    I6_SYS_BUFDATA_END
} i6_sys_bufdata;

typedef struct {
    int tileMode;
    i6_common_pixfmt pixFmt;
    i6_common_compr compress;
    int scanMode;
    int fieldType;
    int layoutType;
    unsigned short width;
    unsigned short height;
    void *virAddr[3];
    unsigned long long phyAddr[3];
    unsigned int stride[3];
    unsigned int bufSize;
    unsigned short ringStartLine;
    unsigned short ringTotalHeight;
    struct {
        int type;
        union {
            unsigned int globalGradient;
        } attr;
    } ispInfo;
    i6_common_rect crop;
} i6_sys_frame;

typedef struct {
    void *virAddr;
    unsigned long long phyAddr;
    unsigned int bufSize;
    unsigned int contentSize;
    bool endOfFrame;
    unsigned long long seqNum;
} i6_sys_raw;

typedef struct {
    void *virAddr;
    unsigned long long phyAddr;
    unsigned int size;
    unsigned int extraData;
    unsigned int fromModule;
} i6_sys_meta;

typedef struct {
    unsigned long long pts;
    unsigned long long sidebandMsg;
    i6_sys_bufdata bufType;
    bool endOfStream;
    bool usrBuf;
    unsigned int seqNum;
    bool drop;
    union {
        i6_sys_frame frame;
        i6_sys_raw raw;
        i6_sys_meta meta;
        unsigned char reserved[512];
    };
    unsigned char cusFlag;
} i6_sys_bufinfo;

/* The port descriptor MI memcpy's out of argument 1 -- 16 bytes, and the probe
 * confirmed sizeof(i6_sys_bind) == 16 on hardware. */
_Static_assert(sizeof(i6_sys_bind) == 16, "i6_sys_bind must match the 16 bytes GetBuf copies");
_Static_assert(sizeof(i6_sys_bufinfo) >= 272,
    "i6_sys_bufinfo must not be smaller than the 272 bytes MI_SYS copies into it");
_Static_assert(offsetof(i6_sys_bufinfo, seqNum) == 24, "i6_sys_bufinfo header layout drifted");
_Static_assert(offsetof(i6_sys_bufinfo, frame) == 32, "i6_sys_bufinfo union must start at 32");
_Static_assert(offsetof(i6_sys_frame, virAddr) == 28, "i6_sys_frame plane pointers drifted");
_Static_assert(offsetof(i6_sys_frame, phyAddr) == 40, "i6_sys_frame phyAddr must be 8-aligned");
_Static_assert(offsetof(i6_sys_frame, stride) == 64, "i6_sys_frame stride drifted");

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

    /*
     * Frame access on a channel's output port. Every arity here was read off
     * libmi_sys.so and independently matches waybeam's Infinity6E typedefs
     * (star6e_framing_stab.c) -- which matters because the same calls take a
     * leading chip id on i6c/m6 and do not on i6/i3. Both references and the
     * disassembly agree on that split, so these take no chip id:
     *
     *   MI_SYS_GetFd                 2 args, payload 20 = port(16) + fd(4)
     *   MI_SYS_CloseFd               1 arg,  payload 4
     *   MI_SYS_ChnOutputPortGetBuf   3 args, payload 304, copies 272 out
     *   MI_SYS_ChnOutputPortPutBuf   1 arg,  payload 4
     *   MI_SYS_FlushInvCache         2 args, payload 8  = va(4) + size(4)
     *   MI_SYS_Va2Pa                 2 args, payload 16 = va(4) + pad + pa(8)
     *
     * fnGetFd returns a descriptor that select()/poll() marks readable when a
     * frame is queued, which is how both references avoid polling GetBuf.
     */
    int (*fnGetFd)(i6_sys_bind *port, int *fd);
    int (*fnCloseFd)(int fd);
    int (*fnGetOutputBuf)(i6_sys_bind *port, i6_sys_bufinfo *buf, int *handle);
    int (*fnPutOutputBuf)(int handle);
    int (*fnFlushInvCache)(void *virAddr, unsigned int size);
    int (*fnVa2Pa)(void *virAddr, unsigned long long *phyAddr);
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

    /*
     * Also optional, for the same reason. raptor's video path binds
     * VIF -> VPE -> VENC in hardware and reads encoded streams from VENC, so
     * nothing on the streaming path calls these -- they serve frame-level
     * consumers (the 2b VIF bring-up check, and later ISP/IPU work). Making
     * them fatal would let a firmware missing one symbol take down streaming
     * that never needed it. Callers null-check.
     */
    sys_lib->fnGetFd = (int(*)(i6_sys_bind *port, int *fd))
        dlsym(sys_lib->handle, "MI_SYS_GetFd");
    sys_lib->fnCloseFd = (int(*)(int fd))
        dlsym(sys_lib->handle, "MI_SYS_CloseFd");
    sys_lib->fnGetOutputBuf = (int(*)(i6_sys_bind *port, i6_sys_bufinfo *buf, int *handle))
        dlsym(sys_lib->handle, "MI_SYS_ChnOutputPortGetBuf");
    sys_lib->fnPutOutputBuf = (int(*)(int handle))
        dlsym(sys_lib->handle, "MI_SYS_ChnOutputPortPutBuf");
    sys_lib->fnFlushInvCache = (int(*)(void *virAddr, unsigned int size))
        dlsym(sys_lib->handle, "MI_SYS_FlushInvCache");
    sys_lib->fnVa2Pa = (int(*)(void *virAddr, unsigned long long *phyAddr))
        dlsym(sys_lib->handle, "MI_SYS_Va2Pa");

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
