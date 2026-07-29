/*
 * star/i6_vpe.h -- MI_VPE bindings, Infinity6E
 *
 * Vendored from OpenIPC divinus, src/hal/star/i6_vpe.h. See i6_common.h for
 * why these headers are vendored and for the four adaptations applied.
 *
 * IMPORTANT -- two channel/param layouts, and Infinity6E uses the i6e_ ones.
 * MI_VPE_CreateChannel and MI_VPE_SetChannelParam take a longer struct on
 * Infinity6E (i6e_vpe_chn / i6e_vpe_para, with the lens-distortion-correction
 * members) than on the original Infinity6 (i6_vpe_chn / i6_vpe_para).
 * divinus picks by SoC series at runtime and casts to the shorter type, since
 * that is what the function pointers are declared with -- see i6_hal.c:302-345,
 * `if (series == 0xF1)`. Our target is 0xF1 only, so the backend always
 * populates the i6e_ variants; the shorter ones are kept solely because the
 * function-pointer signatures name them and because keeping the file diffable
 * against upstream is worth more than trimming two structs.
 *
 * One divergence from divinus beyond the four common adaptations: the
 * loader below also opens libispalgo.so, libcus3a.so and libmi_isp.so.
 * divinus opens those in its i6_isp.h loader, which it happens to run
 * before its VPE loader; this backend has no MI_ISP binding until phase 3,
 * so the dependency lives with the module that has it. The reason it is
 * mandatory rather than tidy is in i6_vpe_load.
 *
 * Copyright (c) 2024 OpenIPC
 * SPDX-License-Identifier: MIT
 */

#ifndef STAR_I6_VPE_H
#define STAR_I6_VPE_H

#include "i6_common.h"

typedef enum {
    I6_VPE_MODE_INVALID,
    I6_VPE_MODE_DVR = 0x1,
    I6_VPE_MODE_CAM_TOP = 0x2,
    I6_VPE_MODE_CAM_BOTTOM = 0x4,
    I6_VPE_MODE_CAM = I6_VPE_MODE_CAM_TOP | I6_VPE_MODE_CAM_BOTTOM,
    I6_VPE_MODE_REALTIME_TOP = 0x8,
    I6_VPE_MODE_REALTIME_BOTTOM = 0x10,
    I6_VPE_MODE_REALTIME = I6_VPE_MODE_REALTIME_TOP | I6_VPE_MODE_REALTIME_BOTTOM,
    I6_VPE_MODE_END
} i6_vpe_mode;

typedef enum {
    I6_VPE_SENS_INVALID,
    I6_VPE_SENS_ID0,
    I6_VPE_SENS_ID1,
    I6_VPE_SENS_ID2,
    I6_VPE_SENS_ID3,
    I6_VPE_SENS_END,
} i6_vpe_sens;

typedef struct {
    unsigned int rev;
    unsigned int size;
    unsigned char data[64];
} i6_vpe_iqver;

typedef struct {
    int mode;
    char bypassOn;
    char proj3x3On;
    int proj3x3[9];
    unsigned short userSliceNum;
    unsigned int focalLengthX;
    unsigned int focalLengthY;
    void *configAddr;
    unsigned int configSize;
    int mapType;
    union {
        struct {
            void *xMapAddr, *yMapAddr;
            unsigned int xMapSize, yMapSize;
        } dispInfo;
        struct {
            void *calibPolyBinAddr;
            unsigned int calibPolyBinSize;
        } calibInfo;
    };
    char lensAdjOn;
} i6e_vpe_ildc;

typedef struct {
    char bypassOn;
    char proj3x3On;
    int proj3x3[9];
    unsigned int focalLengthX;
    unsigned int focalLengthY;
    void *configAddr;
    unsigned int configSize;
    union {
        struct {
            void *xMapAddr, *yMapAddr;
            unsigned int xMapSize, yMapSize;
        } dispInfo;
        struct {
            void *calibPolyBinAddr;
            unsigned int calibPolyBinSize;
        } calibInfo;
    };
} i6e_vpe_ldc;

typedef struct {
    i6_common_dim capt;
    i6_common_pixfmt pixFmt;
    i6_common_hdr hdr;
    i6_vpe_sens sensor;
    char noiseRedOn;
    char edgeOn;
    char edgeSmoothOn;
    char contrastOn;
    char invertOn;
    char rotateOn;
    i6_vpe_mode mode;
    i6_vpe_iqver iqparam;
    i6e_vpe_ildc lensInit;
    char lensAdjOn;
    unsigned int chnPort;
} i6e_vpe_chn;

typedef struct {
    i6_common_dim capt;
    i6_common_pixfmt pixFmt;
    i6_common_hdr hdr;
    i6_vpe_sens sensor;
    char noiseRedOn;
    char edgeOn;
    char edgeSmoothOn;
    char contrastOn;
    char invertOn;
    char rotateOn;
    i6_vpe_mode mode;
    i6_vpe_iqver iqparam;
    char lensAdjOn;
    unsigned int chnPort;
} i6_vpe_chn;

typedef struct {
    char reserved[16];
    i6e_vpe_ldc lensAdj;
    i6_common_hdr hdr;
    // Accepts values from 0-7
    int level3DNR;
    char mirror;
    char flip;
    char reserved2;
    char lensAdjOn;
} i6e_vpe_para;

typedef struct {
    char reserved[16];
    i6_common_hdr hdr;
    // Accepts values from 0-7
    int level3DNR;
    char mirror;
    char flip;
    char reserved2;
    char lensAdjOn;
} i6_vpe_para;

typedef struct {
    i6_common_dim output;
    char mirror;
    char flip;
    i6_common_pixfmt pixFmt;
    i6_common_compr compress;
} i6_vpe_port;

typedef struct {
    void *handle;

    /*
     * ISP-side libraries VPE needs resolved before it can run. Not used
     * directly -- held only so they stay mapped and can be closed. See
     * i6_vpe_load for why they must be opened here.
     */
    void *handleIspAlgo;
    void *handleCus3a;
    void *handleIsp;

    int (*fnCreateChannel)(int channel, i6_vpe_chn *config);
    int (*fnDestroyChannel)(int channel);
    int (*fnSetChannelConfig)(int channel, i6_vpe_chn *config);
    int (*fnSetChannelParam)(int channel, i6_vpe_para *config);
    int (*fnStartChannel)(int channel);
    int (*fnStopChannel)(int channel);

    int (*fnDisablePort)(int channel, int port);
    int (*fnEnablePort)(int channel, int port);
    int (*fnSetPortConfig)(int channel, int port, i6_vpe_port *config);

    /*
     * Optional -- guard every call. MI_VPE_SetPortMode returns 0 for a
     * geometry it does not apply: a port configured after the VPE channel
     * is already running keeps the channel's input size instead. Reading
     * the mode back is the only way to tell, and the crop is what makes
     * the scaler honour the request, since the driver gives an
     * input-domain crop only to ports configured before the channel runs.
     */
    int (*fnGetPortConfig)(int channel, int port, i6_vpe_port *config);
    int (*fnSetPortCrop)(int channel, int port, i6_common_rect *crop);
} i6_vpe_impl;

static inline int i6_vpe_load(i6_vpe_impl *vpe_lib)
{
    /*
     * Load the ISP side first, or MI_VPE_CreateChannel kills the process.
     *
     * libmi_vpe.so leaves MI_ISP_EnableUserspace3A and
     * MI_ISP_DisableUserspace3A undefined, and its DT_NEEDED lists only
     * libc -- so the dynamic loader chains nothing, and with RTLD_LAZY the
     * miss surfaces at first call as a fatal "symbol lookup error" rather
     * than as a dlopen failure we could report. The first board run of
     * star_probe -v died exactly there, immediately after MI_VPE_CreateChannel
     * was entered.
     *
     * One level deeper, libmi_isp.so itself needs libcus3a.so (7 symbols)
     * and libispalgo.so (14) and likewise names neither in DT_NEEDED, which
     * is why divinus opens all three in that order before ever touching VPE
     * (i6_isp.h:32-36, loaded at i6_hal.c:53, ten lines ahead of its VPE
     * load). RTLD_GLOBAL is what makes them satisfy the next library's
     * undefined symbols; dlopen refcounts, so phase 3's real MI_ISP binding
     * can open libmi_isp.so again without conflict.
     *
     * The algorithm libraries are best-effort: their symbols are reached
     * through libmi_isp, not from here, and a board missing them is a
     * broken ISP rather than a broken VPE. libmi_isp.so is not optional --
     * without it the next VPE call provably aborts.
     */
    vpe_lib->handleIspAlgo = dlopen("libispalgo.so", RTLD_LAZY | RTLD_GLOBAL);
    if (!vpe_lib->handleIspAlgo)
        HAL_LOG_WARN("i6_vpe: libispalgo.so not loaded (%s) -- ISP algorithms may fail",
                     dlerror());

    vpe_lib->handleCus3a = dlopen("libcus3a.so", RTLD_LAZY | RTLD_GLOBAL);
    if (!vpe_lib->handleCus3a)
        HAL_LOG_WARN("i6_vpe: libcus3a.so not loaded (%s) -- ISP algorithms may fail", dlerror());

    if (!(vpe_lib->handleIsp = dlopen("libmi_isp.so", RTLD_LAZY | RTLD_GLOBAL))) {
        HAL_LOG_ERR("i6_vpe: failed to load libmi_isp.so: %s", dlerror());
        return RSS_ERR_NOENT;
    }

    /*
     * Confirm the two symbols VPE actually reaches for. They are resolved
     * lazily inside libmi_vpe.so, so without this check a missing one is a
     * process abort mid-call instead of an error return -- and this loader
     * exists to make missing-SDK cases reportable.
     */
    if (!dlsym(vpe_lib->handleIsp, "MI_ISP_EnableUserspace3A")) {
        HAL_LOG_ERR("i6_vpe: libmi_isp.so lacks MI_ISP_EnableUserspace3A, which "
                    "libmi_vpe.so needs -- mismatched MI libraries?");
        return RSS_ERR_NOTSUP;
    }

    if (!(vpe_lib->handle = dlopen("libmi_vpe.so", RTLD_LAZY | RTLD_GLOBAL))) {
        HAL_LOG_ERR("i6_vpe: failed to load library: %s", dlerror());
        return RSS_ERR_NOENT;
    }

    if (!(vpe_lib->fnCreateChannel = (int(*)(int channel, i6_vpe_chn *config))
        hal_symbol_load("i6_vpe", vpe_lib->handle, "MI_VPE_CreateChannel")))
        return RSS_ERR_NOTSUP;

    if (!(vpe_lib->fnDestroyChannel = (int(*)(int channel))
        hal_symbol_load("i6_vpe", vpe_lib->handle, "MI_VPE_DestroyChannel")))
        return RSS_ERR_NOTSUP;

    if (!(vpe_lib->fnSetChannelConfig = (int(*)(int channel, i6_vpe_chn *config))
        hal_symbol_load("i6_vpe", vpe_lib->handle, "MI_VPE_SetChannelAttr")))
        return RSS_ERR_NOTSUP;

    if (!(vpe_lib->fnSetChannelParam = (int(*)(int channel, i6_vpe_para *config))
        hal_symbol_load("i6_vpe", vpe_lib->handle, "MI_VPE_SetChannelParam")))
        return RSS_ERR_NOTSUP;

    if (!(vpe_lib->fnStartChannel = (int(*)(int channel))
        hal_symbol_load("i6_vpe", vpe_lib->handle, "MI_VPE_StartChannel")))
        return RSS_ERR_NOTSUP;

    if (!(vpe_lib->fnStopChannel = (int(*)(int channel))
        hal_symbol_load("i6_vpe", vpe_lib->handle, "MI_VPE_StopChannel")))
        return RSS_ERR_NOTSUP;

    if (!(vpe_lib->fnDisablePort = (int(*)(int channel, int port))
        hal_symbol_load("i6_vpe", vpe_lib->handle, "MI_VPE_DisablePort")))
        return RSS_ERR_NOTSUP;

    if (!(vpe_lib->fnEnablePort = (int(*)(int channel, int port))
        hal_symbol_load("i6_vpe", vpe_lib->handle, "MI_VPE_EnablePort")))
        return RSS_ERR_NOTSUP;

    if (!(vpe_lib->fnSetPortConfig = (int(*)(int channel, int port, i6_vpe_port *config))
        hal_symbol_load("i6_vpe", vpe_lib->handle, "MI_VPE_SetPortMode")))
        return RSS_ERR_NOTSUP;

    /* Optional: absence costs the clone its verification, not the backend. */
    vpe_lib->fnGetPortConfig = (int(*)(int channel, int port, i6_vpe_port *config))
        hal_symbol_load("i6_vpe", vpe_lib->handle, "MI_VPE_GetPortMode");
    vpe_lib->fnSetPortCrop = (int(*)(int channel, int port, i6_common_rect *crop))
        hal_symbol_load("i6_vpe", vpe_lib->handle, "MI_VPE_SetPortCrop");

    return RSS_OK;
}

static inline void i6_vpe_unload(i6_vpe_impl *vpe_lib)
{
    /* Reverse of the load order: VPE first, then what it depended on. */
    if (vpe_lib->handle)
        dlclose(vpe_lib->handle);
    vpe_lib->handle = NULL;
    if (vpe_lib->handleIsp)
        dlclose(vpe_lib->handleIsp);
    vpe_lib->handleIsp = NULL;
    if (vpe_lib->handleCus3a)
        dlclose(vpe_lib->handleCus3a);
    vpe_lib->handleCus3a = NULL;
    if (vpe_lib->handleIspAlgo)
        dlclose(vpe_lib->handleIspAlgo);
    vpe_lib->handleIspAlgo = NULL;
    memset(vpe_lib, 0, sizeof(*vpe_lib));
}

#endif /* STAR_I6_VPE_H */
