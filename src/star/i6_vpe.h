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

    int (*fnCreateChannel)(int channel, i6_vpe_chn *config);
    int (*fnDestroyChannel)(int channel);
    int (*fnSetChannelConfig)(int channel, i6_vpe_chn *config);
    int (*fnSetChannelParam)(int channel, i6_vpe_para *config);
    int (*fnStartChannel)(int channel);
    int (*fnStopChannel)(int channel);

    int (*fnDisablePort)(int channel, int port);
    int (*fnEnablePort)(int channel, int port);
    int (*fnSetPortConfig)(int channel, int port, i6_vpe_port *config);
} i6_vpe_impl;

static inline int i6_vpe_load(i6_vpe_impl *vpe_lib)
{
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

    return RSS_OK;
}

static inline void i6_vpe_unload(i6_vpe_impl *vpe_lib)
{
    if (vpe_lib->handle)
        dlclose(vpe_lib->handle);
    vpe_lib->handle = NULL;
    memset(vpe_lib, 0, sizeof(*vpe_lib));
}

#endif /* STAR_I6_VPE_H */
