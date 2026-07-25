/*
 * star/i6_snr.h -- MI_SNR bindings, Infinity6E
 *
 * Vendored from OpenIPC divinus, src/hal/star/i6_snr.h. See i6_common.h for
 * why these headers are vendored and for the four adaptations applied.
 *
 * Note that MI_SNR is entirely index-based: resolutions are queried by count
 * and index, and no call anywhere in this API names a sensor. The sensor
 * identity is fixed when sensor_<name>_mipi.ko is insmod'd, so raptor's
 * sensor "discovery" is a module check plus the geometry queries below --
 * not the probing the Ingenic backend does.
 *
 * Copyright (c) 2024 OpenIPC
 * SPDX-License-Identifier: MIT
 */

#ifndef STAR_I6_SNR_H
#define STAR_I6_SNR_H

#include "i6_common.h"

typedef enum {
    I6_SNR_HWHDR_NONE,
    I6_SNR_HWHDR_SONY_DOL,
    I6_SNR_HWHDR_DCG,
    I6_SNR_HWHDR_EMBED_RAW8,
    I6_SNR_HWHDR_EMBED_RAW10,
    I6_SNR_HWHDR_EMBED_RAW12,
    I6_SNR_HWHDR_EMBED_RAW16
} i6_snr_hwhdr;

typedef struct {
    unsigned int laneCnt;
    unsigned int rgbFmtOn;
    i6_common_input input;
    unsigned int hsyncMode;
    unsigned int sampDelay;
    i6_snr_hwhdr hwHdr;
    unsigned int virtChn;
    unsigned int packType[2];
} i6_snr_mipi;

typedef struct {
    unsigned int multplxNum;
    i6_common_sync sync;
    i6_common_edge edge;
    int bitswap;
} i6_snr_bt656;

typedef struct {
    i6_common_sync sync;
} i6_snr_par;

typedef union {
    i6_snr_par parallel;
    i6_snr_mipi mipi;
    i6_snr_bt656 bt656;
} i6_snr_intfattr;

typedef struct {
    unsigned int planeCnt;
    i6_common_intf intf;
    i6_common_hdr hdr;
    i6_snr_intfattr intfAttr;
    char earlyInit;
} i6_snr_pad;

typedef struct {
    unsigned int planeId;
    char sensName[32];
    i6_common_rect capt;
    i6_common_bayer bayer;
    i6_common_prec precision;
    int hdrSrc;
    // Value in microseconds
    unsigned int shutter;
    // Value multiplied by 1024
    unsigned int sensGain;
    unsigned int compGain;
    i6_common_pixfmt pixFmt;
} i6_snr_plane;

typedef struct {
    i6_common_rect crop;
    i6_common_dim output;
    unsigned int maxFps;
    unsigned int minFps;
    char desc[32];
} __attribute__((packed, aligned(4))) i6_snr_res;

typedef struct {
    void *handle;

    int (*fnDisable)(unsigned int sensor);
    int (*fnEnable)(unsigned int sensor);

    int (*fnSetFramerate)(unsigned int sensor, unsigned int framerate);
    int (*fnSetOrientation)(unsigned int sensor, unsigned char mirror, unsigned char flip);

    int (*fnGetPadInfo)(unsigned int sensor, i6_snr_pad *info);
    int (*fnGetPlaneInfo)(unsigned int sensor, unsigned int index, i6_snr_plane *info);
    int (*fnSetPlaneMode)(unsigned int sensor, unsigned char active);

    int (*fnCurrentResolution)(unsigned int sensor, unsigned char *index, i6_snr_res *resolution);
    int (*fnGetResolution)(unsigned int sensor, unsigned char index, i6_snr_res *resolution);
    int (*fnGetResolutionCount)(unsigned int sensor, unsigned int *count);
    int (*fnSetResolution)(unsigned int sensor, unsigned char index);

    int (*fnCustomFunction)(unsigned int sensor, unsigned int command, unsigned int size,
        void *data, int drvOrUsr);
} i6_snr_impl;

static inline int i6_snr_load(i6_snr_impl *snr_lib)
{
    if (!(snr_lib->handle = dlopen("libmi_sensor.so", RTLD_LAZY | RTLD_GLOBAL))) {
        HAL_LOG_ERR("i6_snr: failed to load library: %s", dlerror());
        return RSS_ERR_NOENT;
    }

    if (!(snr_lib->fnDisable = (int(*)(unsigned int sensor))
        hal_symbol_load("i6_snr", snr_lib->handle, "MI_SNR_Disable")))
        return RSS_ERR_NOTSUP;

    if (!(snr_lib->fnEnable = (int(*)(unsigned int sensor))
        hal_symbol_load("i6_snr", snr_lib->handle, "MI_SNR_Enable")))
        return RSS_ERR_NOTSUP;

    if (!(snr_lib->fnSetFramerate = (int(*)(unsigned int sensor, unsigned int framerate))
        hal_symbol_load("i6_snr", snr_lib->handle, "MI_SNR_SetFps")))
        return RSS_ERR_NOTSUP;

    if (!(snr_lib->fnSetOrientation = (int(*)(unsigned int sensor, unsigned char mirror,
        unsigned char flip))
        hal_symbol_load("i6_snr", snr_lib->handle, "MI_SNR_SetOrien")))
        return RSS_ERR_NOTSUP;

    if (!(snr_lib->fnGetPadInfo = (int(*)(unsigned int sensor, i6_snr_pad *info))
        hal_symbol_load("i6_snr", snr_lib->handle, "MI_SNR_GetPadInfo")))
        return RSS_ERR_NOTSUP;

    if (!(snr_lib->fnGetPlaneInfo = (int(*)(unsigned int sensor, unsigned int index,
        i6_snr_plane *info))
        hal_symbol_load("i6_snr", snr_lib->handle, "MI_SNR_GetPlaneInfo")))
        return RSS_ERR_NOTSUP;

    if (!(snr_lib->fnSetPlaneMode = (int(*)(unsigned int sensor, unsigned char active))
        hal_symbol_load("i6_snr", snr_lib->handle, "MI_SNR_SetPlaneMode")))
        return RSS_ERR_NOTSUP;

    if (!(snr_lib->fnCurrentResolution = (int(*)(unsigned int sensor, unsigned char *index,
        i6_snr_res *resolution))
        hal_symbol_load("i6_snr", snr_lib->handle, "MI_SNR_GetCurRes")))
        return RSS_ERR_NOTSUP;

    if (!(snr_lib->fnGetResolution = (int(*)(unsigned int sensor, unsigned char index,
        i6_snr_res *resolution))
        hal_symbol_load("i6_snr", snr_lib->handle, "MI_SNR_GetRes")))
        return RSS_ERR_NOTSUP;

    if (!(snr_lib->fnGetResolutionCount = (int(*)(unsigned int sensor, unsigned int *count))
        hal_symbol_load("i6_snr", snr_lib->handle, "MI_SNR_QueryResCount")))
        return RSS_ERR_NOTSUP;

    if (!(snr_lib->fnSetResolution = (int(*)(unsigned int sensor, unsigned char index))
        hal_symbol_load("i6_snr", snr_lib->handle, "MI_SNR_SetRes")))
        return RSS_ERR_NOTSUP;

    if (!(snr_lib->fnCustomFunction = (int(*)(unsigned int sensor, unsigned int command,
        unsigned int size, void *data, int drvOrUsr))
        hal_symbol_load("i6_snr", snr_lib->handle, "MI_SNR_CustFunction")))
        return RSS_ERR_NOTSUP;

    return RSS_OK;
}

static inline void i6_snr_unload(i6_snr_impl *snr_lib)
{
    if (snr_lib->handle)
        dlclose(snr_lib->handle);
    snr_lib->handle = NULL;
    memset(snr_lib, 0, sizeof(*snr_lib));
}

#endif /* STAR_I6_SNR_H */
