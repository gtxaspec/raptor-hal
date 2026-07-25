/*
 * star/i6_venc.h -- MI_VENC bindings, Infinity6E
 *
 * Vendored from OpenIPC divinus, src/hal/star/i6_venc.h. See i6_common.h for
 * why these headers are vendored and for the four adaptations applied.
 *
 * Two things to know before using this:
 *
 *  - Rate-control mode numbering differs between the original Infinity6
 *    (series 0xEF, I6OG_VENC_RATEMODE_*) and everything later, because 0xEF
 *    has no H264ABR and the values after it shift by one. Infinity6E is
 *    0xF1, so use the plain I6_VENC_RATEMODE_* set; the I6OG_ enum is kept
 *    only to stay diffable against upstream, which supports both.
 *  - fnSetSourceConfig is intentionally not fatal to load. divinus does not
 *    check it either: MI_VENC_SetInputSourceConfig is absent from some
 *    firmware builds' libmi_venc, and it is an optimization (ring-buffer
 *    input mode) rather than a requirement. Null-check it at every call site.
 *    It is resolved with a bare dlsym rather than hal_symbol_load -- the one
 *    place this file deviates from upstream beyond the four common
 *    adaptations -- because logging at ERR level for an optional symbol
 *    would report a non-problem as a failure.
 *
 * I6_VENC_CHN_NUM below is where hal_caps.c's INFINITY6E max_enc_channels
 * comes from -- 9 addressable channels, capped to RSS_MAX_ENC_CHANNELS.
 *
 * Copyright (c) 2024 OpenIPC
 * SPDX-License-Identifier: MIT
 */

#ifndef STAR_I6_VENC_H
#define STAR_I6_VENC_H

#include "i6_common.h"

#define I6_VENC_CHN_NUM 9

typedef enum {
    I6_VENC_CODEC_H264 = 2,
    I6_VENC_CODEC_H265,
    I6_VENC_CODEC_MJPG,
    I6_VENC_CODEC_END
} i6_venc_codec;

typedef enum {
    I6_VENC_NALU_H264_PSLICE = 1,
    I6_VENC_NALU_H264_ISLICE = 5,
    I6_VENC_NALU_H264_SEI,
    I6_VENC_NALU_H264_SPS,
    I6_VENC_NALU_H264_PPS,
    I6_VENC_NALU_H264_IPSLICE,
    I6_VENC_NALU_H264_PREFIX = 14,
    I6_VENC_NALU_H264_END
} i6_venc_nalu_h264;

typedef enum {
    I6_VENC_NALU_H265_PSLICE = 1,
    I6_VENC_NALU_H265_ISLICE = 19,
    I6_VENC_NALU_H265_VPS = 32,
    I6_VENC_NALU_H265_SPS,
    I6_VENC_NALU_H265_PPS,
    I6_VENC_NALU_H265_SEI = 39,
    I6_VENC_NALU_H265_END
} i6_venc_nalu_h265;

typedef enum {
    I6_VENC_NALU_MJPG_ECS = 5,
    I6_VENC_NALU_MJPG_APP,
    I6_VENC_NALU_MJPG_VDO,
    I6_VENC_NALU_MJPG_PIC,
    I6_VENC_NALU_MJPG_END
} i6_venc_nalu_mjpg;

typedef enum {
    I6_VENC_SRC_CONF_NORMAL,
    I6_VENC_SRC_CONF_RING_ONE,
    I6_VENC_SRC_CONF_RING_HALF,
    I6_VENC_SRC_CONF_END
} i6_venc_src_conf;

typedef enum {
    I6_VENC_RATEMODE_H264CBR = 1,
    I6_VENC_RATEMODE_H264VBR,
    I6_VENC_RATEMODE_H264ABR,
    I6_VENC_RATEMODE_H264QP,
    I6_VENC_RATEMODE_H264AVBR,
    I6_VENC_RATEMODE_MJPGCBR,
    I6_VENC_RATEMODE_MJPGQP,
    I6_VENC_RATEMODE_H265CBR,
    I6_VENC_RATEMODE_H265VBR,
    I6_VENC_RATEMODE_H265QP,
    I6_VENC_RATEMODE_H265AVBR,
    I6_VENC_RATEMODE_END
} i6_venc_ratemode;

typedef enum {
    I6OG_VENC_RATEMODE_H264CBR = 1,
    I6OG_VENC_RATEMODE_H264VBR,
    I6OG_VENC_RATEMODE_H264QP,
    I6OG_VENC_RATEMODE_H264AVBR,
    I6OG_VENC_RATEMODE_MJPGCBR,
    I6OG_VENC_RATEMODE_MJPGQP,
    I6OG_VENC_RATEMODE_H265CBR,
    I6OG_VENC_RATEMODE_H265VBR,
    I6OG_VENC_RATEMODE_H265QP,
    I6OG_VENC_RATEMODE_H265AVBR,
    I6OG_VENC_RATEMODE_END
} i6og_venc_ratemode;

typedef struct {
    unsigned int maxWidth;
    unsigned int maxHeight;
    unsigned int bufSize;
    unsigned int profile;
    char byFrame;
    unsigned int width;
    unsigned int height;
    unsigned int bFrameNum;
    unsigned int refNum;
} i6_venc_attr_h26x;

typedef struct {
    unsigned int maxWidth;
    unsigned int maxHeight;
    unsigned int bufSize;
    char byFrame;
    unsigned int width;
    unsigned int height;
    char dcfThumbs;
    unsigned int markPerRow;
} i6_venc_attr_mjpg;

typedef struct {
    i6_venc_codec codec;
    union {
        i6_venc_attr_h26x h264;
        i6_venc_attr_mjpg mjpg;
        i6_venc_attr_h26x h265;
    };
} i6_venc_attrib;

typedef struct {
    unsigned int gop;
    unsigned int statTime;
    unsigned int fpsNum;
    unsigned int fpsDen;
    unsigned int bitrate;
    unsigned int avgLvl;
} i6_venc_rate_h26xcbr;

typedef struct {
    unsigned int gop;
    unsigned int statTime;
    unsigned int fpsNum;
    unsigned int fpsDen;
    unsigned int maxBitrate;
    unsigned int maxQual;
    unsigned int minQual;
} i6_venc_rate_h26xvbr;

typedef struct {
    unsigned int gop;
    unsigned int fpsNum;
    unsigned int fpsDen;
    unsigned int interQual;
    unsigned int predQual;
} i6_venc_rate_h26xqp;

typedef struct {
    unsigned int gop;
    unsigned int statTime;
    unsigned int fpsNum;
    unsigned int fpsDen;
    unsigned int avgBitrate;
    unsigned int maxBitrate;
} i6_venc_rate_h26xabr;

typedef struct {
    unsigned int bitrate;
    unsigned int fpsNum;
    unsigned int fpsDen;
} i6_venc_rate_mjpgcbr;

typedef struct {
    unsigned int fpsNum;
    unsigned int fpsDen;
    unsigned int quality;
} i6_venc_rate_mjpgqp;

typedef struct {
    i6_venc_ratemode mode;
    union {
        i6_venc_rate_h26xcbr h264Cbr;
        i6_venc_rate_h26xvbr h264Vbr;
        i6_venc_rate_h26xqp h264Qp;
        i6_venc_rate_h26xabr h264Abr;
        i6_venc_rate_h26xvbr h264Avbr;
        i6_venc_rate_mjpgcbr mjpgCbr;
        i6_venc_rate_mjpgqp mjpgQp;
        i6_venc_rate_h26xcbr h265Cbr;
        i6_venc_rate_h26xvbr h265Vbr;
        i6_venc_rate_h26xqp h265Qp;
        i6_venc_rate_h26xvbr h265Avbr;
    };
    void *extend;
} i6_venc_rate;

typedef struct {
    i6_venc_attrib attrib;
    i6_venc_rate rate;
} i6_venc_chn;

typedef struct {
    unsigned int quality;
    unsigned char qtLuma[64];
    unsigned char qtChroma[64];
    unsigned int mcuPerEcs;
} i6_venc_jpg;

typedef union {
    i6_venc_nalu_h264 h264Nalu;
    i6_venc_nalu_mjpg mjpgNalu;
    i6_venc_nalu_h265 h265Nalu;
} i6_venc_nalu;

typedef struct {
    i6_venc_nalu packType;
    unsigned int offset;
    unsigned int length;
    unsigned int sliceId;
} i6_venc_packinfo;

typedef struct {
    unsigned long long addr;
    unsigned char *data;
    unsigned int length;
    unsigned long long timestamp;
    char endFrame;
    i6_venc_nalu naluType;
    unsigned int offset;
    unsigned int packNum;
    i6_venc_packinfo packetInfo[8];
} i6_venc_pack;

typedef struct {
    unsigned int leftPics;
    unsigned int leftBytes;
    unsigned int leftFrames;
    unsigned int leftMillis;
    unsigned int curPacks;
    unsigned int leftRecvPics;
    unsigned int leftEncPics;
    unsigned int fpsNum;
    unsigned int fpsDen;
    unsigned int bitrate;
} i6_venc_stat;

typedef struct {
    unsigned int size;
    unsigned int skipMb;
    unsigned int ipcmMb;
    unsigned int iMb16x8;
    unsigned int iMb16x16;
    unsigned int iMb8x16;
    unsigned int iMb8x8;
    unsigned int pMb16;
    unsigned int pMb8;
    unsigned int pMb4;
    unsigned int refSliceType;
    unsigned int refType;
    unsigned int updAttrCnt;
    unsigned int startQual;
} i6_venc_strminfo_h264;

typedef struct {
    unsigned int size;
    unsigned int iCu64x64;
    unsigned int iCu32x32;
    unsigned int iCu16x16;
    unsigned int iCu8x8;
    unsigned int pCu32x32;
    unsigned int pCu16x16;
    unsigned int pCu8x8;
    unsigned int pCu4x4;
    unsigned int refType;
    unsigned int updAttrCnt;
    unsigned int startQual;
} i6_venc_strminfo_h265;

typedef struct {
    unsigned int size;
    unsigned int updAttrCnt;
    unsigned int quality;
} i6_venc_strminfo_mjpg;

typedef struct {
    i6_venc_pack *packet;
    unsigned int count;
    unsigned int sequence;
    int handle;
    union {
        i6_venc_strminfo_h264 h264Info;
        i6_venc_strminfo_mjpg mjpgInfo;
        i6_venc_strminfo_h265 h265Info;
    };
} i6_venc_strm;

typedef struct {
    void *handle;

    int (*fnCreateChannel)(int channel, i6_venc_chn *config);
    int (*fnDestroyChannel)(int channel);
    int (*fnGetChannelConfig)(int channel, i6_venc_chn *config);
    int (*fnGetChannelDeviceId)(int channel, unsigned int *device);
    int (*fnResetChannel)(int channel);
    int (*fnSetChannelConfig)(int channel, i6_venc_chn *config);

    int (*fnFreeDescriptor)(int channel);
    int (*fnGetDescriptor)(int channel);

    int (*fnGetJpegParam)(int channel, i6_venc_jpg *param);
    int (*fnSetJpegParam)(int channel, i6_venc_jpg *param);

    int (*fnFreeStream)(int channel, i6_venc_strm *stream);
    int (*fnGetStream)(int channel, i6_venc_strm *stream, unsigned int timeout);

    int (*fnQuery)(int channel, i6_venc_stat *stats);

    int (*fnSetSourceConfig)(int channel, i6_venc_src_conf *config);

    int (*fnRequestIdr)(int channel, char instant);
    int (*fnStartReceiving)(int channel);
    int (*fnStartReceivingEx)(int channel, int *count);
    int (*fnStopReceiving)(int channel);
} i6_venc_impl;

static inline int i6_venc_load(i6_venc_impl *venc_lib)
{
    if (!(venc_lib->handle = dlopen("libmi_venc.so", RTLD_LAZY | RTLD_GLOBAL))) {
        HAL_LOG_ERR("i6_venc: failed to load library: %s", dlerror());
        return RSS_ERR_NOENT;
    }

    if (!(venc_lib->fnCreateChannel = (int(*)(int channel, i6_venc_chn *config))
        hal_symbol_load("i6_venc", venc_lib->handle, "MI_VENC_CreateChn")))
        return RSS_ERR_NOTSUP;

    if (!(venc_lib->fnDestroyChannel = (int(*)(int channel))
        hal_symbol_load("i6_venc", venc_lib->handle, "MI_VENC_DestroyChn")))
        return RSS_ERR_NOTSUP;

    if (!(venc_lib->fnGetChannelConfig = (int(*)(int channel, i6_venc_chn *config))
        hal_symbol_load("i6_venc", venc_lib->handle, "MI_VENC_GetChnAttr")))
        return RSS_ERR_NOTSUP;

    if (!(venc_lib->fnGetChannelDeviceId = (int(*)(int channel, unsigned int *device))
        hal_symbol_load("i6_venc", venc_lib->handle, "MI_VENC_GetChnDevid")))
        return RSS_ERR_NOTSUP;

    if (!(venc_lib->fnResetChannel = (int(*)(int channel))
        hal_symbol_load("i6_venc", venc_lib->handle, "MI_VENC_ResetChn")))
        return RSS_ERR_NOTSUP;

    if (!(venc_lib->fnSetChannelConfig = (int(*)(int channel, i6_venc_chn *config))
        hal_symbol_load("i6_venc", venc_lib->handle, "MI_VENC_SetChnAttr")))
        return RSS_ERR_NOTSUP;

    if (!(venc_lib->fnFreeDescriptor = (int(*)(int channel))
        hal_symbol_load("i6_venc", venc_lib->handle, "MI_VENC_CloseFd")))
        return RSS_ERR_NOTSUP;

    if (!(venc_lib->fnGetDescriptor = (int(*)(int channel))
        hal_symbol_load("i6_venc", venc_lib->handle, "MI_VENC_GetFd")))
        return RSS_ERR_NOTSUP;

    if (!(venc_lib->fnGetJpegParam = (int(*)(int channel, i6_venc_jpg *param))
        hal_symbol_load("i6_venc", venc_lib->handle, "MI_VENC_GetJpegParam")))
        return RSS_ERR_NOTSUP;

    if (!(venc_lib->fnSetJpegParam = (int(*)(int channel, i6_venc_jpg *param))
        hal_symbol_load("i6_venc", venc_lib->handle, "MI_VENC_SetJpegParam")))
        return RSS_ERR_NOTSUP;

    if (!(venc_lib->fnFreeStream = (int(*)(int channel, i6_venc_strm *stream))
        hal_symbol_load("i6_venc", venc_lib->handle, "MI_VENC_ReleaseStream")))
        return RSS_ERR_NOTSUP;

    if (!(venc_lib->fnGetStream = (int(*)(int channel, i6_venc_strm *stream,
        unsigned int timeout))
        hal_symbol_load("i6_venc", venc_lib->handle, "MI_VENC_GetStream")))
        return RSS_ERR_NOTSUP;

    if (!(venc_lib->fnQuery = (int(*)(int channel, i6_venc_stat *stats))
        hal_symbol_load("i6_venc", venc_lib->handle, "MI_VENC_Query")))
        return RSS_ERR_NOTSUP;

    /* Optional — see the file header. Absence is not an error. */
    venc_lib->fnSetSourceConfig = (int(*)(int channel, i6_venc_src_conf *config))
        dlsym(venc_lib->handle, "MI_VENC_SetInputSourceConfig");

    if (!(venc_lib->fnRequestIdr = (int(*)(int channel, char instant))
        hal_symbol_load("i6_venc", venc_lib->handle, "MI_VENC_RequestIdr")))
        return RSS_ERR_NOTSUP;

    if (!(venc_lib->fnStartReceiving = (int(*)(int channel))
        hal_symbol_load("i6_venc", venc_lib->handle, "MI_VENC_StartRecvPic")))
        return RSS_ERR_NOTSUP;

    if (!(venc_lib->fnStartReceivingEx = (int(*)(int channel, int *count))
        hal_symbol_load("i6_venc", venc_lib->handle, "MI_VENC_StartRecvPicEx")))
        return RSS_ERR_NOTSUP;

    if (!(venc_lib->fnStopReceiving = (int(*)(int channel))
        hal_symbol_load("i6_venc", venc_lib->handle, "MI_VENC_StopRecvPic")))
        return RSS_ERR_NOTSUP;

    return RSS_OK;
}

static inline void i6_venc_unload(i6_venc_impl *venc_lib)
{
    if (venc_lib->handle)
        dlclose(venc_lib->handle);
    venc_lib->handle = NULL;
    memset(venc_lib, 0, sizeof(*venc_lib));
}

#endif /* STAR_I6_VENC_H */
