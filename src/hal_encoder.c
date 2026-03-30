/*
 * hal_encoder.c -- Raptor HAL encoder implementation
 *
 * Implements all encoder vtable functions for both old-style
 * (T20/T21/T23/T30) and new-style (T31/T32/T40/T41) SDKs.
 *
 * Compiled for every target platform; SDK differences are handled
 * with #ifdef HAL_OLD_SDK / HAL_NEW_SDK blocks.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: MIT
 */

#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "hal_internal.h"

/* ── Constants ─────────────────────────────────────────────────────── */

#define RSS_MAX_ENC_CHANNELS 8
#define RSS_MAX_NALS_PER_FRAME 16

/* Scratch buffer size for ring-buffer linearization (new SDK) */
#define RSS_SCRATCH_DEFAULT_SIZE (512 * 1024)

/* ══════════════════════════════════════════════════════════════════════
 * Translation Helpers
 * ══════════════════════════════════════════════════════════════════════ */

/*
 * hal_translate_codec -- convert rss_codec_t to vendor payload type.
 *
 * Old SDK: returns IMPPayloadType (PT_H264, PT_H265, PT_JPEG).
 * New SDK: returns IMPEncoderProfile (composite type|profile).
 */
#if defined(HAL_OLD_SDK)
static IMPPayloadType hal_translate_codec(rss_codec_t codec)
{
    switch (codec) {
    case RSS_CODEC_H264:
        return PT_H264;
#if !defined(PLATFORM_T20)
    case RSS_CODEC_H265:
        return PT_H265;
#endif
    case RSS_CODEC_JPEG: /* fall through */
    case RSS_CODEC_MJPEG:
        return PT_JPEG;
    default:
        return PT_H264;
    }
}
#endif

#if defined(HAL_NEW_SDK)
static IMPEncoderProfile hal_translate_profile(rss_codec_t codec, int profile)
{
    switch (codec) {
    case RSS_CODEC_H264:
        if (profile == 0)
            return IMP_ENC_PROFILE_AVC_BASELINE;
        if (profile == 1)
            return IMP_ENC_PROFILE_AVC_MAIN;
        return IMP_ENC_PROFILE_AVC_HIGH;
    case RSS_CODEC_H265:
        return IMP_ENC_PROFILE_HEVC_MAIN;
    case RSS_CODEC_JPEG: /* fall through */
    case RSS_CODEC_MJPEG:
        return IMP_ENC_PROFILE_JPEG;
    default:
        return IMP_ENC_PROFILE_AVC_HIGH;
    }
}
#endif

/*
 * hal_translate_rc_mode -- convert rss_rc_mode_t to vendor RC enum.
 */
static IMPEncoderRcMode hal_translate_rc_mode(rss_rc_mode_t mode)
{
#if defined(HAL_NEW_SDK)
    switch (mode) {
    case RSS_RC_FIXQP:
        return IMP_ENC_RC_MODE_FIXQP;
    case RSS_RC_CBR:
        return IMP_ENC_RC_MODE_CBR;
    case RSS_RC_VBR:
        return IMP_ENC_RC_MODE_VBR;
#if defined(PLATFORM_T32)
    case RSS_RC_SMART:
        return IMP_ENC_RC_MODE_SMART;
#else
    case RSS_RC_SMART:
        return IMP_ENC_RC_MODE_CAPPED_VBR;
#endif
    case RSS_RC_CAPPED_VBR:
        return IMP_ENC_RC_MODE_CAPPED_VBR;
    case RSS_RC_CAPPED_QUALITY:
        return IMP_ENC_RC_MODE_CAPPED_QUALITY;
    default:
        return IMP_ENC_RC_MODE_CBR;
    }
#else
    switch (mode) {
    case RSS_RC_FIXQP:
        return ENC_RC_MODE_FIXQP;
    case RSS_RC_CBR:
        return ENC_RC_MODE_CBR;
    case RSS_RC_VBR:
        return ENC_RC_MODE_VBR;
    case RSS_RC_SMART:
        return ENC_RC_MODE_SMART;
    case RSS_RC_CAPPED_VBR:
        return ENC_RC_MODE_VBR;
    case RSS_RC_CAPPED_QUALITY:
        return ENC_RC_MODE_VBR;
    default:
        return ENC_RC_MODE_CBR;
    }
#endif
}

/*
 * hal_translate_nal_type_h264 -- vendor H264 NAL enum to rss_nal_type_t.
 */
static rss_nal_type_t hal_translate_nal_type_h264(IMPEncoderH264NaluType nal)
{
    switch (nal) {
    case IMP_H264_NAL_SPS:
        return RSS_NAL_H264_SPS;
    case IMP_H264_NAL_PPS:
        return RSS_NAL_H264_PPS;
    case IMP_H264_NAL_SEI:
        return RSS_NAL_H264_SEI;
    case IMP_H264_NAL_SLICE_IDR:
        return RSS_NAL_H264_IDR;
    case IMP_H264_NAL_SLICE:
        return RSS_NAL_H264_SLICE;
    default:
        return RSS_NAL_UNKNOWN;
    }
}

/*
 * hal_translate_nal_type_h265 -- vendor H265 NAL enum to rss_nal_type_t.
 * Not available on T20 (no H265 support).
 */
#if !defined(PLATFORM_T20)
static rss_nal_type_t hal_translate_nal_type_h265(IMPEncoderH265NaluType nal)
{
    switch (nal) {
    case IMP_H265_NAL_VPS:
        return RSS_NAL_H265_VPS;
    case IMP_H265_NAL_SPS:
        return RSS_NAL_H265_SPS;
    case IMP_H265_NAL_PPS:
        return RSS_NAL_H265_PPS;
    case IMP_H265_NAL_PREFIX_SEI:
        return RSS_NAL_H265_SEI;
    case IMP_H265_NAL_SUFFIX_SEI:
        return RSS_NAL_H265_SEI;
    case IMP_H265_NAL_SLICE_IDR_W_RADL:
        return RSS_NAL_H265_IDR;
    case IMP_H265_NAL_SLICE_IDR_N_LP:
        return RSS_NAL_H265_IDR;
    case IMP_H265_NAL_SLICE_CRA:
        return RSS_NAL_H265_IDR;
    case IMP_H265_NAL_SLICE_BLA_W_LP:
        return RSS_NAL_H265_IDR;
    case IMP_H265_NAL_SLICE_BLA_W_RADL:
        return RSS_NAL_H265_IDR;
    case IMP_H265_NAL_SLICE_BLA_N_LP:
        return RSS_NAL_H265_IDR;
    default:
        /* TRAIL_N/TRAIL_R/TSA/STSA/RADL/RASL are all regular slices */
        if (nal <= IMP_H265_NAL_SLICE_RASL_R)
            return RSS_NAL_H265_SLICE;
        return RSS_NAL_UNKNOWN;
    }
}
#endif

/*
 * hal_is_idr_h264 -- check if a vendor H264 NAL type is an IDR.
 */
static bool hal_is_idr_h264(IMPEncoderH264NaluType nal)
{
    return nal == IMP_H264_NAL_SLICE_IDR;
}

/*
 * hal_is_idr_h265 -- check if a vendor H265 NAL type is an IDR.
 * Not available on T20 (no H265 support).
 */
#if !defined(PLATFORM_T20)
static bool hal_is_idr_h265(IMPEncoderH265NaluType nal)
{
    return nal == IMP_H265_NAL_SLICE_IDR_W_RADL || nal == IMP_H265_NAL_SLICE_IDR_N_LP ||
           nal == IMP_H265_NAL_SLICE_CRA || nal == IMP_H265_NAL_SLICE_BLA_W_LP ||
           nal == IMP_H265_NAL_SLICE_BLA_W_RADL || nal == IMP_H265_NAL_SLICE_BLA_N_LP;
}
#endif

/*
 * hal_ensure_nal_array -- grow the per-channel NAL array if needed.
 */
static int hal_ensure_nal_array(rss_hal_ctx_t *c, int chn, int count)
{
    if (chn < 0 || chn >= RSS_MAX_ENC_CHANNELS)
        return -EINVAL;

    if (c->nal_array_caps[chn] >= count)
        return 0;

    int new_cap = count > RSS_MAX_NALS_PER_FRAME ? count : RSS_MAX_NALS_PER_FRAME;
    rss_nal_unit_t *arr =
        (rss_nal_unit_t *)realloc(c->nal_arrays[chn], new_cap * sizeof(rss_nal_unit_t));
    if (!arr)
        return -ENOMEM;

    c->nal_arrays[chn] = arr;
    c->nal_array_caps[chn] = new_cap;
    return 0;
}

#if defined(HAL_NEW_SDK) && !defined(PLATFORM_T32)
/*
 * hal_ensure_scratch -- ensure the scratch linearization buffer exists.
 * Only needed on new SDK (except T32) for ring-buffer pack linearization.
 */
static int hal_ensure_scratch(rss_hal_ctx_t *c, size_t needed)
{
    if (c->scratch_size >= needed)
        return 0;

    size_t alloc = needed > RSS_SCRATCH_DEFAULT_SIZE ? needed : RSS_SCRATCH_DEFAULT_SIZE;
    uint8_t *buf = (uint8_t *)realloc(c->scratch_buf, alloc);
    if (!buf)
        return -ENOMEM;

    c->scratch_buf = buf;
    c->scratch_size = alloc;
    return 0;
}
#endif /* HAL_NEW_SDK */

/* ══════════════════════════════════════════════════════════════════════
 * 1. Group Management
 * ══════════════════════════════════════════════════════════════════════ */

int hal_enc_create_group(void *ctx, int grp)
{
    (void)ctx;
    int ret = IMP_Encoder_CreateGroup(grp);
    if (ret != 0)
        HAL_LOG_ERR("IMP_Encoder_CreateGroup(%d) failed: %d", grp, ret);
    return ret;
}

int hal_enc_destroy_group(void *ctx, int grp)
{
    (void)ctx;
    int ret = IMP_Encoder_DestroyGroup(grp);
    if (ret != 0)
        HAL_LOG_ERR("IMP_Encoder_DestroyGroup(%d) failed: %d", grp, ret);
    return ret;
}

/* ══════════════════════════════════════════════════════════════════════
 * 2. Channel Creation
 * ══════════════════════════════════════════════════════════════════════ */

#if defined(HAL_OLD_SDK)
/*
 * Old SDK channel creation (T20, T21, T23, T30):
 *
 * Build IMPEncoderCHNAttr with:
 *   encAttr.enType     = PT_H264 / PT_H265 / PT_JPEG
 *   encAttr.picWidth   = width
 *   encAttr.picHeight  = height
 *   encAttr.bufSize    = buf_size (0 = auto)
 *   encAttr.profile    = H264 profile (0/1/2)
 *   rcAttr.outFrmRate  = { fps_num, fps_den }
 *   rcAttr.maxGop      = gop_length
 *   rcAttr.attrRcMode  = per-codec RC union
 */
static int hal_enc_create_channel_old(int chn, const rss_video_config_t *cfg)
{
    IMPEncoderCHNAttr chnAttr;
    IMPPayloadType pt;
    IMPEncoderRcMode rc;

    memset(&chnAttr, 0, sizeof(chnAttr));

    pt = hal_translate_codec(cfg->codec);
    rc = hal_translate_rc_mode(cfg->rc_mode);

    /* Encoder attributes */
    chnAttr.encAttr.enType = pt;
    chnAttr.encAttr.bufSize = cfg->buf_size;
    chnAttr.encAttr.profile = (uint32_t)cfg->profile;
    chnAttr.encAttr.picWidth = cfg->width;
    chnAttr.encAttr.picHeight = cfg->height;

    /* Frame rate */
    chnAttr.rcAttr.outFrmRate.frmRateNum = cfg->fps_num;
    chnAttr.rcAttr.outFrmRate.frmRateDen = cfg->fps_den;

    /* GOP */
    chnAttr.rcAttr.maxGop = cfg->gop_length;

    /* Rate control mode */
    chnAttr.rcAttr.attrRcMode.rcMode = rc;

    switch (rc) {
    case ENC_RC_MODE_FIXQP:
        if (pt == PT_H264) {
            chnAttr.rcAttr.attrRcMode.attrH264FixQp.qp =
                (cfg->init_qp >= 0) ? (uint32_t)cfg->init_qp : 35;
        }
#if defined(PLATFORM_T21) || defined(PLATFORM_T23) || defined(PLATFORM_T30)
        else if (pt == PT_H265) {
            chnAttr.rcAttr.attrRcMode.attrH265FixQp.qp =
                (cfg->init_qp >= 0) ? (uint32_t)cfg->init_qp : 35;
        }
#endif
        break;

    case ENC_RC_MODE_CBR:
        if (pt == PT_H264) {
            chnAttr.rcAttr.attrRcMode.attrH264Cbr.outBitRate =
                cfg->bitrate / 1000; /* old SDK uses kbps */
            chnAttr.rcAttr.attrRcMode.attrH264Cbr.maxQp = (uint32_t)cfg->max_qp;
            chnAttr.rcAttr.attrRcMode.attrH264Cbr.minQp = (uint32_t)cfg->min_qp;
            chnAttr.rcAttr.attrRcMode.attrH264Cbr.frmQPStep = 3;
            chnAttr.rcAttr.attrRcMode.attrH264Cbr.gopQPStep = 15;
            chnAttr.rcAttr.attrRcMode.attrH264Cbr.adaptiveMode = false;
            chnAttr.rcAttr.attrRcMode.attrH264Cbr.gopRelation = false;
        }
#if defined(PLATFORM_T21) || defined(PLATFORM_T23) || defined(PLATFORM_T30)
        else if (pt == PT_H265) {
            chnAttr.rcAttr.attrRcMode.attrH265Cbr.outBitRate = cfg->bitrate / 1000;
            chnAttr.rcAttr.attrRcMode.attrH265Cbr.maxQp = (uint32_t)cfg->max_qp;
            chnAttr.rcAttr.attrRcMode.attrH265Cbr.minQp = (uint32_t)cfg->min_qp;
            chnAttr.rcAttr.attrRcMode.attrH265Cbr.frmQPStep = 3;
            chnAttr.rcAttr.attrRcMode.attrH265Cbr.gopQPStep = 15;
        }
#endif
        break;

    case ENC_RC_MODE_VBR:
        if (pt == PT_H264) {
            chnAttr.rcAttr.attrRcMode.attrH264Vbr.maxBitRate = cfg->max_bitrate / 1000;
            chnAttr.rcAttr.attrRcMode.attrH264Vbr.maxQp = (uint32_t)cfg->max_qp;
            chnAttr.rcAttr.attrRcMode.attrH264Vbr.minQp = (uint32_t)cfg->min_qp;
            chnAttr.rcAttr.attrRcMode.attrH264Vbr.staticTime = 1;
            chnAttr.rcAttr.attrRcMode.attrH264Vbr.changePos = 80;
            chnAttr.rcAttr.attrRcMode.attrH264Vbr.qualityLvl = 2;
            chnAttr.rcAttr.attrRcMode.attrH264Vbr.frmQPStep = 3;
            chnAttr.rcAttr.attrRcMode.attrH264Vbr.gopQPStep = 15;
            chnAttr.rcAttr.attrRcMode.attrH264Vbr.gopRelation = false;
        }
#if defined(PLATFORM_T21) || defined(PLATFORM_T23) || defined(PLATFORM_T30)
        else if (pt == PT_H265) {
            chnAttr.rcAttr.attrRcMode.attrH265Vbr.maxBitRate = cfg->max_bitrate / 1000;
            chnAttr.rcAttr.attrRcMode.attrH265Vbr.maxQp = (uint32_t)cfg->max_qp;
            chnAttr.rcAttr.attrRcMode.attrH265Vbr.minQp = (uint32_t)cfg->min_qp;
            chnAttr.rcAttr.attrRcMode.attrH265Vbr.staticTime = 1;
            chnAttr.rcAttr.attrRcMode.attrH265Vbr.changePos = 80;
            chnAttr.rcAttr.attrRcMode.attrH265Vbr.qualityLvl = 2;
            chnAttr.rcAttr.attrRcMode.attrH265Vbr.frmQPStep = 3;
            chnAttr.rcAttr.attrRcMode.attrH265Vbr.gopQPStep = 15;
        }
#endif
        break;

    case ENC_RC_MODE_SMART:
        if (pt == PT_H264) {
            chnAttr.rcAttr.attrRcMode.attrH264Smart.maxBitRate = cfg->max_bitrate / 1000;
            chnAttr.rcAttr.attrRcMode.attrH264Smart.maxQp = (uint32_t)cfg->max_qp;
            chnAttr.rcAttr.attrRcMode.attrH264Smart.minQp = (uint32_t)cfg->min_qp;
            chnAttr.rcAttr.attrRcMode.attrH264Smart.staticTime = 1;
            chnAttr.rcAttr.attrRcMode.attrH264Smart.changePos = 80;
            chnAttr.rcAttr.attrRcMode.attrH264Smart.qualityLvl = 2;
            chnAttr.rcAttr.attrRcMode.attrH264Smart.frmQPStep = 3;
            chnAttr.rcAttr.attrRcMode.attrH264Smart.gopQPStep = 15;
            chnAttr.rcAttr.attrRcMode.attrH264Smart.gopRelation = false;
        }
#if defined(PLATFORM_T21) || defined(PLATFORM_T23) || defined(PLATFORM_T30)
        else if (pt == PT_H265) {
            chnAttr.rcAttr.attrRcMode.attrH265Smart.maxBitRate = cfg->max_bitrate / 1000;
            chnAttr.rcAttr.attrRcMode.attrH265Smart.maxQp = (uint32_t)cfg->max_qp;
            chnAttr.rcAttr.attrRcMode.attrH265Smart.minQp = (uint32_t)cfg->min_qp;
            chnAttr.rcAttr.attrRcMode.attrH265Smart.staticTime = 1;
            chnAttr.rcAttr.attrRcMode.attrH265Smart.changePos = 80;
            chnAttr.rcAttr.attrRcMode.attrH265Smart.qualityLvl = 2;
            chnAttr.rcAttr.attrRcMode.attrH265Smart.frmQPStep = 3;
            chnAttr.rcAttr.attrRcMode.attrH265Smart.gopQPStep = 15;
        }
#endif
        break;

    default:
        break;
    }

#if defined(PLATFORM_T23)
    /* IVDC (ISP-VPU Direct Connect) — reduces rmem usage */
    chnAttr.bEnableIvdc = cfg->ivdc;
    if (cfg->ivdc)
        HAL_LOG_INFO("enc chn %d: IVDC enabled", chn);
#endif

    return IMP_Encoder_CreateChn(chn, &chnAttr);
}
#endif /* HAL_OLD_SDK */

#if defined(HAL_NEW_SDK)
/*
 * New SDK channel creation (T31, T32, T40, T41):
 *
 * Use IMP_Encoder_SetDefaultParam() to fill sensible defaults,
 * then override specific fields.
 *
 * Note: IMPEncoderCHNAttr is #defined to IMPEncoderChnAttr in
 * hal_internal.h for new SDK, so we use the old-style name
 * consistently.
 */
static int hal_enc_create_channel_new(int chn, const rss_video_config_t *cfg)
{
    IMPEncoderCHNAttr chnAttr;
    IMPEncoderProfile profile;
    IMPEncoderRcMode rc;
    int ret;
    int init_qp;

    memset(&chnAttr, 0, sizeof(chnAttr));

    profile = hal_translate_profile(cfg->codec, cfg->profile);
    rc = hal_translate_rc_mode(cfg->rc_mode);
    init_qp = (cfg->init_qp >= 0) ? cfg->init_qp : -1;

    /* JPEG: SetDefaultParam must be called with specific parameters.
     * fps=24, gop=0, quality as the QP param, bitrate=0.
     * CreateGroup is skipped for JPEG — it registers into the video group.
     * SetbufshareChn must be called BEFORE CreateChn. */
    if (cfg->codec == RSS_CODEC_JPEG || cfg->codec == RSS_CODEC_MJPEG) {
        int quality = (cfg->init_qp >= 0) ? cfg->init_qp : 25;
        ret = IMP_Encoder_SetDefaultParam(&chnAttr, profile, IMP_ENC_RC_MODE_FIXQP, cfg->width,
                                          cfg->height, 24, 1, /* fps_num=24, fps_den=1 */
                                          0,                  /* gop_length=0 */
                                          0,                  /* uMaxSameSenceCnt=0 */
                                          quality,            /* iInitialQP = JPEG quality */
                                          0                   /* uTargetBitRate=0 */
        );
        if (ret != 0) {
            HAL_LOG_ERR("SetDefaultParam (JPEG) failed: %d", ret);
            return ret;
        }
        return IMP_Encoder_CreateChn(chn, &chnAttr);
    }

    /*
     * SetDefaultParam signature:
     *   T31/T40/T41: (..., uMaxSameSenceCnt, iInitialQP, uTargetBitRate)
     *   T32:         (..., uBufSize, iInitialQP, uTargetBitRate)
     */
#if defined(PLATFORM_T32)
    ret = IMP_Encoder_SetDefaultParam(&chnAttr, profile, rc, cfg->width, cfg->height, cfg->fps_num,
                                      cfg->fps_den, cfg->gop_length,
                                      cfg->buf_size,               /* T32: uBufSize */
                                      init_qp, cfg->bitrate / 1000 /* SDK expects kbps */
    );
#else
    ret = IMP_Encoder_SetDefaultParam(&chnAttr, profile, rc, cfg->width, cfg->height, cfg->fps_num,
                                      cfg->fps_den, cfg->gop_length,
                                      2,                           /* uMaxSameSenceCnt: default 2 */
                                      init_qp, cfg->bitrate / 1000 /* SDK expects kbps */
    );
#endif
    if (ret != 0) {
        HAL_LOG_ERR("SetDefaultParam failed: %d", ret);
        return ret;
    }

    /* Override QP bounds if caller specified non-default values.
     * T32 uses old-style RC attr member names (attrH264Cbr etc.)
     * while T31/T40/T41 use new-style (attrCbr etc.). */
#if defined(PLATFORM_T32)
    /* T32: SetDefaultParam fills old-style RC attrs;
     * just override QP bounds in the H264 CBR/VBR structs.
     * CAPPED_VBR maps to CVBR, CAPPED_QUALITY maps to AVBR. */
    switch (rc) {
    case IMP_ENC_RC_MODE_CBR:
        if (cfg->min_qp >= 0)
            chnAttr.rcAttr.attrRcMode.attrH264Cbr.minQp = (uint32_t)cfg->min_qp;
        if (cfg->max_qp >= 0)
            chnAttr.rcAttr.attrRcMode.attrH264Cbr.maxQp = (uint32_t)cfg->max_qp;
        if (cfg->bitrate > 0)
            chnAttr.rcAttr.attrRcMode.attrH264Cbr.outBitRate = cfg->bitrate / 1000;
        break;
    case IMP_ENC_RC_MODE_VBR:
        if (cfg->min_qp >= 0)
            chnAttr.rcAttr.attrRcMode.attrH264Vbr.minQp = (uint32_t)cfg->min_qp;
        if (cfg->max_qp >= 0)
            chnAttr.rcAttr.attrRcMode.attrH264Vbr.maxQp = (uint32_t)cfg->max_qp;
        if (cfg->max_bitrate > 0)
            chnAttr.rcAttr.attrRcMode.attrH264Vbr.maxBitRate = cfg->max_bitrate / 1000;
        break;
    case ENC_RC_MODE_SMART:
        if (cfg->min_qp >= 0)
            chnAttr.rcAttr.attrRcMode.attrH264Smart.minQp = (uint8_t)cfg->min_qp;
        if (cfg->max_qp >= 0)
            chnAttr.rcAttr.attrRcMode.attrH264Smart.maxQp = (uint8_t)cfg->max_qp;
        if (cfg->max_bitrate > 0)
            chnAttr.rcAttr.attrRcMode.attrH264Smart.maxBitRate = cfg->max_bitrate / 1000;
        break;
    case ENC_RC_MODE_CVBR:
        if (cfg->min_qp >= 0)
            chnAttr.rcAttr.attrRcMode.attrH264CVbr.minQp = (uint8_t)cfg->min_qp;
        if (cfg->max_qp >= 0)
            chnAttr.rcAttr.attrRcMode.attrH264CVbr.maxQp = (uint8_t)cfg->max_qp;
        if (cfg->max_bitrate > 0)
            chnAttr.rcAttr.attrRcMode.attrH264CVbr.maxBitRate = cfg->max_bitrate / 1000;
        break;
    case ENC_RC_MODE_AVBR:
        if (cfg->min_qp >= 0)
            chnAttr.rcAttr.attrRcMode.attrH264AVbr.minQp = (uint8_t)cfg->min_qp;
        if (cfg->max_qp >= 0)
            chnAttr.rcAttr.attrRcMode.attrH264AVbr.maxQp = (uint8_t)cfg->max_qp;
        if (cfg->max_bitrate > 0)
            chnAttr.rcAttr.attrRcMode.attrH264AVbr.maxBitRate = cfg->max_bitrate / 1000;
        break;
    default:
        break;
    }
    /* T32 does not have gopAttr; GOP is set via SetDefaultParam's uGopLength arg */
#else
    switch (rc) {
    case IMP_ENC_RC_MODE_CBR:
        if (cfg->min_qp >= 0)
            chnAttr.rcAttr.attrRcMode.attrCbr.iMinQP = (int16_t)cfg->min_qp;
        if (cfg->max_qp >= 0)
            chnAttr.rcAttr.attrRcMode.attrCbr.iMaxQP = (int16_t)cfg->max_qp;
        if (cfg->bitrate > 0)
            chnAttr.rcAttr.attrRcMode.attrCbr.uTargetBitRate = cfg->bitrate / 1000;
        break;

    case IMP_ENC_RC_MODE_VBR:
        if (cfg->min_qp >= 0)
            chnAttr.rcAttr.attrRcMode.attrVbr.iMinQP = (int16_t)cfg->min_qp;
        if (cfg->max_qp >= 0)
            chnAttr.rcAttr.attrRcMode.attrVbr.iMaxQP = (int16_t)cfg->max_qp;
        if (cfg->bitrate > 0)
            chnAttr.rcAttr.attrRcMode.attrVbr.uTargetBitRate = cfg->bitrate / 1000;
        if (cfg->max_bitrate > 0)
            chnAttr.rcAttr.attrRcMode.attrVbr.uMaxBitRate = cfg->max_bitrate / 1000;
        break;

    case IMP_ENC_RC_MODE_CAPPED_VBR:
        if (cfg->min_qp >= 0)
            chnAttr.rcAttr.attrRcMode.attrCappedVbr.iMinQP = (int16_t)cfg->min_qp;
        if (cfg->max_qp >= 0)
            chnAttr.rcAttr.attrRcMode.attrCappedVbr.iMaxQP = (int16_t)cfg->max_qp;
        if (cfg->bitrate > 0)
            chnAttr.rcAttr.attrRcMode.attrCappedVbr.uTargetBitRate = cfg->bitrate / 1000;
        if (cfg->max_bitrate > 0)
            chnAttr.rcAttr.attrRcMode.attrCappedVbr.uMaxBitRate = cfg->max_bitrate / 1000;
        break;

    case IMP_ENC_RC_MODE_CAPPED_QUALITY:
        if (cfg->min_qp >= 0)
            chnAttr.rcAttr.attrRcMode.attrCappedQuality.iMinQP = (int16_t)cfg->min_qp;
        if (cfg->max_qp >= 0)
            chnAttr.rcAttr.attrRcMode.attrCappedQuality.iMaxQP = (int16_t)cfg->max_qp;
        if (cfg->bitrate > 0)
            chnAttr.rcAttr.attrRcMode.attrCappedQuality.uTargetBitRate = cfg->bitrate / 1000;
        if (cfg->max_bitrate > 0)
            chnAttr.rcAttr.attrRcMode.attrCappedQuality.uMaxBitRate = cfg->max_bitrate / 1000;
        break;

    default:
        break;
    }

    /* GOP length — don't override uGopCtrlMode, SetDefaultParam sets it correctly */
    chnAttr.gopAttr.uGopLength = (uint16_t)cfg->gop_length;
#endif

#if defined(PLATFORM_T32) || defined(PLATFORM_T41)
    /* IVDC (ISP-VPU Direct Connect) — reduces rmem usage */
    chnAttr.bEnableIvdc = cfg->ivdc;
    if (cfg->ivdc)
        HAL_LOG_INFO("enc chn %d: IVDC enabled", chn);
#endif

    return IMP_Encoder_CreateChn(chn, &chnAttr);
}
#endif /* HAL_NEW_SDK */

int hal_enc_create_channel(void *ctx, int chn, const rss_video_config_t *cfg)
{
    (void)ctx;

    if (!cfg)
        return -EINVAL;

    HAL_LOG_INFO("enc create chn %d: %ux%u codec=%d rc=%d bitrate=%u gop=%u", chn, cfg->width,
                 cfg->height, cfg->codec, cfg->rc_mode, cfg->bitrate, cfg->gop_length);

#if defined(HAL_OLD_SDK)
    return hal_enc_create_channel_old(chn, cfg);
#elif defined(HAL_NEW_SDK)
    return hal_enc_create_channel_new(chn, cfg);
#else
#error "Neither HAL_OLD_SDK nor HAL_NEW_SDK defined"
#endif
}

/* ══════════════════════════════════════════════════════════════════════
 * 3. Channel Destroy / Register / Unregister
 * ══════════════════════════════════════════════════════════════════════ */

int hal_enc_destroy_channel(void *ctx, int chn)
{
    (void)ctx;
    int ret = IMP_Encoder_DestroyChn(chn);
    if (ret != 0)
        HAL_LOG_ERR("IMP_Encoder_DestroyChn(%d) failed: %d", chn, ret);
    return ret;
}

int hal_enc_register_channel(void *ctx, int grp, int chn)
{
    (void)ctx;
    int ret = IMP_Encoder_RegisterChn(grp, chn);
    if (ret != 0)
        HAL_LOG_ERR("IMP_Encoder_RegisterChn(%d, %d) failed: %d", grp, chn, ret);
    return ret;
}

int hal_enc_unregister_channel(void *ctx, int chn)
{
    (void)ctx;
    int ret = IMP_Encoder_UnRegisterChn(chn);
    if (ret != 0)
        HAL_LOG_ERR("IMP_Encoder_UnRegisterChn(%d) failed: %d", chn, ret);
    return ret;
}

/* ══════════════════════════════════════════════════════════════════════
 * 4. Stream Control
 * ══════════════════════════════════════════════════════════════════════ */

int hal_enc_start(void *ctx, int chn)
{
    (void)ctx;
    int ret = IMP_Encoder_StartRecvPic(chn);
    if (ret != 0)
        HAL_LOG_ERR("IMP_Encoder_StartRecvPic(%d) failed: %d", chn, ret);
    return ret;
}

int hal_enc_stop(void *ctx, int chn)
{
    (void)ctx;
    int ret = IMP_Encoder_StopRecvPic(chn);
    if (ret != 0)
        HAL_LOG_ERR("IMP_Encoder_StopRecvPic(%d) failed: %d", chn, ret);
    return ret;
}

int hal_enc_poll(void *ctx, int chn, uint32_t timeout_ms)
{
    (void)ctx;
    return IMP_Encoder_PollingStream(chn, timeout_ms);
}

/* ══════════════════════════════════════════════════════════════════════
 * 5. Frame Access -- get_frame / release_frame
 * ══════════════════════════════════════════════════════════════════════ */

/*
 * Detect codec from the first NAL pack of a stream.
 * On new SDK, use IMPEncoderEncType extracted from channel attr;
 * on old SDK, use the payload type from the channel attr.
 *
 * For simplicity, we let the caller track codec per-channel.
 * Here we detect from the IMPEncoderStream itself using the NAL
 * type enum values: H264 NAL types are 0-12, H265 are 0-64 but
 * with VPS=32 distinguishing them.
 *
 * Actually: old SDK uses dataType.h264Type (always H264 enum on T20,
 * or h265Type on T21+), new SDK uses nalType union. We detect based
 * on the SPS/VPS/PPS presence by checking the value range.
 */

int hal_enc_get_frame(void *ctx, int chn, rss_frame_t *frame)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;
    IMPEncoderStream stream;
    int ret;
    uint32_t i;

    if (!c || !frame)
        return -EINVAL;

    if (chn < 0 || chn >= RSS_MAX_ENC_CHANNELS)
        return -EINVAL;

    memset(&stream, 0, sizeof(stream));

    /* Get one frame (blocking) */
    ret = IMP_Encoder_GetStream(chn, &stream, 1);
    if (ret != 0)
        return ret;

    /* Ensure NAL array is large enough */
    ret = hal_ensure_nal_array(c, chn, (int)stream.packCount);
    if (ret != 0) {
        IMP_Encoder_ReleaseStream(chn, &stream);
        return ret;
    }

    rss_nal_unit_t *nals = c->nal_arrays[chn];

    /* Detect codec from first pack's NAL type */
    rss_codec_t codec = RSS_CODEC_H264;
    bool is_key = false;

#if defined(HAL_OLD_SDK) || defined(PLATFORM_T32)
    /*
     * Old SDK (and T32 hybrid): each pack has its own virAddr/length.
     * NAL type is in pack[i].dataType.h264Type (T20) or
     * dataType.h265Type (T21/T23/T30/T32).
     *
     * Detect codec from first non-filler NAL: if any pack has
     * h264Type >= 32 it's really H265 accessed via h265Type.
     */
    {
        /* Peek at first pack to determine codec */
        IMPEncoderH264NaluType first_nal = stream.pack[0].dataType.h264Type;
#if !defined(PLATFORM_T20)
        if (first_nal == (IMPEncoderH264NaluType)IMP_H265_NAL_VPS ||
            first_nal == (IMPEncoderH264NaluType)IMP_H265_NAL_SPS ||
            first_nal == (IMPEncoderH264NaluType)IMP_H265_NAL_PPS || (int)first_nal >= 32) {
            codec = RSS_CODEC_H265;
        } else
#endif
            if (first_nal == IMP_H264_NAL_UNKNOWN && stream.packCount == 1) {
            /* Single-pack, unknown NAL -- likely JPEG */
            codec = RSS_CODEC_JPEG;
        }
    }

    for (i = 0; i < stream.packCount; i++) {
        nals[i].data = (const uint8_t *)(uintptr_t)stream.pack[i].virAddr;
        nals[i].length = stream.pack[i].length;
        nals[i].frame_end = stream.pack[i].frameEnd;

#if !defined(PLATFORM_T20)
        if (codec == RSS_CODEC_H265) {
            IMPEncoderH265NaluType h265nal =
                (IMPEncoderH265NaluType)stream.pack[i].dataType.h264Type;
            nals[i].type = hal_translate_nal_type_h265(h265nal);
            if (hal_is_idr_h265(h265nal))
                is_key = true;
        } else
#endif
            if (codec == RSS_CODEC_JPEG) {
            nals[i].type = RSS_NAL_JPEG_FRAME;
            is_key = true;
        } else {
            nals[i].type = hal_translate_nal_type_h264(stream.pack[i].dataType.h264Type);
            if (hal_is_idr_h264(stream.pack[i].dataType.h264Type))
                is_key = true;
        }
    }

#elif defined(HAL_NEW_SDK) /* T31/T40/T41 only, T32 handled above */
    /*
     * New SDK: packs have offset/length into ring buffer at
     * stream.virAddr with total size stream.streamSize.
     * Must handle wrap-around.
     */
    {
        /* Detect codec from first pack */
        IMPEncoderH264NaluType first_h264 = stream.pack[0].nalType.h264NalType;
        IMPEncoderH265NaluType first_h265 = stream.pack[0].nalType.h265NalType;

        if (first_h265 == IMP_H265_NAL_VPS || first_h265 == IMP_H265_NAL_SPS ||
            first_h265 == IMP_H265_NAL_PPS || (int)first_h264 >= 32) {
            codec = RSS_CODEC_H265;
        } else if (first_h264 == IMP_H264_NAL_UNKNOWN && stream.packCount == 1) {
            codec = RSS_CODEC_JPEG;
        }
    }

    uint8_t *vaddr = (uint8_t *)(uintptr_t)stream.virAddr;
    uint32_t stream_size = stream.streamSize;

    /* Calculate total frame data size for possible linearization */
    uint32_t total_size = 0;
    bool needs_linearize = false;
    for (i = 0; i < stream.packCount; i++) {
        total_size += stream.pack[i].length;
        if (stream.pack[i].offset + stream.pack[i].length > stream_size)
            needs_linearize = true;
    }

    /* Allocate scratch buffer only if wrapping detected */
    uint8_t *scratch = NULL;
    if (needs_linearize) {
        ret = hal_ensure_scratch(c, total_size);
        if (ret != 0) {
            IMP_Encoder_ReleaseStream(chn, &stream);
            return ret;
        }
        scratch = c->scratch_buf;
    }

    for (i = 0; i < stream.packCount; i++) {
        uint32_t offset = stream.pack[i].offset;
        uint32_t len = stream.pack[i].length;

        if (offset + len <= stream_size) {
            /* Contiguous -- point directly into ring buffer */
            nals[i].data = vaddr + offset;
        } else {
            /* Ring buffer wrap -- linearize into scratch */
            uint32_t first = stream_size - offset;
            uint32_t second = len - first;
            memcpy(scratch, vaddr + offset, first);
            memcpy(scratch + first, vaddr, second);
            nals[i].data = scratch;
            scratch += len;
        }

        nals[i].length = len;
        nals[i].frame_end = stream.pack[i].frameEnd;

        if (codec == RSS_CODEC_H265) {
            IMPEncoderH265NaluType h265nal = stream.pack[i].nalType.h265NalType;
            nals[i].type = hal_translate_nal_type_h265(h265nal);
            if (hal_is_idr_h265(h265nal))
                is_key = true;
        } else if (codec == RSS_CODEC_JPEG) {
            nals[i].type = RSS_NAL_JPEG_FRAME;
            is_key = true;
        } else {
            IMPEncoderH264NaluType h264nal = stream.pack[i].nalType.h264NalType;
            nals[i].type = hal_translate_nal_type_h264(h264nal);
            if (hal_is_idr_h264(h264nal))
                is_key = true;
        }
    }
#endif

    /* Fill the public frame struct */
    frame->nals = nals;
    frame->nal_count = stream.packCount;
    frame->codec = codec;
    frame->timestamp = (stream.packCount > 0) ? stream.pack[0].timestamp : 0;
    frame->seq = stream.seq;
    frame->is_key = is_key;

    /*
     * Store the vendor stream struct in _priv for release.
     * We must heap-allocate because the stack copy goes away.
     */
    IMPEncoderStream *priv = (IMPEncoderStream *)malloc(sizeof(IMPEncoderStream));
    if (!priv) {
        IMP_Encoder_ReleaseStream(chn, &stream);
        return -ENOMEM;
    }
    memcpy(priv, &stream, sizeof(stream));
    frame->_priv = priv;

    return 0;
}

int hal_enc_release_frame(void *ctx, int chn, rss_frame_t *frame)
{
    (void)ctx;

    if (!frame || !frame->_priv)
        return -EINVAL;

    IMPEncoderStream *stream = (IMPEncoderStream *)frame->_priv;
    int ret = IMP_Encoder_ReleaseStream(chn, stream);

    free(stream);
    frame->_priv = NULL;
    frame->nals = NULL;
    frame->nal_count = 0;

    if (ret != 0)
        HAL_LOG_ERR("IMP_Encoder_ReleaseStream(%d) failed: %d", chn, ret);

    return ret;
}

/* ══════════════════════════════════════════════════════════════════════
 * 6. IDR Request
 * ══════════════════════════════════════════════════════════════════════ */

int hal_enc_request_idr(void *ctx, int chn)
{
    (void)ctx;
    int ret = IMP_Encoder_RequestIDR(chn);
    if (ret != 0)
        HAL_LOG_ERR("IMP_Encoder_RequestIDR(%d) failed: %d", chn, ret);
    return ret;
}

/* ══════════════════════════════════════════════════════════════════════
 * 7. Dynamic Parameter Updates
 * ══════════════════════════════════════════════════════════════════════ */

/*
 * hal_enc_set_rc_mode -- switch rate control mode at runtime.
 *
 * All platforms: IMP_Encoder_SetChnAttrRcMode(chn, &rcMode)
 * Takes effect at next IDR frame.
 */
int hal_enc_set_rc_mode(void *ctx, int chn, rss_rc_mode_t mode, uint32_t bitrate)
{
    (void)ctx;

    IMPEncoderRcMode vendor_mode = hal_translate_rc_mode(mode);
    uint32_t bitrate_kbps = bitrate / 1000;
    if (bitrate_kbps == 0)
        bitrate_kbps = 2000; /* fallback */

    /* Read current RC attrs to preserve QP bounds, deltas, step sizes.
     * Only change the mode and bitrate fields. */
    IMPEncoderAttrRcMode rcAttr;
    int ret = IMP_Encoder_GetChnAttrRcMode(chn, &rcAttr);
    if (ret != 0) {
        HAL_LOG_ERR("GetChnAttrRcMode(%d) failed: %d", chn, ret);
        return ret;
    }
    rcAttr.rcMode = vendor_mode;

#if defined(HAL_NEW_SDK) && !defined(PLATFORM_T32)
    /* New SDK (T31/T40/T41): patch bitrate in the target mode struct */
    switch (vendor_mode) {
    case IMP_ENC_RC_MODE_FIXQP:
        if (rcAttr.attrFixQp.iInitialQP < 1)
            rcAttr.attrFixQp.iInitialQP = 35;
        break;
    case IMP_ENC_RC_MODE_CBR:
        rcAttr.attrCbr.uTargetBitRate = bitrate_kbps;
        if (rcAttr.attrCbr.iMaxQP == 0) rcAttr.attrCbr.iMaxQP = 45;
        if (rcAttr.attrCbr.iMinQP == 0) rcAttr.attrCbr.iMinQP = 15;
        break;
    case IMP_ENC_RC_MODE_VBR:
        rcAttr.attrVbr.uTargetBitRate = bitrate_kbps;
        rcAttr.attrVbr.uMaxBitRate = bitrate_kbps * 4 / 3;
        if (rcAttr.attrVbr.iMaxQP == 0) rcAttr.attrVbr.iMaxQP = 45;
        if (rcAttr.attrVbr.iMinQP == 0) rcAttr.attrVbr.iMinQP = 15;
        break;
    case IMP_ENC_RC_MODE_CAPPED_VBR:
        rcAttr.attrCappedVbr.uTargetBitRate = bitrate_kbps;
        rcAttr.attrCappedVbr.uMaxBitRate = bitrate_kbps * 4 / 3;
        if (rcAttr.attrCappedVbr.iMaxQP == 0) rcAttr.attrCappedVbr.iMaxQP = 45;
        if (rcAttr.attrCappedVbr.iMinQP == 0) rcAttr.attrCappedVbr.iMinQP = 15;
        break;
    case IMP_ENC_RC_MODE_CAPPED_QUALITY:
        rcAttr.attrCappedQuality.uTargetBitRate = bitrate_kbps;
        rcAttr.attrCappedQuality.uMaxBitRate = bitrate_kbps * 4 / 3;
        if (rcAttr.attrCappedQuality.iMaxQP == 0) rcAttr.attrCappedQuality.iMaxQP = 45;
        if (rcAttr.attrCappedQuality.iMinQP == 0) rcAttr.attrCappedQuality.iMinQP = 15;
        break;
    default:
        break;
    }
#else
    /* Old SDK (T20-T23) and T32: patch bitrate in H264-prefixed structs */
    switch (vendor_mode) {
    case ENC_RC_MODE_FIXQP:
        if (rcAttr.attrH264FixQp.qp == 0) rcAttr.attrH264FixQp.qp = 35;
        break;
    case ENC_RC_MODE_CBR:
        rcAttr.attrH264Cbr.outBitRate = bitrate_kbps;
        if (rcAttr.attrH264Cbr.maxQp == 0) rcAttr.attrH264Cbr.maxQp = 45;
        if (rcAttr.attrH264Cbr.minQp == 0) rcAttr.attrH264Cbr.minQp = 15;
        if (rcAttr.attrH264Cbr.frmQPStep == 0) rcAttr.attrH264Cbr.frmQPStep = 3;
        if (rcAttr.attrH264Cbr.gopQPStep == 0) rcAttr.attrH264Cbr.gopQPStep = 15;
        break;
    case ENC_RC_MODE_VBR:
        rcAttr.attrH264Vbr.maxBitRate = bitrate_kbps;
        if (rcAttr.attrH264Vbr.maxQp == 0) rcAttr.attrH264Vbr.maxQp = 45;
        if (rcAttr.attrH264Vbr.minQp == 0) rcAttr.attrH264Vbr.minQp = 15;
        if (rcAttr.attrH264Vbr.frmQPStep == 0) rcAttr.attrH264Vbr.frmQPStep = 3;
        if (rcAttr.attrH264Vbr.gopQPStep == 0) rcAttr.attrH264Vbr.gopQPStep = 15;
        break;
    case ENC_RC_MODE_SMART:
        rcAttr.attrH264Smart.maxBitRate = bitrate_kbps;
        if (rcAttr.attrH264Smart.maxQp == 0) rcAttr.attrH264Smart.maxQp = 45;
        if (rcAttr.attrH264Smart.minQp == 0) rcAttr.attrH264Smart.minQp = 15;
        if (rcAttr.attrH264Smart.frmQPStep == 0) rcAttr.attrH264Smart.frmQPStep = 3;
        if (rcAttr.attrH264Smart.gopQPStep == 0) rcAttr.attrH264Smart.gopQPStep = 15;
        break;
    default:
        break;
    }
#endif

    ret = IMP_Encoder_SetChnAttrRcMode(chn, &rcAttr);
    if (ret != 0) {
        HAL_LOG_ERR("SetChnAttrRcMode(%d, mode=%d) failed: %d", chn, (int)vendor_mode, ret);
        return ret;
    }
    IMP_Encoder_RequestIDR(chn);
    HAL_LOG_INFO("encoder chn %d: rc_mode -> %d, bitrate %u kbps", chn, (int)mode, bitrate_kbps);
    return 0;
}

/*
 * hal_enc_set_bitrate -- change bitrate dynamically.
 *
 * New SDK: IMP_Encoder_SetChnBitRate(chn, target, max)
 * Old SDK: must rebuild the full RC mode attr and call SetChnAttrRcMode.
 *          As a simplified fallback, we get the current RC attr, patch
 *          the bitrate field, and set it back.
 */
int hal_enc_set_bitrate(void *ctx, int chn, uint32_t bitrate)
{
    (void)ctx;
    int ret;

#if defined(HAL_NEW_SDK)
    ret = IMP_Encoder_SetChnBitRate(chn, (int)bitrate, (int)(bitrate * 4 / 3));
    if (ret != 0)
        HAL_LOG_ERR("SetChnBitRate(%d, %u) failed: %d", chn, bitrate, ret);
    return ret;
#else
    /* Old SDK: get current RC mode, patch bitrate, set it back */
    IMPEncoderAttrRcMode rcMode;
    ret = IMP_Encoder_GetChnAttrRcMode(chn, &rcMode);
    if (ret != 0) {
        HAL_LOG_ERR("GetChnAttrRcMode(%d) failed: %d", chn, ret);
        return ret;
    }

    uint32_t bitrate_kbps = bitrate / 1000;

    switch (rcMode.rcMode) {
    case ENC_RC_MODE_CBR:
        rcMode.attrH264Cbr.outBitRate = bitrate_kbps;
        break;
    case ENC_RC_MODE_VBR:
        rcMode.attrH264Vbr.maxBitRate = bitrate_kbps;
        break;
    case ENC_RC_MODE_SMART:
        rcMode.attrH264Smart.maxBitRate = bitrate_kbps;
        break;
    default:
        /* FixQP has no bitrate concept */
        return 0;
    }

    ret = IMP_Encoder_SetChnAttrRcMode(chn, &rcMode);
    if (ret != 0) {
        HAL_LOG_ERR("SetChnAttrRcMode(%d) failed: %d", chn, ret);
        return ret;
    }
    IMP_Encoder_RequestIDR(chn);
    return 0;
#endif
}

/*
 * hal_enc_set_gop -- change GOP length dynamically.
 *
 * New SDK (T31/T40/T41): IMP_Encoder_SetChnGopLength(chn, gop)
 * New SDK (T32): has SetGOPSize (old-style name)
 * Old SDK: IMP_Encoder_SetGOPSize(chn, &cfg)
 */
int hal_enc_set_gop(void *ctx, int chn, uint32_t gop_length)
{
    (void)ctx;
    int ret;

#if defined(HAL_NEW_SDK)
#if defined(PLATFORM_T32)
    /* T32 uses the old-style SetGOPSize function */
    IMPEncoderGOPSizeCfg gopCfg;
    gopCfg.gopsize = (int)gop_length;
    ret = IMP_Encoder_SetGOPSize(chn, &gopCfg);
#else
    ret = IMP_Encoder_SetChnGopLength(chn, (int)gop_length);
#endif
    if (ret != 0)
        HAL_LOG_ERR("SetGop(%d, %u) failed: %d", chn, gop_length, ret);
    return ret;
#else
    /* Old SDK: SetGOPSize with IMPEncoderGOPSizeCfg */
    IMPEncoderGOPSizeCfg gopCfg;
    gopCfg.gopsize = (int)gop_length;
    ret = IMP_Encoder_SetGOPSize(chn, &gopCfg);
    if (ret != 0)
        HAL_LOG_ERR("SetGOPSize(%d, %u) failed: %d", chn, gop_length, ret);
    return ret;
#endif
}

/*
 * hal_enc_set_fps -- change encoder frame rate dynamically.
 *
 * Same API on all SoCs: IMP_Encoder_SetChnFrmRate(chn, &frmRate).
 */
int hal_enc_set_fps(void *ctx, int chn, uint32_t fps_num, uint32_t fps_den)
{
    (void)ctx;
    IMPEncoderFrmRate frmRate;

    frmRate.frmRateNum = fps_num;
    frmRate.frmRateDen = fps_den;

    int ret = IMP_Encoder_SetChnFrmRate(chn, &frmRate);
    if (ret != 0)
        HAL_LOG_ERR("SetChnFrmRate(%d, %u/%u) failed: %d", chn, fps_num, fps_den, ret);
    return ret;
}

/* ══════════════════════════════════════════════════════════════════════
 * 8. Buffer Sharing
 * ══════════════════════════════════════════════════════════════════════ */

/*
 * hal_enc_set_bufshare -- JPEG shares H264/H265 buffer.
 *
 * New SDK (T31/T40/T41): IMP_Encoder_SetbufshareChn(src, dst)
 * T32: no SetbufshareChn, return success (no-op).
 * Old SDK: not supported, return success (no-op).
 */
int hal_enc_set_bufshare(void *ctx, int src_chn, int dst_chn)
{
    (void)ctx;

#if defined(HAL_NEW_SDK) && !defined(PLATFORM_T32)
    int ret = IMP_Encoder_SetbufshareChn(src_chn, dst_chn);
    if (ret != 0)
        HAL_LOG_ERR("SetbufshareChn(%d, %d) failed: %d", src_chn, dst_chn, ret);
    return ret;
#else
    /* Not available on old SDK or T32 -- no-op success */
    (void)src_chn;
    (void)dst_chn;
    return 0;
#endif
}

/* ======================================================================
 * 9. Encoder Getters and Additional Setters
 * ====================================================================== */

/*
 * hal_enc_get_channel_attr -- retrieve channel attributes into rss_video_config_t.
 *
 * Reverse-translates vendor IMPEncoderCHNAttr back to RSS types.
 */
int hal_enc_get_channel_attr(void *ctx, int chn, rss_video_config_t *cfg)
{
    (void)ctx;
    int ret;

    if (!cfg)
        return RSS_ERR_INVAL;

    IMPEncoderCHNAttr chnAttr;
    memset(&chnAttr, 0, sizeof(chnAttr));
    ret = IMP_Encoder_GetChnAttr(chn, &chnAttr);
    if (ret != 0) {
        HAL_LOG_ERR("IMP_Encoder_GetChnAttr(%d) failed: %d", chn, ret);
        return ret;
    }

    memset(cfg, 0, sizeof(*cfg));

#if defined(HAL_NEW_SDK) && !defined(PLATFORM_T32)
    /* New SDK (T31/T40/T41): unified struct with encAttr containing profile enum */
    cfg->width = chnAttr.encAttr.uWidth;
    cfg->height = chnAttr.encAttr.uHeight;

    /* Reverse-translate profile to codec */
    switch (chnAttr.encAttr.eProfile) {
    case IMP_ENC_PROFILE_AVC_BASELINE:
    case IMP_ENC_PROFILE_AVC_MAIN:
    case IMP_ENC_PROFILE_AVC_HIGH:
        cfg->codec = RSS_CODEC_H264;
        if (chnAttr.encAttr.eProfile == IMP_ENC_PROFILE_AVC_BASELINE)
            cfg->profile = 0;
        else if (chnAttr.encAttr.eProfile == IMP_ENC_PROFILE_AVC_MAIN)
            cfg->profile = 1;
        else
            cfg->profile = 2;
        break;
    case IMP_ENC_PROFILE_HEVC_MAIN:
        cfg->codec = RSS_CODEC_H265;
        cfg->profile = 0;
        break;
    case IMP_ENC_PROFILE_JPEG:
        cfg->codec = RSS_CODEC_JPEG;
        cfg->profile = 0;
        break;
    default:
        cfg->codec = RSS_CODEC_H264;
        cfg->profile = 2;
        break;
    }

    /* Frame rate */
    cfg->fps_num = chnAttr.rcAttr.outFrmRate.frmRateNum;
    cfg->fps_den = chnAttr.rcAttr.outFrmRate.frmRateDen;

    /* GOP */
    cfg->gop_length = chnAttr.gopAttr.uGopLength;

    /* RC mode and bitrate */
    switch (chnAttr.rcAttr.attrRcMode.rcMode) {
    case IMP_ENC_RC_MODE_FIXQP:
        cfg->rc_mode = RSS_RC_FIXQP;
        break;
    case IMP_ENC_RC_MODE_CBR:
        cfg->rc_mode = RSS_RC_CBR;
        cfg->bitrate = chnAttr.rcAttr.attrRcMode.attrCbr.uTargetBitRate;
        cfg->min_qp = chnAttr.rcAttr.attrRcMode.attrCbr.iMinQP;
        cfg->max_qp = chnAttr.rcAttr.attrRcMode.attrCbr.iMaxQP;
        break;
    case IMP_ENC_RC_MODE_VBR:
        cfg->rc_mode = RSS_RC_VBR;
        cfg->bitrate = chnAttr.rcAttr.attrRcMode.attrVbr.uTargetBitRate;
        cfg->max_bitrate = chnAttr.rcAttr.attrRcMode.attrVbr.uMaxBitRate;
        cfg->min_qp = chnAttr.rcAttr.attrRcMode.attrVbr.iMinQP;
        cfg->max_qp = chnAttr.rcAttr.attrRcMode.attrVbr.iMaxQP;
        break;
    case IMP_ENC_RC_MODE_CAPPED_VBR:
        cfg->rc_mode = RSS_RC_CAPPED_VBR;
        cfg->bitrate = chnAttr.rcAttr.attrRcMode.attrCappedVbr.uTargetBitRate;
        cfg->max_bitrate = chnAttr.rcAttr.attrRcMode.attrCappedVbr.uMaxBitRate;
        cfg->min_qp = chnAttr.rcAttr.attrRcMode.attrCappedVbr.iMinQP;
        cfg->max_qp = chnAttr.rcAttr.attrRcMode.attrCappedVbr.iMaxQP;
        break;
    case IMP_ENC_RC_MODE_CAPPED_QUALITY:
        cfg->rc_mode = RSS_RC_CAPPED_QUALITY;
        cfg->bitrate = chnAttr.rcAttr.attrRcMode.attrCappedQuality.uTargetBitRate;
        cfg->max_bitrate = chnAttr.rcAttr.attrRcMode.attrCappedQuality.uMaxBitRate;
        cfg->min_qp = chnAttr.rcAttr.attrRcMode.attrCappedQuality.iMinQP;
        cfg->max_qp = chnAttr.rcAttr.attrRcMode.attrCappedQuality.iMaxQP;
        break;
    default:
        cfg->rc_mode = RSS_RC_CBR;
        break;
    }
#else
    /* Old SDK (T20/T21/T23/T30) and T32 (hybrid): per-codec struct layout */
    /* Old SDK: per-codec struct layout */
    cfg->width = chnAttr.encAttr.picWidth;
    cfg->height = chnAttr.encAttr.picHeight;
    cfg->profile = (int)chnAttr.encAttr.profile;
    cfg->buf_size = chnAttr.encAttr.bufSize;

    switch (chnAttr.encAttr.enType) {
    case PT_H264:
        cfg->codec = RSS_CODEC_H264;
        break;
#if !defined(PLATFORM_T20)
    case PT_H265:
        cfg->codec = RSS_CODEC_H265;
        break;
#endif
    case PT_JPEG:
        cfg->codec = RSS_CODEC_JPEG;
        break;
    default:
        cfg->codec = RSS_CODEC_H264;
        break;
    }

    cfg->fps_num = chnAttr.rcAttr.outFrmRate.frmRateNum;
    cfg->fps_den = chnAttr.rcAttr.outFrmRate.frmRateDen;
    cfg->gop_length = chnAttr.rcAttr.maxGop;

    switch (chnAttr.rcAttr.attrRcMode.rcMode) {
    case ENC_RC_MODE_FIXQP:
        cfg->rc_mode = RSS_RC_FIXQP;
        break;
    case ENC_RC_MODE_CBR:
        cfg->rc_mode = RSS_RC_CBR;
        cfg->bitrate = chnAttr.rcAttr.attrRcMode.attrH264Cbr.outBitRate * 1000;
        cfg->min_qp = (int16_t)chnAttr.rcAttr.attrRcMode.attrH264Cbr.minQp;
        cfg->max_qp = (int16_t)chnAttr.rcAttr.attrRcMode.attrH264Cbr.maxQp;
        break;
    case ENC_RC_MODE_VBR:
        cfg->rc_mode = RSS_RC_VBR;
        cfg->max_bitrate = chnAttr.rcAttr.attrRcMode.attrH264Vbr.maxBitRate * 1000;
        cfg->min_qp = (int16_t)chnAttr.rcAttr.attrRcMode.attrH264Vbr.minQp;
        cfg->max_qp = (int16_t)chnAttr.rcAttr.attrRcMode.attrH264Vbr.maxQp;
        break;
    case ENC_RC_MODE_SMART:
        cfg->rc_mode = RSS_RC_SMART;
        cfg->max_bitrate = chnAttr.rcAttr.attrRcMode.attrH264Smart.maxBitRate * 1000;
        cfg->min_qp = (int16_t)chnAttr.rcAttr.attrRcMode.attrH264Smart.minQp;
        cfg->max_qp = (int16_t)chnAttr.rcAttr.attrRcMode.attrH264Smart.maxQp;
        break;
    default:
        cfg->rc_mode = RSS_RC_CBR;
        break;
    }
#endif

    return RSS_OK;
}

/*
 * hal_enc_get_fps -- retrieve current encoder frame rate.
 */
int hal_enc_get_fps(void *ctx, int chn, uint32_t *fps_num, uint32_t *fps_den)
{
    (void)ctx;
    int ret;

    if (!fps_num || !fps_den)
        return RSS_ERR_INVAL;

    IMPEncoderFrmRate frmRate;
    ret = IMP_Encoder_GetChnFrmRate(chn, &frmRate);
    if (ret != 0) {
        HAL_LOG_ERR("IMP_Encoder_GetChnFrmRate(%d) failed: %d", chn, ret);
        return ret;
    }

    *fps_num = frmRate.frmRateNum;
    *fps_den = frmRate.frmRateDen;
    return RSS_OK;
}

/*
 * hal_enc_get_gop_attr -- retrieve current GOP length.
 */
int hal_enc_get_gop_attr(void *ctx, int chn, uint32_t *gop_length)
{
    (void)ctx;
    int ret;

    if (!gop_length)
        return RSS_ERR_INVAL;

    IMPEncoderCHNAttr chnAttr;
    memset(&chnAttr, 0, sizeof(chnAttr));
    ret = IMP_Encoder_GetChnAttr(chn, &chnAttr);
    if (ret != 0) {
        HAL_LOG_ERR("IMP_Encoder_GetChnAttr(%d) for GOP failed: %d", chn, ret);
        return ret;
    }

#if defined(HAL_NEW_SDK) && !defined(PLATFORM_T32)
    *gop_length = chnAttr.gopAttr.uGopLength;
#else
    *gop_length = chnAttr.rcAttr.maxGop;
#endif
    return RSS_OK;
}

/*
 * hal_enc_set_gop_attr -- set GOP length (alias with different name from set_gop).
 */
int hal_enc_set_gop_attr(void *ctx, int chn, uint32_t gop_length)
{
    return hal_enc_set_gop(ctx, chn, gop_length);
}

/*
 * hal_enc_get_avg_bitrate -- retrieve average bitrate from channel stat.
 */
int hal_enc_get_avg_bitrate(void *ctx, int chn, uint32_t *bitrate)
{
    (void)ctx;
    int ret;

    if (!bitrate)
        return RSS_ERR_INVAL;

    IMPEncoderCHNStat stat;
    memset(&stat, 0, sizeof(stat));
    ret = IMP_Encoder_Query(chn, &stat);
    if (ret != 0) {
        HAL_LOG_ERR("IMP_Encoder_Query(%d) failed: %d", chn, ret);
        return ret;
    }

    *bitrate = stat.curPacks; /* SDK reports current bitrate in curPacks field */
    return RSS_OK;
}

/*
 * hal_enc_flush_stream -- flush encoder output buffer.
 */
int hal_enc_flush_stream(void *ctx, int chn)
{
    (void)ctx;
    int ret = IMP_Encoder_FlushStream(chn);
    if (ret != 0)
        HAL_LOG_ERR("IMP_Encoder_FlushStream(%d) failed: %d", chn, ret);
    return ret;
}

/*
 * hal_enc_query -- check if the encoder channel is busy.
 */
int hal_enc_query(void *ctx, int chn, bool *busy)
{
    (void)ctx;
    int ret;

    if (!busy)
        return RSS_ERR_INVAL;

    IMPEncoderCHNStat stat;
    memset(&stat, 0, sizeof(stat));
    ret = IMP_Encoder_Query(chn, &stat);
    if (ret != 0) {
        HAL_LOG_ERR("IMP_Encoder_Query(%d) failed: %d", chn, ret);
        return ret;
    }

    *busy = (stat.registered != 0);
    return RSS_OK;
}

/*
 * hal_enc_get_fd -- get the file descriptor for polling the encoder channel.
 */
int hal_enc_get_fd(void *ctx, int chn)
{
    (void)ctx;
#if defined(PLATFORM_T20)
    (void)chn;
    return RSS_ERR_NOTSUP;
#else
    return IMP_Encoder_GetFd(chn);
#endif
}

/*
 * hal_enc_set_qp -- set QP value for the encoder channel.
 *
 * New SDK: IMP_Encoder_SetChnQp
 * Old SDK: not supported
 */
int hal_enc_set_qp(void *ctx, int chn, int qp)
{
    (void)ctx;
#if defined(PLATFORM_T31)
    return IMP_Encoder_SetChnQp(chn, qp);
#else
    (void)chn;
    (void)qp;
    return RSS_ERR_NOTSUP;
#endif
}

/*
 * hal_enc_set_qp_bounds -- set min/max QP for the encoder channel.
 *
 * New SDK: modify RC mode attrs inline.
 * Old SDK: modify the per-codec RC mode attrs.
 */
int hal_enc_set_qp_bounds(void *ctx, int chn, int min_qp, int max_qp)
{
    (void)ctx;
    int ret;

#if defined(HAL_NEW_SDK)
    ret = IMP_Encoder_SetChnQpBounds(chn, min_qp, max_qp);
    if (ret != 0)
        HAL_LOG_ERR("SetChnQpBounds(%d) failed: %d", chn, ret);
    return ret;
#else
    IMPEncoderAttrRcMode rcMode;
    ret = IMP_Encoder_GetChnAttrRcMode(chn, &rcMode);
    if (ret != 0) {
        HAL_LOG_ERR("GetChnAttrRcMode(%d) failed: %d", chn, ret);
        return ret;
    }

    switch (rcMode.rcMode) {
    case ENC_RC_MODE_CBR:
        rcMode.attrH264Cbr.minQp = (uint32_t)min_qp;
        rcMode.attrH264Cbr.maxQp = (uint32_t)max_qp;
        break;
    case ENC_RC_MODE_VBR:
        rcMode.attrH264Vbr.minQp = (uint32_t)min_qp;
        rcMode.attrH264Vbr.maxQp = (uint32_t)max_qp;
        break;
    case ENC_RC_MODE_SMART:
        rcMode.attrH264Smart.minQp = (uint32_t)min_qp;
        rcMode.attrH264Smart.maxQp = (uint32_t)max_qp;
        break;
    default:
        return RSS_ERR_NOTSUP;
    }

    ret = IMP_Encoder_SetChnAttrRcMode(chn, &rcMode);
    if (ret != 0)
        HAL_LOG_ERR("SetChnAttrRcMode(%d) QP bounds failed: %d", chn, ret);
    return ret;
#endif
}

/*
 * hal_enc_set_qp_ip_delta -- set QP delta between I and P frames.
 *
 * T31 only: uses IMP_Encoder_SetChnQpIPDelta() direct API.
 */
int hal_enc_set_qp_ip_delta(void *ctx, int chn, int delta)
{
    (void)ctx;
#if defined(PLATFORM_T31)
    return IMP_Encoder_SetChnQpIPDelta(chn, delta);
#else
    (void)chn;
    (void)delta;
    return RSS_ERR_NOTSUP;
#endif
}

/*
 * hal_enc_set_stream_buf_size -- set encoder stream buffer size.
 *
 * New SDK: re-create with different buf size, or use SetStreamBufSize if available.
 * Old SDK: set via encAttr.bufSize before channel creation; after creation not supported.
 */
int hal_enc_set_stream_buf_size(void *ctx, int chn, uint32_t size)
{
    (void)ctx;
#if defined(PLATFORM_T31) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
    return IMP_Encoder_SetStreamBufSize(chn, size);
#else
    (void)chn;
    (void)size;
    return RSS_ERR_NOTSUP;
#endif
}

/*
 * hal_enc_get_stream_buf_size -- get encoder stream buffer size.
 */
int hal_enc_get_stream_buf_size(void *ctx, int chn, uint32_t *size)
{
    (void)ctx;

    if (!size)
        return RSS_ERR_INVAL;

#if defined(PLATFORM_T31) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
    return IMP_Encoder_GetStreamBufSize(chn, size);
#else
    int ret;
    IMPEncoderCHNAttr chnAttr;
    memset(&chnAttr, 0, sizeof(chnAttr));
    ret = IMP_Encoder_GetChnAttr(chn, &chnAttr);
    if (ret != 0) {
        HAL_LOG_ERR("GetChnAttr(%d) for buf size failed: %d", chn, ret);
        return ret;
    }

    *size = chnAttr.encAttr.bufSize;
    return RSS_OK;
#endif
}

/* ======================================================================
 * 10. Additional Encoder Functions
 * ====================================================================== */

/*
 * hal_enc_get_chn_gop_attr -- get GOP attributes (vendor struct).
 *
 * IMP_Encoder_GetChnGopAttr(chn, IMPEncoderGopAttr *)
 * T31+T40+T41 only (new SDK, not T32).
 */
int hal_enc_get_chn_gop_attr(void *ctx, int chn, void *gop_attr)
{
    (void)ctx;
    if (!gop_attr)
        return RSS_ERR_INVAL;
#if defined(PLATFORM_T31) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
    return IMP_Encoder_GetChnGopAttr(chn, (IMPEncoderGopAttr *)gop_attr);
#else
    (void)chn;
    return RSS_ERR_NOTSUP;
#endif
}

/*
 * hal_enc_set_chn_gop_attr -- set GOP attributes (vendor struct).
 *
 * IMP_Encoder_SetChnGopAttr(chn, const IMPEncoderGopAttr *)
 * T31+T40+T41 only.
 */
int hal_enc_set_chn_gop_attr(void *ctx, int chn, const void *gop_attr)
{
    (void)ctx;
    if (!gop_attr)
        return RSS_ERR_INVAL;
#if defined(PLATFORM_T31) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
    return IMP_Encoder_SetChnGopAttr(chn, (const IMPEncoderGopAttr *)gop_attr);
#else
    (void)chn;
    return RSS_ERR_NOTSUP;
#endif
}

/*
 * hal_enc_get_chn_enc_type -- get encoding protocol type.
 *
 * IMP_Encoder_GetChnEncType(chn, IMPEncoderEncType *)
 * All except T20 (new SDK has it; T21+T23+T30 have it in old SDK too).
 */
int hal_enc_get_chn_enc_type(void *ctx, int chn, void *enc_type)
{
    (void)ctx;
    if (!enc_type)
        return RSS_ERR_INVAL;
#if defined(PLATFORM_T20)
    (void)chn;
    return RSS_ERR_NOTSUP;
#elif defined(HAL_NEW_SDK) && !defined(PLATFORM_T32)
    return IMP_Encoder_GetChnEncType(chn, (IMPEncoderEncType *)enc_type);
#else
    /* Old SDK (T21/T23/T30) and T32: output type is IMPPayloadType */
    return IMP_Encoder_GetChnEncType(chn, (IMPPayloadType *)enc_type);
#endif
}

/*
 * hal_enc_get_chn_ave_bitrate -- get average bitrate over N frames.
 *
 * IMP_Encoder_GetChnAveBitrate(chn, stream, frames, double *br)
 * T31 only.
 */
int hal_enc_get_chn_ave_bitrate(void *ctx, int chn, void *stream, int frames, double *br)
{
    (void)ctx;
    if (!stream || !br)
        return RSS_ERR_INVAL;
#if defined(PLATFORM_T31)
    return IMP_Encoder_GetChnAveBitrate(chn, (IMPEncoderStream *)stream, frames, br);
#else
    (void)chn;
    (void)frames;
    return RSS_ERR_NOTSUP;
#endif
}

/*
 * hal_enc_set_chn_entropy_mode -- set entropy mode (CAVLC/CABAC).
 *
 * IMP_Encoder_SetChnEntropyMode(chn, IMPEncoderEntropyMode)
 * T31 only.
 */
int hal_enc_set_chn_entropy_mode(void *ctx, int chn, int mode)
{
    (void)ctx;
#if defined(PLATFORM_T31)
    return IMP_Encoder_SetChnEntropyMode(chn, (IMPEncoderEntropyMode)mode);
#else
    (void)chn;
    (void)mode;
    return RSS_ERR_NOTSUP;
#endif
}

/*
 * hal_enc_get_max_stream_cnt -- get max stream count for channel.
 *
 * IMP_Encoder_GetMaxStreamCnt(chn, int *nrMaxStream)
 * All SoCs.
 */
int hal_enc_get_max_stream_cnt(void *ctx, int chn, int *cnt)
{
    (void)ctx;
    if (!cnt)
        return RSS_ERR_INVAL;
    return IMP_Encoder_GetMaxStreamCnt(chn, cnt);
}

/*
 * hal_enc_set_max_stream_cnt -- set max stream count for channel.
 *
 * IMP_Encoder_SetMaxStreamCnt(chn, int nrMaxStream)
 * All SoCs.
 */
int hal_enc_set_max_stream_cnt(void *ctx, int chn, int cnt)
{
    (void)ctx;
    return IMP_Encoder_SetMaxStreamCnt(chn, cnt);
}

/*
 * hal_enc_set_pool -- bind encoder channel to memory pool.
 *
 * IMP_Encoder_SetPool(chn, poolID)
 * T23+T31+T32+T40+T41.
 */
int hal_enc_set_pool(void *ctx, int chn, int pool_id)
{
    (void)ctx;
#if defined(PLATFORM_T23) || defined(PLATFORM_T31) || defined(PLATFORM_T32) ||                     \
    defined(PLATFORM_T40) || defined(PLATFORM_T41)
    return IMP_Encoder_SetPool(chn, pool_id);
#else
    (void)chn;
    (void)pool_id;
    return RSS_ERR_NOTSUP;
#endif
}

/*
 * hal_enc_get_pool -- get memory pool ID for encoder channel.
 *
 * IMP_Encoder_GetPool(chn) -- returns pool ID directly.
 * T23+T31+T32+T40+T41.
 */
int hal_enc_get_pool(void *ctx, int chn)
{
    (void)ctx;
#if defined(PLATFORM_T23) || defined(PLATFORM_T31) || defined(PLATFORM_T32) ||                     \
    defined(PLATFORM_T40) || defined(PLATFORM_T41)
    return IMP_Encoder_GetPool(chn);
#else
    (void)chn;
    return RSS_ERR_NOTSUP;
#endif
}
