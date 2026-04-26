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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "hal_internal.h"

/* ── Constants ─────────────────────────────────────────────────────── */

#define RSS_MAX_NALS_PER_FRAME 16

/* Forward declaration — defined in Phase 6 (JPEG Features) section */
static int hal_jpeg_set_quality(int chn, int quality);

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
#if defined(HAL_HYBRID_SDK)
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

#if defined(HAL_NEW_SDK) && !defined(HAL_HYBRID_SDK)
/*
 * hal_ensure_scratch -- ensure the scratch linearization buffer exists.
 * Only needed on new SDK (except T32) for ring-buffer pack linearization.
 */
static int hal_ensure_scratch(rss_hal_ctx_t *c, int chn, size_t needed)
{
    if (chn < 0 || chn >= RSS_MAX_ENC_CHANNELS)
        return -EINVAL;

    if (c->scratch_size[chn] >= needed)
        return 0;

    size_t alloc = needed > RSS_SCRATCH_DEFAULT_SIZE ? needed : RSS_SCRATCH_DEFAULT_SIZE;
    uint8_t *buf = (uint8_t *)realloc(c->scratch_buf[chn], alloc);
    if (!buf)
        return -ENOMEM;

    c->scratch_buf[chn] = buf;
    c->scratch_size[chn] = alloc;
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

    /* GOP — old SDK rounds maxGop up to the next multiple of fps */
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
            chnAttr.rcAttr.attrRcMode.attrH264Cbr.maxQp =
                (cfg->max_qp >= 0) ? (uint32_t)cfg->max_qp : 45;
            chnAttr.rcAttr.attrRcMode.attrH264Cbr.minQp =
                (cfg->min_qp >= 0) ? (uint32_t)cfg->min_qp : 15;
            chnAttr.rcAttr.attrRcMode.attrH264Cbr.frmQPStep = 3;
            chnAttr.rcAttr.attrRcMode.attrH264Cbr.gopQPStep = 15;
            chnAttr.rcAttr.attrRcMode.attrH264Cbr.adaptiveMode = false;
            chnAttr.rcAttr.attrRcMode.attrH264Cbr.gopRelation = false;
        }
#if defined(PLATFORM_T21) || defined(PLATFORM_T23) || defined(PLATFORM_T30)
        else if (pt == PT_H265) {
            chnAttr.rcAttr.attrRcMode.attrH265Cbr.outBitRate = cfg->bitrate / 1000;
            chnAttr.rcAttr.attrRcMode.attrH265Cbr.maxQp =
                (cfg->max_qp >= 0) ? (uint32_t)cfg->max_qp : 45;
            chnAttr.rcAttr.attrRcMode.attrH265Cbr.minQp =
                (cfg->min_qp >= 0) ? (uint32_t)cfg->min_qp : 15;
            chnAttr.rcAttr.attrRcMode.attrH265Cbr.frmQPStep = 3;
            chnAttr.rcAttr.attrRcMode.attrH265Cbr.gopQPStep = 15;
        }
#endif
        break;

    case ENC_RC_MODE_VBR:
        if (pt == PT_H264) {
            chnAttr.rcAttr.attrRcMode.attrH264Vbr.maxBitRate = cfg->max_bitrate / 1000;
            chnAttr.rcAttr.attrRcMode.attrH264Vbr.maxQp =
                (cfg->max_qp >= 0) ? (uint32_t)cfg->max_qp : 45;
            chnAttr.rcAttr.attrRcMode.attrH264Vbr.minQp =
                (cfg->min_qp >= 0) ? (uint32_t)cfg->min_qp : 15;
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
            chnAttr.rcAttr.attrRcMode.attrH265Vbr.maxQp =
                (cfg->max_qp >= 0) ? (uint32_t)cfg->max_qp : 45;
            chnAttr.rcAttr.attrRcMode.attrH265Vbr.minQp =
                (cfg->min_qp >= 0) ? (uint32_t)cfg->min_qp : 15;
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
            chnAttr.rcAttr.attrRcMode.attrH264Smart.maxQp =
                (cfg->max_qp >= 0) ? (uint32_t)cfg->max_qp : 45;
            chnAttr.rcAttr.attrRcMode.attrH264Smart.minQp =
                (cfg->min_qp >= 0) ? (uint32_t)cfg->min_qp : 15;
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
            chnAttr.rcAttr.attrRcMode.attrH265Smart.maxQp =
                (cfg->max_qp >= 0) ? (uint32_t)cfg->max_qp : 45;
            chnAttr.rcAttr.attrRcMode.attrH265Smart.minQp =
                (cfg->min_qp >= 0) ? (uint32_t)cfg->min_qp : 15;
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

    /* hSkip controls hierarchical frame reference patterns (H1M mode),
     * not the IDR interval. For N1X (normal I/P encoding), the SDK
     * ignores m/n — IDR interval is controlled solely by maxGop.
     * Tested: identical output with and without hSkip on T20/jxf23.
     * Keep N1X for compatibility with prudynt-based configurations. */
    chnAttr.rcAttr.attrHSkip.hSkipAttr.skipType = IMP_Encoder_STYPE_N1X;
    chnAttr.rcAttr.attrHSkip.maxHSkipType = IMP_Encoder_STYPE_N1X;

#if defined(PLATFORM_T23)
    /* IVDC (ISP-VPU Direct Connect) — reduces rmem usage */
    chnAttr.bEnableIvdc = cfg->ivdc;
    if (cfg->ivdc)
        HAL_LOG_INFO("enc chn %d: IVDC enabled", chn);
#endif

    {
        int ret = IMP_Encoder_CreateChn(chn, &chnAttr);
        if (ret != 0)
            return ret;
    }

    /* Apply JPEG quantization tables after channel creation */
    if (pt == PT_JPEG) {
        int quality = (cfg->init_qp >= 0) ? cfg->init_qp : 25;
        hal_jpeg_set_quality(chn, quality);
    }

    return RSS_OK;
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
        ret = IMP_Encoder_CreateChn(chn, &chnAttr);
        if (ret != 0)
            return ret;
        /* Apply JPEG quantization tables (SetJpegeQl where available) */
        hal_jpeg_set_quality(chn, quality);
        return RSS_OK;
    }

    /*
     * SetDefaultParam signature:
     *   T31/T40/T41: (..., uMaxSameSenceCnt, iInitialQP, uTargetBitRate)
     *   T32:         (..., uBufSize, iInitialQP, uTargetBitRate)
     */
#if defined(HAL_HYBRID_SDK)
    ret = IMP_Encoder_SetDefaultParam(&chnAttr, profile, rc, cfg->width, cfg->height, cfg->fps_num,
                                      cfg->fps_den, cfg->gop_length,
                                      cfg->buf_size,               /* T32: uBufSize */
                                      init_qp, cfg->bitrate / 1000 /* SDK expects kbps */
    );
#else
    {
        int max_scene = (cfg->max_same_scene_cnt > 0) ? (int)cfg->max_same_scene_cnt : 2;
        uint32_t br_kbps = (rc == IMP_ENC_RC_MODE_FIXQP) ? 0 : cfg->bitrate / 1000;
        if (rc == IMP_ENC_RC_MODE_FIXQP && init_qp < 0)
            init_qp = 35;
        ret = IMP_Encoder_SetDefaultParam(&chnAttr, profile, rc, cfg->width, cfg->height,
                                          cfg->fps_num, cfg->fps_den, cfg->gop_length, max_scene,
                                          init_qp, br_kbps);
    }
#endif
    if (ret != 0) {
        HAL_LOG_ERR("SetDefaultParam failed: %d", ret);
        return ret;
    }

    /* Override QP bounds if caller specified non-default values.
     * T32 uses old-style RC attr member names (attrH264Cbr etc.)
     * while T31/T40/T41 use new-style (attrCbr etc.). */
#if defined(HAL_HYBRID_SDK)
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
    /* Override SDK defaults from SetDefaultParam.
     * When cfg value is -1 (unset), apply proven defaults matching prudynt.
     * SDK defaults (MinQP=15, MaxQP=48, MaxPictureSize=2*bitrate) produce
     * worse quality than prudynt's values. */
    {
        int32_t br_kbps = cfg->bitrate / 1000;
        int16_t cbr_min = (cfg->min_qp >= 0) ? (int16_t)cfg->min_qp : 34;
        int16_t cbr_max = (cfg->max_qp >= 0) ? (int16_t)cfg->max_qp : 51;
        int16_t vbr_min = (cfg->min_qp >= 0) ? (int16_t)cfg->min_qp : 20;
        int16_t vbr_max = (cfg->max_qp >= 0) ? (int16_t)cfg->max_qp : 45;

        switch (rc) {
        case IMP_ENC_RC_MODE_CBR:
            chnAttr.rcAttr.attrRcMode.attrCbr.iMinQP = cbr_min;
            chnAttr.rcAttr.attrRcMode.attrCbr.iMaxQP = cbr_max;
            if (cfg->bitrate > 0) {
                chnAttr.rcAttr.attrRcMode.attrCbr.uTargetBitRate = br_kbps;
                chnAttr.rcAttr.attrRcMode.attrCbr.uMaxPictureSize = br_kbps;
            }
            break;

        case IMP_ENC_RC_MODE_VBR:
            chnAttr.rcAttr.attrRcMode.attrVbr.iMinQP = vbr_min;
            chnAttr.rcAttr.attrRcMode.attrVbr.iMaxQP = vbr_max;
            if (cfg->bitrate > 0) {
                chnAttr.rcAttr.attrRcMode.attrVbr.uTargetBitRate = br_kbps;
                chnAttr.rcAttr.attrRcMode.attrVbr.uMaxPictureSize = br_kbps;
            }
            if (cfg->max_bitrate > 0)
                chnAttr.rcAttr.attrRcMode.attrVbr.uMaxBitRate = cfg->max_bitrate / 1000;
            if (cfg->ip_delta >= 0)
                chnAttr.rcAttr.attrRcMode.attrVbr.iIPDelta = cfg->ip_delta;
            if (cfg->pb_delta >= 0)
                chnAttr.rcAttr.attrRcMode.attrVbr.iPBDelta = cfg->pb_delta;
            break;

        case IMP_ENC_RC_MODE_CAPPED_VBR:
            chnAttr.rcAttr.attrRcMode.attrCappedVbr.iMinQP = vbr_min;
            chnAttr.rcAttr.attrRcMode.attrCappedVbr.iMaxQP = vbr_max;
            if (cfg->bitrate > 0) {
                chnAttr.rcAttr.attrRcMode.attrCappedVbr.uTargetBitRate = br_kbps;
                chnAttr.rcAttr.attrRcMode.attrCappedVbr.uMaxPictureSize = br_kbps;
            }
            if (cfg->max_bitrate > 0)
                chnAttr.rcAttr.attrRcMode.attrCappedVbr.uMaxBitRate = cfg->max_bitrate / 1000;
            if (cfg->ip_delta >= 0)
                chnAttr.rcAttr.attrRcMode.attrCappedVbr.iIPDelta = cfg->ip_delta;
            if (cfg->pb_delta >= 0)
                chnAttr.rcAttr.attrRcMode.attrCappedVbr.iPBDelta = cfg->pb_delta;
            if (cfg->max_psnr > 0)
                chnAttr.rcAttr.attrRcMode.attrCappedVbr.uMaxPSNR = cfg->max_psnr;
            break;

        case IMP_ENC_RC_MODE_CAPPED_QUALITY:
            chnAttr.rcAttr.attrRcMode.attrCappedQuality.iMinQP = vbr_min;
            chnAttr.rcAttr.attrRcMode.attrCappedQuality.iMaxQP = vbr_max;
            if (cfg->bitrate > 0) {
                chnAttr.rcAttr.attrRcMode.attrCappedQuality.uTargetBitRate = br_kbps;
                chnAttr.rcAttr.attrRcMode.attrCappedQuality.uMaxPictureSize = br_kbps;
            }
            if (cfg->max_bitrate > 0)
                chnAttr.rcAttr.attrRcMode.attrCappedQuality.uMaxBitRate = cfg->max_bitrate / 1000;
            if (cfg->ip_delta >= 0)
                chnAttr.rcAttr.attrRcMode.attrCappedQuality.iIPDelta = cfg->ip_delta;
            if (cfg->pb_delta >= 0)
                chnAttr.rcAttr.attrRcMode.attrCappedQuality.iPBDelta = cfg->pb_delta;
            if (cfg->max_psnr > 0)
                chnAttr.rcAttr.attrRcMode.attrCappedQuality.uMaxPSNR = cfg->max_psnr;
            break;

        default:
            break;
        }
    }

    /* GOP length and mode */
    chnAttr.gopAttr.uGopLength = (uint16_t)cfg->gop_length;
    if (cfg->max_same_scene_cnt > 0)
        chnAttr.gopAttr.uMaxSameSenceCnt = cfg->max_same_scene_cnt;
    if (cfg->gop_mode == RSS_GOP_SMARTP)
        chnAttr.gopAttr.uGopCtrlMode = IMP_ENC_GOP_CTRL_MODE_SMARTP;
    else if (cfg->gop_mode == RSS_GOP_PYRAMIDAL)
        chnAttr.gopAttr.uGopCtrlMode = IMP_ENC_GOP_CTRL_MODE_PYRAMIDAL;

    /* RC options bitmask — apply to whichever RC mode struct is active */
    if (cfg->rc_options != 0) {
        IMPEncoderRcOptions opts = (IMPEncoderRcOptions)cfg->rc_options;
        switch (rc) {
        case IMP_ENC_RC_MODE_CBR:
            chnAttr.rcAttr.attrRcMode.attrCbr.eRcOptions = opts;
            break;
        case IMP_ENC_RC_MODE_VBR:
            chnAttr.rcAttr.attrRcMode.attrVbr.eRcOptions = opts;
            break;
        case IMP_ENC_RC_MODE_CAPPED_VBR:
            chnAttr.rcAttr.attrRcMode.attrCappedVbr.eRcOptions = opts;
            break;
        case IMP_ENC_RC_MODE_CAPPED_QUALITY:
            chnAttr.rcAttr.attrRcMode.attrCappedQuality.eRcOptions = opts;
            break;
        default:
            break;
        }
    }
#endif

#if defined(HAL_HYBRID_SDK) || defined(PLATFORM_T41)
    /* IVDC (ISP-VPU Direct Connect) — reduces rmem usage */
    chnAttr.bEnableIvdc = cfg->ivdc;
    if (cfg->ivdc)
        HAL_LOG_INFO("enc chn %d: IVDC enabled", chn);
#endif

    /* Pre-CreateChn tuning: must be set while channel is uninitialized */
    if (cfg->max_stream_cnt > 0) {
        HAL_LOG_INFO("enc chn %d: SetMaxStreamCnt(%d)", chn, cfg->max_stream_cnt);
        IMP_Encoder_SetMaxStreamCnt(chn, cfg->max_stream_cnt);
    }
#if !defined(HAL_HYBRID_SDK)
    if (cfg->stream_buf_size > 0) {
        HAL_LOG_INFO("enc chn %d: SetStreamBufSize(%u)", chn, cfg->stream_buf_size);
        IMP_Encoder_SetStreamBufSize(chn, cfg->stream_buf_size);
    }
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

#if defined(HAL_OLD_SDK) || defined(HAL_HYBRID_SDK)
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
        if (stream.pack[i].offset > stream_size ||
            stream.pack[i].length > stream_size - stream.pack[i].offset)
            needs_linearize = true;
    }

    /* Allocate scratch buffer only if wrapping detected */
    uint8_t *scratch = NULL;
    if (needs_linearize) {
        ret = hal_ensure_scratch(c, chn, total_size);
        if (ret != 0) {
            IMP_Encoder_ReleaseStream(chn, &stream);
            return ret;
        }
        scratch = c->scratch_buf[chn];
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
     * Store the vendor stream struct in the per-channel preallocated slot
     * (avoids malloc/free on every frame at 30fps × N streams). */
    memcpy(&c->stream_priv[chn], &stream, sizeof(stream));
    frame->_priv = &c->stream_priv[chn];

    return 0;
}

int hal_enc_release_frame(void *ctx, int chn, rss_frame_t *frame)
{
    (void)ctx;

    if (!frame || !frame->_priv)
        return -EINVAL;

    IMPEncoderStream *stream = (IMPEncoderStream *)frame->_priv;
    int ret = IMP_Encoder_ReleaseStream(chn, stream);

    /* _priv points into ctx->stream_priv[] — not heap-allocated, don't free */
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
    int flush_ret = IMP_Encoder_FlushStream(chn);
    if (flush_ret != 0)
        HAL_LOG_ERR("IMP_Encoder_FlushStream(%d) failed: %d", chn, flush_ret);
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

#if defined(HAL_NEW_SDK) && !defined(HAL_HYBRID_SDK)
    /* New SDK (T31/T40/T41): patch bitrate in the target mode struct */
    switch (vendor_mode) {
    case IMP_ENC_RC_MODE_FIXQP:
        if (rcAttr.attrFixQp.iInitialQP < 1)
            rcAttr.attrFixQp.iInitialQP = 35;
        break;
    case IMP_ENC_RC_MODE_CBR:
        rcAttr.attrCbr.uTargetBitRate = bitrate_kbps;
        rcAttr.attrCbr.uMaxPictureSize = bitrate_kbps;
        if (rcAttr.attrCbr.iMaxQP == 0)
            rcAttr.attrCbr.iMaxQP = 51;
        if (rcAttr.attrCbr.iMinQP == 0)
            rcAttr.attrCbr.iMinQP = 34;
        break;
    case IMP_ENC_RC_MODE_VBR:
        rcAttr.attrVbr.uTargetBitRate = bitrate_kbps;
        rcAttr.attrVbr.uMaxBitRate = bitrate_kbps * 4 / 3;
        rcAttr.attrVbr.uMaxPictureSize = bitrate_kbps;
        if (rcAttr.attrVbr.iMaxQP == 0)
            rcAttr.attrVbr.iMaxQP = 45;
        if (rcAttr.attrVbr.iMinQP == 0)
            rcAttr.attrVbr.iMinQP = 20;
        break;
    case IMP_ENC_RC_MODE_CAPPED_VBR:
        rcAttr.attrCappedVbr.uTargetBitRate = bitrate_kbps;
        rcAttr.attrCappedVbr.uMaxBitRate = bitrate_kbps * 4 / 3;
        rcAttr.attrCappedVbr.uMaxPictureSize = bitrate_kbps;
        if (rcAttr.attrCappedVbr.iMaxQP == 0)
            rcAttr.attrCappedVbr.iMaxQP = 45;
        if (rcAttr.attrCappedVbr.iMinQP == 0)
            rcAttr.attrCappedVbr.iMinQP = 20;
        break;
    case IMP_ENC_RC_MODE_CAPPED_QUALITY:
        rcAttr.attrCappedQuality.uTargetBitRate = bitrate_kbps;
        rcAttr.attrCappedQuality.uMaxBitRate = bitrate_kbps * 4 / 3;
        rcAttr.attrCappedQuality.uMaxPictureSize = bitrate_kbps;
        if (rcAttr.attrCappedQuality.iMaxQP == 0)
            rcAttr.attrCappedQuality.iMaxQP = 45;
        if (rcAttr.attrCappedQuality.iMinQP == 0)
            rcAttr.attrCappedQuality.iMinQP = 20;
        break;
    default:
        break;
    }
#elif defined(HAL_HYBRID_SDK)
    /* T32 hybrid: H264-prefixed structs but different member names */
    switch (vendor_mode) {
    case ENC_RC_MODE_FIXQP:
        if (rcAttr.attrH264FixQp.IQp == 0)
            rcAttr.attrH264FixQp.IQp = 35;
        break;
    case ENC_RC_MODE_CBR:
        rcAttr.attrH264Cbr.outBitRate = bitrate_kbps;
        if (rcAttr.attrH264Cbr.maxQp == 0)
            rcAttr.attrH264Cbr.maxQp = 45;
        if (rcAttr.attrH264Cbr.minQp == 0)
            rcAttr.attrH264Cbr.minQp = 15;
        break;
    case ENC_RC_MODE_VBR:
        rcAttr.attrH264Vbr.maxBitRate = bitrate_kbps;
        if (rcAttr.attrH264Vbr.maxQp == 0)
            rcAttr.attrH264Vbr.maxQp = 45;
        if (rcAttr.attrH264Vbr.minQp == 0)
            rcAttr.attrH264Vbr.minQp = 15;
        break;
    case ENC_RC_MODE_SMART:
        rcAttr.attrH264Smart.maxBitRate = bitrate_kbps;
        if (rcAttr.attrH264Smart.maxQp == 0)
            rcAttr.attrH264Smart.maxQp = 45;
        if (rcAttr.attrH264Smart.minQp == 0)
            rcAttr.attrH264Smart.minQp = 15;
        break;
    default:
        break;
    }
#else
    /* Old SDK (T20/T21/T23/T30): H264-prefixed structs with QP step fields */
    switch (vendor_mode) {
    case ENC_RC_MODE_FIXQP:
        if (rcAttr.attrH264FixQp.qp == 0)
            rcAttr.attrH264FixQp.qp = 35;
        break;
    case ENC_RC_MODE_CBR:
        rcAttr.attrH264Cbr.outBitRate = bitrate_kbps;
        if (rcAttr.attrH264Cbr.maxQp == 0)
            rcAttr.attrH264Cbr.maxQp = 45;
        if (rcAttr.attrH264Cbr.minQp == 0)
            rcAttr.attrH264Cbr.minQp = 15;
        if (rcAttr.attrH264Cbr.frmQPStep == 0)
            rcAttr.attrH264Cbr.frmQPStep = 3;
        if (rcAttr.attrH264Cbr.gopQPStep == 0)
            rcAttr.attrH264Cbr.gopQPStep = 15;
        break;
    case ENC_RC_MODE_VBR:
        rcAttr.attrH264Vbr.maxBitRate = bitrate_kbps;
        if (rcAttr.attrH264Vbr.maxQp == 0)
            rcAttr.attrH264Vbr.maxQp = 45;
        if (rcAttr.attrH264Vbr.minQp == 0)
            rcAttr.attrH264Vbr.minQp = 15;
        if (rcAttr.attrH264Vbr.frmQPStep == 0)
            rcAttr.attrH264Vbr.frmQPStep = 3;
        if (rcAttr.attrH264Vbr.gopQPStep == 0)
            rcAttr.attrH264Vbr.gopQPStep = 15;
        break;
    case ENC_RC_MODE_SMART:
        rcAttr.attrH264Smart.maxBitRate = bitrate_kbps;
        if (rcAttr.attrH264Smart.maxQp == 0)
            rcAttr.attrH264Smart.maxQp = 45;
        if (rcAttr.attrH264Smart.minQp == 0)
            rcAttr.attrH264Smart.minQp = 15;
        if (rcAttr.attrH264Smart.frmQPStep == 0)
            rcAttr.attrH264Smart.frmQPStep = 3;
        if (rcAttr.attrH264Smart.gopQPStep == 0)
            rcAttr.attrH264Smart.gopQPStep = 15;
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
#if defined(HAL_HYBRID_SDK)
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

#if defined(HAL_NEW_SDK) && !defined(HAL_HYBRID_SDK)
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

#if defined(HAL_NEW_SDK) && !defined(HAL_HYBRID_SDK)
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

#if defined(HAL_NEW_SDK) && !defined(HAL_HYBRID_SDK)
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
 * T31: direct API. Other new SDK: patch via Get/SetChnAttrRcMode.
 */
int hal_enc_set_qp_ip_delta(void *ctx, int chn, int delta)
{
    (void)ctx;
#if defined(PLATFORM_T31)
    return IMP_Encoder_SetChnQpIPDelta(chn, delta);
#elif defined(HAL_NEW_SDK) && !defined(HAL_HYBRID_SDK)
    int ret;
    IMPEncoderAttrRcMode rcAttr;
    memset(&rcAttr, 0, sizeof(rcAttr));
    ret = IMP_Encoder_GetChnAttrRcMode(chn, &rcAttr);
    if (ret != 0)
        return ret;
    switch (rcAttr.rcMode) {
    case IMP_ENC_RC_MODE_VBR:
        rcAttr.attrVbr.iIPDelta = (int16_t)delta;
        break;
    case IMP_ENC_RC_MODE_CAPPED_VBR:
        rcAttr.attrCappedVbr.iIPDelta = (int16_t)delta;
        break;
    case IMP_ENC_RC_MODE_CAPPED_QUALITY:
        rcAttr.attrCappedQuality.iIPDelta = (int16_t)delta;
        break;
    default:
        return RSS_ERR_NOTSUP;
    }
    return IMP_Encoder_SetChnAttrRcMode(chn, &rcAttr);
#else
    (void)chn;
    (void)delta;
    return RSS_ERR_NOTSUP;
#endif
}

/*
 * hal_enc_set_qp_pb_delta -- set QP delta between P and B frames.
 */
int hal_enc_set_qp_pb_delta(void *ctx, int chn, int delta)
{
    (void)ctx;
#if defined(HAL_NEW_SDK) && !defined(HAL_HYBRID_SDK)
    int ret;
    IMPEncoderAttrRcMode rcAttr;
    memset(&rcAttr, 0, sizeof(rcAttr));
    ret = IMP_Encoder_GetChnAttrRcMode(chn, &rcAttr);
    if (ret != 0)
        return ret;
    switch (rcAttr.rcMode) {
    case IMP_ENC_RC_MODE_VBR:
        rcAttr.attrVbr.iPBDelta = (int16_t)delta;
        break;
    case IMP_ENC_RC_MODE_CAPPED_VBR:
        rcAttr.attrCappedVbr.iPBDelta = (int16_t)delta;
        break;
    case IMP_ENC_RC_MODE_CAPPED_QUALITY:
        rcAttr.attrCappedQuality.iPBDelta = (int16_t)delta;
        break;
    default:
        return RSS_ERR_NOTSUP;
    }
    return IMP_Encoder_SetChnAttrRcMode(chn, &rcAttr);
#else
    (void)chn;
    (void)delta;
    return RSS_ERR_NOTSUP;
#endif
}

/*
 * hal_enc_set_max_psnr -- set PSNR quality cap for capped_vbr/capped_quality.
 */
int hal_enc_set_max_psnr(void *ctx, int chn, int psnr)
{
    (void)ctx;
#if defined(HAL_NEW_SDK) && !defined(HAL_HYBRID_SDK)
    int ret;
    IMPEncoderAttrRcMode rcAttr;
    memset(&rcAttr, 0, sizeof(rcAttr));
    ret = IMP_Encoder_GetChnAttrRcMode(chn, &rcAttr);
    if (ret != 0)
        return ret;
    switch (rcAttr.rcMode) {
    case IMP_ENC_RC_MODE_CAPPED_VBR:
        rcAttr.attrCappedVbr.uMaxPSNR = (uint16_t)psnr;
        break;
    case IMP_ENC_RC_MODE_CAPPED_QUALITY:
        rcAttr.attrCappedQuality.uMaxPSNR = (uint16_t)psnr;
        break;
    default:
        return RSS_ERR_NOTSUP;
    }
    return IMP_Encoder_SetChnAttrRcMode(chn, &rcAttr);
#else
    (void)chn;
    (void)psnr;
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
#elif defined(HAL_NEW_SDK) && !defined(HAL_HYBRID_SDK)
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
#if defined(PLATFORM_T23) || defined(PLATFORM_T31) || defined(HAL_HYBRID_SDK) ||                   \
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
#if defined(PLATFORM_T23) || defined(PLATFORM_T31) || defined(HAL_HYBRID_SDK) ||                   \
    defined(PLATFORM_T40) || defined(PLATFORM_T41)
    return IMP_Encoder_GetPool(chn);
#else
    (void)chn;
    return RSS_ERR_NOTSUP;
#endif
}

/*
 * hal_enc_get_rmem_info -- find /dev/rmem virtual mapping in this process.
 *
 * Parses /proc/self/maps for the /dev/rmem mmap region created by libimp.
 * Returns the virtual base address and size. Used by RVD to compute rmem
 * offsets from encoder virAddr pointers for zero-copy ring reference mode.
 */
int hal_enc_get_rmem_info(void *ctx, uintptr_t *virt_base, uint32_t *size, uint32_t *mmap_offset)
{
    (void)ctx;
    if (!virt_base || !size || !mmap_offset)
        return -EINVAL;

    FILE *f = fopen("/proc/self/maps", "r");
    if (!f)
        return -EIO;

    /* Format: start-end perms offset dev inode pathname
     * Example: 756a5000-76fa5000 rw-s 02700000 00:05 92 /dev/rmem */
    char line[256];
    int found = 0;
    while (fgets(line, sizeof(line), f)) {
        if (!strstr(line, "/dev/rmem"))
            continue;
        uintptr_t start, end;
        unsigned long offset;
        if (sscanf(line, "%lx-%lx %*s %lx", (unsigned long *)&start, (unsigned long *)&end,
                   &offset) == 3) {
            *virt_base = start;
            *size = (uint32_t)(end - start);
            *mmap_offset = (uint32_t)offset;
            found = 1;
            break;
        }
    }

    fclose(f);
    return found ? 0 : -ENOENT;
}

/*
 * hal_enc_inject_stream_shm -- inject POSIX SHM as encoder output buffer.
 *
 * Runtime-probes the libimp channel struct by writing a marker via
 * SetMaxStreamCnt, scanning libimp's data segment for it, then writing
 * the SHM address to the external buffer fields. Must be called BEFORE
 * IMP_Encoder_CreateChn.
 *
 * WARNING: this is a fragile technique that relies on the internal layout
 * of libimp's channel struct. A vendor SDK update that changes the struct
 * layout will silently corrupt encoder state. The marker-verify step
 * catches some false positives but not all. Only used on old-SDK SoCs
 * (T10-T30/T32/T33) where no official external-buffer API exists.
 *
 * The shm_addr and shm_size values come from the caller (RVD) which
 * creates the SHM internally — they are NOT from external input.
 */
static int find_libimp_rw_region(uintptr_t *start, size_t *size)
{
    FILE *f = fopen("/proc/self/maps", "r");
    if (!f)
        return -1;

    char line[512];
    uintptr_t seg_start = 0, seg_end = 0;
    int found = 0;

    while (fgets(line, sizeof(line), f)) {
        uintptr_t s, e;
        if (sscanf(line, "%lx-%lx", (unsigned long *)&s, (unsigned long *)&e) != 2)
            continue;
        if (strstr(line, "libimp.so") && strstr(line, "rw-")) {
            if (!found || s < seg_start)
                seg_start = s;
            if (e > seg_end)
                seg_end = e;
            found = 1;
        } else if (found && s == seg_end && strstr(line, "rw-p") && !strstr(line, ".so") &&
                   !strstr(line, "[")) {
            seg_end = e;
        }
    }
    fclose(f);

    if (!found)
        return -1;
    *start = seg_start;
    *size = seg_end - seg_start;
    return 0;
}

int hal_enc_inject_stream_shm(void *ctx, int chn, void *shm_addr, uint32_t shm_size)
{
    (void)ctx;
    if (!shm_addr || shm_size == 0)
        return -EINVAL;

    uintptr_t region_start;
    size_t region_size;
    if (find_libimp_rw_region(&region_start, &region_size) < 0 || region_size < 16) {
        HAL_LOG_ERR("inject_stream_shm: can't find libimp rw region");
        return -ENOENT;
    }

    int orig_cnt = 0;
    IMP_Encoder_GetMaxStreamCnt(chn, &orig_cnt);

    uint32_t marker = 0xDEADBEEF;
    IMP_Encoder_SetMaxStreamCnt(chn, (int)marker);

    uint32_t *p = (uint32_t *)region_start;
    uint32_t *end = (uint32_t *)(region_start + region_size - 4);
    uintptr_t marker_addr = 0;
    while (p <= end) {
        if (*p == marker) {
            marker_addr = (uintptr_t)p;
            break;
        }
        p++;
    }

    IMP_Encoder_SetMaxStreamCnt(chn, orig_cnt);

    if (!marker_addr) {
        HAL_LOG_ERR("inject_stream_shm: marker not found in libimp");
        return -ENOENT;
    }

    /* Verify: after restoring orig_cnt, the found address should hold it.
     * Guards against false positives from marker collision. */
    if (*(uint32_t *)marker_addr != (uint32_t)orig_cnt) {
        HAL_LOG_ERR("inject_stream_shm: marker verify failed (collision?)");
        return -ENOENT;
    }

    /* ext_buf = marker_addr + 8, ext_size = marker_addr + 12
     * Offsets verified on T20/T23/T30 libimp channel struct layout. */
    uint32_t *ext_buf = (uint32_t *)(marker_addr + 8);
    uint32_t *ext_sz = (uint32_t *)(marker_addr + 12);

    *ext_buf = (uint32_t)(uintptr_t)shm_addr;
    *ext_sz = shm_size;

    HAL_LOG_INFO("inject_stream_shm chn %d: shm=0x%lx size=%u marker_off=+0x%lx", chn,
                 (unsigned long)(uintptr_t)shm_addr, shm_size,
                 (unsigned long)(marker_addr - region_start));
    return 0;
}

/* ══════════════════════════════════════════════════════════════════════
 * 11. Phase 1 — Bandwidth Reduction Features
 * ══════════════════════════════════════════════════════════════════════ */

/*
 * hal_enc_set_gop_mode -- set GOP control mode at runtime.
 *
 * T31/T40/T41: read gopAttr, change uGopCtrlMode, write back.
 * Others: not supported.
 */
int hal_enc_set_gop_mode(void *ctx, int chn, rss_gop_mode_t mode)
{
    (void)ctx;
#if defined(PLATFORM_T31) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
    IMPEncoderGopAttr gopAttr;
    int ret = IMP_Encoder_GetChnGopAttr(chn, &gopAttr);
    if (ret != 0) {
        HAL_LOG_ERR("GetChnGopAttr(%d) failed: %d", chn, ret);
        return ret;
    }

    switch (mode) {
    case RSS_GOP_SMARTP:
        gopAttr.uGopCtrlMode = IMP_ENC_GOP_CTRL_MODE_SMARTP;
        break;
    case RSS_GOP_PYRAMIDAL:
        gopAttr.uGopCtrlMode = IMP_ENC_GOP_CTRL_MODE_PYRAMIDAL;
        break;
    default:
        gopAttr.uGopCtrlMode = IMP_ENC_GOP_CTRL_MODE_DEFAULT;
        break;
    }

    ret = IMP_Encoder_SetChnGopAttr(chn, &gopAttr);
    if (ret != 0)
        HAL_LOG_ERR("SetChnGopAttr(%d) gop_mode failed: %d", chn, ret);
    return ret;
#else
    (void)chn;
    (void)mode;
    return RSS_ERR_NOTSUP;
#endif
}

/*
 * hal_enc_get_gop_mode -- get current GOP control mode.
 */
int hal_enc_get_gop_mode(void *ctx, int chn, rss_gop_mode_t *mode)
{
    (void)ctx;
    if (!mode)
        return RSS_ERR_INVAL;
#if defined(PLATFORM_T31) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
    IMPEncoderGopAttr gopAttr;
    int ret = IMP_Encoder_GetChnGopAttr(chn, &gopAttr);
    if (ret != 0) {
        HAL_LOG_ERR("GetChnGopAttr(%d) failed: %d", chn, ret);
        return ret;
    }

    switch (gopAttr.uGopCtrlMode) {
    case IMP_ENC_GOP_CTRL_MODE_SMARTP:
        *mode = RSS_GOP_SMARTP;
        break;
    case IMP_ENC_GOP_CTRL_MODE_PYRAMIDAL:
        *mode = RSS_GOP_PYRAMIDAL;
        break;
    default:
        *mode = RSS_GOP_DEFAULT;
        break;
    }
    return RSS_OK;
#else
    (void)chn;
    return RSS_ERR_NOTSUP;
#endif
}

/*
 * hal_enc_set_rc_options -- set RC options bitmask at runtime.
 *
 * T31/T40/T41: read current RC mode attrs, patch eRcOptions, write back.
 */
int hal_enc_set_rc_options(void *ctx, int chn, uint32_t options)
{
    (void)ctx;
#if defined(PLATFORM_T31) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
    IMPEncoderAttrRcMode rcAttr;
    int ret = IMP_Encoder_GetChnAttrRcMode(chn, &rcAttr);
    if (ret != 0) {
        HAL_LOG_ERR("GetChnAttrRcMode(%d) failed: %d", chn, ret);
        return ret;
    }

    IMPEncoderRcOptions opts = (IMPEncoderRcOptions)options;

    switch (rcAttr.rcMode) {
    case IMP_ENC_RC_MODE_CBR:
        rcAttr.attrCbr.eRcOptions = opts;
        break;
    case IMP_ENC_RC_MODE_VBR:
        rcAttr.attrVbr.eRcOptions = opts;
        break;
    case IMP_ENC_RC_MODE_CAPPED_VBR:
        rcAttr.attrCappedVbr.eRcOptions = opts;
        break;
    case IMP_ENC_RC_MODE_CAPPED_QUALITY:
        rcAttr.attrCappedQuality.eRcOptions = opts;
        break;
    default:
        return RSS_ERR_NOTSUP;
    }

    ret = IMP_Encoder_SetChnAttrRcMode(chn, &rcAttr);
    if (ret != 0)
        HAL_LOG_ERR("SetChnAttrRcMode(%d) rc_options failed: %d", chn, ret);
    return ret;
#else
    (void)chn;
    (void)options;
    return RSS_ERR_NOTSUP;
#endif
}

/*
 * hal_enc_get_rc_options -- get current RC options bitmask.
 */
int hal_enc_get_rc_options(void *ctx, int chn, uint32_t *options)
{
    (void)ctx;
    if (!options)
        return RSS_ERR_INVAL;
#if defined(PLATFORM_T31) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
    IMPEncoderAttrRcMode rcAttr;
    int ret = IMP_Encoder_GetChnAttrRcMode(chn, &rcAttr);
    if (ret != 0) {
        HAL_LOG_ERR("GetChnAttrRcMode(%d) failed: %d", chn, ret);
        return ret;
    }

    switch (rcAttr.rcMode) {
    case IMP_ENC_RC_MODE_CBR:
        *options = (uint32_t)rcAttr.attrCbr.eRcOptions;
        break;
    case IMP_ENC_RC_MODE_VBR:
        *options = (uint32_t)rcAttr.attrVbr.eRcOptions;
        break;
    case IMP_ENC_RC_MODE_CAPPED_VBR:
        *options = (uint32_t)rcAttr.attrCappedVbr.eRcOptions;
        break;
    case IMP_ENC_RC_MODE_CAPPED_QUALITY:
        *options = (uint32_t)rcAttr.attrCappedQuality.eRcOptions;
        break;
    default:
        *options = 0;
        break;
    }
    return RSS_OK;
#else
    (void)chn;
    return RSS_ERR_NOTSUP;
#endif
}

/*
 * hal_enc_set_max_same_scene_cnt -- set max same scene reference count.
 *
 * T31/T40/T41: modify gopAttr.uMaxSameSenceCnt via Get/SetChnGopAttr.
 */
int hal_enc_set_max_same_scene_cnt(void *ctx, int chn, uint32_t count)
{
    (void)ctx;
#if defined(PLATFORM_T31) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
    IMPEncoderGopAttr gopAttr;
    int ret = IMP_Encoder_GetChnGopAttr(chn, &gopAttr);
    if (ret != 0) {
        HAL_LOG_ERR("GetChnGopAttr(%d) failed: %d", chn, ret);
        return ret;
    }

    gopAttr.uMaxSameSenceCnt = count;
    ret = IMP_Encoder_SetChnGopAttr(chn, &gopAttr);
    if (ret != 0)
        HAL_LOG_ERR("SetChnGopAttr(%d) max_same_scene failed: %d", chn, ret);
    return ret;
#else
    (void)chn;
    (void)count;
    return RSS_ERR_NOTSUP;
#endif
}

/*
 * hal_enc_get_max_same_scene_cnt -- get max same scene reference count.
 */
int hal_enc_get_max_same_scene_cnt(void *ctx, int chn, uint32_t *count)
{
    (void)ctx;
    if (!count)
        return RSS_ERR_INVAL;
#if defined(PLATFORM_T31) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
    IMPEncoderGopAttr gopAttr;
    int ret = IMP_Encoder_GetChnGopAttr(chn, &gopAttr);
    if (ret != 0) {
        HAL_LOG_ERR("GetChnGopAttr(%d) failed: %d", chn, ret);
        return ret;
    }
    *count = gopAttr.uMaxSameSenceCnt;
    return RSS_OK;
#else
    (void)chn;
    return RSS_ERR_NOTSUP;
#endif
}

/*
 * hal_enc_set_pskip -- configure P-skip for static scenes.
 *
 * T32 only: IMP_Encoder_SetPskipCfg.
 */
int hal_enc_set_pskip(void *ctx, int chn, const rss_pskip_cfg_t *cfg)
{
    (void)ctx;
    if (!cfg)
        return RSS_ERR_INVAL;
#if defined(HAL_HYBRID_SDK)
    IMPEncoderPskipCfg pskip;
    memset(&pskip, 0, sizeof(pskip));
    pskip.enable = cfg->enable;
    pskip.pskipMaxFrames = cfg->max_frames;
    pskip.pskipThr = cfg->threshold;
    int ret = IMP_Encoder_SetPskipCfg(chn, &pskip);
    if (ret != 0)
        HAL_LOG_ERR("SetPskipCfg(%d) failed: %d", chn, ret);
    return ret;
#else
    (void)chn;
    return RSS_ERR_NOTSUP;
#endif
}

/*
 * hal_enc_get_pskip -- get P-skip configuration.
 */
int hal_enc_get_pskip(void *ctx, int chn, rss_pskip_cfg_t *cfg)
{
    (void)ctx;
    if (!cfg)
        return RSS_ERR_INVAL;
#if defined(HAL_HYBRID_SDK)
    IMPEncoderPskipCfg pskip;
    int ret = IMP_Encoder_GetPskipCfg(chn, &pskip);
    if (ret != 0) {
        HAL_LOG_ERR("GetPskipCfg(%d) failed: %d", chn, ret);
        return ret;
    }
    cfg->enable = pskip.enable;
    cfg->max_frames = pskip.pskipMaxFrames;
    cfg->threshold = pskip.pskipThr;
    return RSS_OK;
#else
    (void)chn;
    return RSS_ERR_NOTSUP;
#endif
}

/*
 * hal_enc_request_pskip -- request immediate P-skip.
 */
int hal_enc_request_pskip(void *ctx, int chn)
{
    (void)ctx;
#if defined(HAL_HYBRID_SDK)
    int ret = IMP_Encoder_RequestPskip(chn);
    if (ret != 0)
        HAL_LOG_ERR("RequestPskip(%d) failed: %d", chn, ret);
    return ret;
#else
    (void)chn;
    return RSS_ERR_NOTSUP;
#endif
}

/*
 * hal_enc_set_srd -- configure spatial redundancy detection (H265 only).
 *
 * T32 only: IMP_Encoder_SetSrdCfg.
 */
int hal_enc_set_srd(void *ctx, int chn, const rss_srd_cfg_t *cfg)
{
    (void)ctx;
    if (!cfg)
        return RSS_ERR_INVAL;
#if defined(HAL_HYBRID_SDK)
    IMPEncoderSrdCfg srd;
    memset(&srd, 0, sizeof(srd));
    srd.enable = cfg->enable;
    srd.level = cfg->level;
    int ret = IMP_Encoder_SetSrdCfg(chn, &srd);
    if (ret != 0)
        HAL_LOG_ERR("SetSrdCfg(%d) failed: %d", chn, ret);
    return ret;
#else
    (void)chn;
    return RSS_ERR_NOTSUP;
#endif
}

/*
 * hal_enc_get_srd -- get SRD configuration.
 */
int hal_enc_get_srd(void *ctx, int chn, rss_srd_cfg_t *cfg)
{
    (void)ctx;
    if (!cfg)
        return RSS_ERR_INVAL;
#if defined(HAL_HYBRID_SDK)
    IMPEncoderSrdCfg srd;
    int ret = IMP_Encoder_GetSrdCfg(chn, &srd);
    if (ret != 0) {
        HAL_LOG_ERR("GetSrdCfg(%d) failed: %d", chn, ret);
        return ret;
    }
    cfg->enable = srd.enable;
    cfg->level = srd.level;
    return RSS_OK;
#else
    (void)chn;
    return RSS_ERR_NOTSUP;
#endif
}

/*
 * hal_enc_set_max_pic_size -- cap max I/P frame size.
 *
 * T32/T40/T41: IMP_Encoder_SetChnMaxPictureSize(chn, maxI_kbits, maxP_kbits).
 */
int hal_enc_set_max_pic_size(void *ctx, int chn, uint32_t max_i_kbits, uint32_t max_p_kbits)
{
    (void)ctx;
#if defined(HAL_HYBRID_SDK) || defined(PLATFORM_T41)
    int ret = IMP_Encoder_SetChnMaxPictureSize(chn, max_i_kbits, max_p_kbits);
    if (ret != 0)
        HAL_LOG_ERR("SetChnMaxPictureSize(%d, I=%u, P=%u) failed: %d", chn, max_i_kbits,
                    max_p_kbits, ret);
    return ret;
#else
    (void)chn;
    (void)max_i_kbits;
    (void)max_p_kbits;
    return RSS_ERR_NOTSUP;
#endif
}

/*
 * hal_enc_set_super_frame -- configure oversized frame handling.
 *
 * T21: IMPEncoderSuperFrmCfg (5 fields, no maxReEncodeTimes).
 * T32: IMPEncoderSuperFrmCfg (6 fields, includes maxReEncodeTimes).
 */
int hal_enc_set_super_frame(void *ctx, int chn, const rss_super_frame_cfg_t *cfg)
{
    (void)ctx;
    if (!cfg)
        return RSS_ERR_INVAL;
#if defined(PLATFORM_T21) || defined(HAL_HYBRID_SDK)
    IMPEncoderSuperFrmCfg sfcfg;
    memset(&sfcfg, 0, sizeof(sfcfg));
    sfcfg.superFrmMode = (IMPEncoderSuperFrmMode)cfg->mode;
    sfcfg.superIFrmBitsThr = cfg->i_bits_thr;
    sfcfg.superPFrmBitsThr = cfg->p_bits_thr;
    sfcfg.superBFrmBitsThr = 0;
    sfcfg.rcPriority = (IMPEncoderRcPriority)cfg->priority;
#if defined(HAL_HYBRID_SDK)
    sfcfg.maxReEncodeTimes = cfg->max_reencode;
#endif
    int ret = IMP_Encoder_SetSuperFrameCfg(chn, &sfcfg);
    if (ret != 0)
        HAL_LOG_ERR("SetSuperFrameCfg(%d) failed: %d", chn, ret);
    return ret;
#else
    (void)chn;
    return RSS_ERR_NOTSUP;
#endif
}

/*
 * hal_enc_get_super_frame -- get super frame configuration.
 */
int hal_enc_get_super_frame(void *ctx, int chn, rss_super_frame_cfg_t *cfg)
{
    (void)ctx;
    if (!cfg)
        return RSS_ERR_INVAL;
#if defined(PLATFORM_T21) || defined(HAL_HYBRID_SDK)
    IMPEncoderSuperFrmCfg sfcfg;
    int ret = IMP_Encoder_GetSuperFrameCfg(chn, &sfcfg);
    if (ret != 0) {
        HAL_LOG_ERR("GetSuperFrameCfg(%d) failed: %d", chn, ret);
        return ret;
    }
    cfg->mode = (rss_super_frame_mode_t)sfcfg.superFrmMode;
    cfg->i_bits_thr = sfcfg.superIFrmBitsThr;
    cfg->p_bits_thr = sfcfg.superPFrmBitsThr;
    cfg->priority = (rss_rc_priority_t)sfcfg.rcPriority;
#if defined(HAL_HYBRID_SDK)
    cfg->max_reencode = sfcfg.maxReEncodeTimes;
#else
    cfg->max_reencode = 0;
#endif
    return RSS_OK;
#else
    (void)chn;
    return RSS_ERR_NOTSUP;
#endif
}

/*
 * hal_enc_set_color2grey -- enable/disable color-to-grey mode.
 *
 * T21 only: IMP_Encoder_SetChnColor2Grey.
 */
int hal_enc_set_color2grey(void *ctx, int chn, bool enable)
{
    (void)ctx;
#if defined(PLATFORM_T21)
    IMPEncoderColor2GreyCfg cfg;
    cfg.enable = enable;
    int ret = IMP_Encoder_SetChnColor2Grey(chn, &cfg);
    if (ret != 0)
        HAL_LOG_ERR("SetChnColor2Grey(%d, %d) failed: %d", chn, (int)enable, ret);
    return ret;
#else
    (void)chn;
    (void)enable;
    return RSS_ERR_NOTSUP;
#endif
}

/*
 * hal_enc_get_color2grey -- get color-to-grey state.
 */
int hal_enc_get_color2grey(void *ctx, int chn, bool *enable)
{
    (void)ctx;
    if (!enable)
        return RSS_ERR_INVAL;
#if defined(PLATFORM_T21)
    IMPEncoderColor2GreyCfg cfg;
    int ret = IMP_Encoder_GetChnColor2Grey(chn, &cfg);
    if (ret != 0) {
        HAL_LOG_ERR("GetChnColor2Grey(%d) failed: %d", chn, ret);
        return ret;
    }
    *enable = cfg.enable;
    return RSS_OK;
#else
    (void)chn;
    return RSS_ERR_NOTSUP;
#endif
}

/* ══════════════════════════════════════════════════════════════════════
 * 12. Phase 2 — Quality Improvement Features
 * ══════════════════════════════════════════════════════════════════════ */

/*
 * hal_enc_set_roi -- set a single ROI region.
 *
 * T21: IMP_Encoder_SetChnROI(chn, &roiCfg) — single region.
 * T32: IMP_Encoder_SetChnROI(chn, &roiAttr) — array of 16, so
 *      we Get the full array, modify one entry, and Set it back.
 */
int hal_enc_set_roi(void *ctx, int chn, const rss_enc_roi_t *roi)
{
    (void)ctx;
    if (!roi)
        return RSS_ERR_INVAL;

#if defined(PLATFORM_T21)
    IMPEncoderROICfg roiCfg;
    memset(&roiCfg, 0, sizeof(roiCfg));
    roiCfg.u32Index = roi->index;
    roiCfg.bEnable = roi->enable;
    roiCfg.bRelatedQp = roi->relative_qp;
    roiCfg.s32Qp = roi->qp;
    roiCfg.rect.p0.x = roi->x;
    roiCfg.rect.p0.y = roi->y;
    roiCfg.rect.p1.x = roi->x + roi->w;
    roiCfg.rect.p1.y = roi->y + roi->h;
    int ret = IMP_Encoder_SetChnROI(chn, &roiCfg);
    if (ret != 0)
        HAL_LOG_ERR("SetChnROI(%d, idx=%u) failed: %d", chn, roi->index, ret);
    return ret;

#elif defined(HAL_HYBRID_SDK)
    IMPEncoderROIAttr roiAttr;
    int ret = IMP_Encoder_GetChnROI(chn, &roiAttr);
    if (ret != 0) {
        HAL_LOG_ERR("GetChnROI(%d) failed: %d", chn, ret);
        return ret;
    }
    if (roi->index >= IMP_ENC_ROI_WIN_COUNT)
        return RSS_ERR_INVAL;

    IMPEncoderROICfg *r = &roiAttr.roi[roi->index];
    r->u32Index = roi->index;
    r->bEnable = roi->enable;
    r->bRelatedQp = roi->relative_qp;
    r->s32Qp = roi->qp;
    r->rect.p0.x = roi->x;
    r->rect.p0.y = roi->y;
    r->rect.p1.x = roi->x + roi->w;
    r->rect.p1.y = roi->y + roi->h;

    ret = IMP_Encoder_SetChnROI(chn, &roiAttr);
    if (ret != 0)
        HAL_LOG_ERR("SetChnROI(%d, idx=%u) failed: %d", chn, roi->index, ret);
    return ret;

#else
    (void)chn;
    return RSS_ERR_NOTSUP;
#endif
}

/*
 * hal_enc_get_roi -- get a single ROI region.
 */
int hal_enc_get_roi(void *ctx, int chn, uint32_t index, rss_enc_roi_t *roi)
{
    (void)ctx;
    if (!roi)
        return RSS_ERR_INVAL;

#if defined(PLATFORM_T21)
    IMPEncoderROICfg roiCfg;
    memset(&roiCfg, 0, sizeof(roiCfg));
    roiCfg.u32Index = index;
    int ret = IMP_Encoder_GetChnROI(chn, &roiCfg);
    if (ret != 0) {
        HAL_LOG_ERR("GetChnROI(%d, idx=%u) failed: %d", chn, index, ret);
        return ret;
    }
    roi->index = roiCfg.u32Index;
    roi->enable = roiCfg.bEnable;
    roi->relative_qp = roiCfg.bRelatedQp;
    roi->qp = roiCfg.s32Qp;
    roi->x = roiCfg.rect.p0.x;
    roi->y = roiCfg.rect.p0.y;
    roi->w = roiCfg.rect.p1.x - roiCfg.rect.p0.x;
    roi->h = roiCfg.rect.p1.y - roiCfg.rect.p0.y;
    return RSS_OK;

#elif defined(HAL_HYBRID_SDK)
    IMPEncoderROIAttr roiAttr;
    int ret = IMP_Encoder_GetChnROI(chn, &roiAttr);
    if (ret != 0) {
        HAL_LOG_ERR("GetChnROI(%d) failed: %d", chn, ret);
        return ret;
    }
    if (index >= IMP_ENC_ROI_WIN_COUNT)
        return RSS_ERR_INVAL;

    const IMPEncoderROICfg *r = &roiAttr.roi[index];
    roi->index = r->u32Index;
    roi->enable = r->bEnable;
    roi->relative_qp = r->bRelatedQp;
    roi->qp = r->s32Qp;
    roi->x = r->rect.p0.x;
    roi->y = r->rect.p0.y;
    roi->w = r->rect.p1.x - r->rect.p0.x;
    roi->h = r->rect.p1.y - r->rect.p0.y;
    return RSS_OK;

#else
    (void)chn;
    (void)index;
    return RSS_ERR_NOTSUP;
#endif
}

/*
 * hal_enc_set_map_roi -- set per-macroblock ROI quality map.
 *
 * T32 only: IMP_Encoder_SetChnMapRoi.
 */
int hal_enc_set_map_roi(void *ctx, int chn, const uint8_t *map, uint32_t map_size, int type)
{
    (void)ctx;
    if (!map)
        return RSS_ERR_INVAL;

#if defined(HAL_HYBRID_SDK)
    IMPEncoderMapRoiCfg mapCfg;
    IMPEncoderMappingList list;

    memset(&mapCfg, 0, sizeof(mapCfg));
    memset(&list, 0, sizeof(list));

    mapCfg.map = (uint8_t *)map;
    mapCfg.mapSize = map_size;
    mapCfg.type = (IMPEncoderMappingType)type;
    list.mapdata = (uint8_t *)map;
    list.length = map_size;

    int ret = IMP_Encoder_SetChnMapRoi(chn, &mapCfg, &list);
    if (ret != 0)
        HAL_LOG_ERR("SetChnMapRoi(%d) failed: %d", chn, ret);
    return ret;
#else
    (void)chn;
    (void)map_size;
    (void)type;
    return RSS_ERR_NOTSUP;
#endif
}

/*
 * hal_enc_set_qp_bounds_per_frame -- set separate I/P frame QP bounds.
 *
 * T32/T41: IMP_Encoder_SetChnQpBoundsPerFrame.
 */
int hal_enc_set_qp_bounds_per_frame(void *ctx, int chn, int min_i, int max_i, int min_p, int max_p)
{
    (void)ctx;
#if defined(HAL_HYBRID_SDK) || defined(PLATFORM_T41)
    int ret = IMP_Encoder_SetChnQpBoundsPerFrame(chn, min_i, max_i, min_p, max_p);
    if (ret != 0)
        HAL_LOG_ERR("SetChnQpBoundsPerFrame(%d) failed: %d", chn, ret);
    return ret;
#else
    (void)chn;
    (void)min_i;
    (void)max_i;
    (void)min_p;
    (void)max_p;
    return RSS_ERR_NOTSUP;
#endif
}

/*
 * hal_enc_set_qpg_mode -- set macroblock-level QP control mode.
 *
 * T21: IMPEncoderQpgMode enum (QPG_CLOSE..QPG_SASM_TAB).
 * T32: IMPEncoderQpgMode enum (MBQP_AUTO..MBQP_TEXT_ROWRC).
 * Values are platform-specific; callers should use caps to determine availability.
 */
int hal_enc_set_qpg_mode(void *ctx, int chn, int mode)
{
    (void)ctx;
#if defined(PLATFORM_T21) || defined(HAL_HYBRID_SDK)
    IMPEncoderQpgMode qpg = (IMPEncoderQpgMode)mode;
    int ret = IMP_Encoder_SetQpgMode(chn, &qpg);
    if (ret != 0)
        HAL_LOG_ERR("SetQpgMode(%d, %d) failed: %d", chn, mode, ret);
    return ret;
#else
    (void)chn;
    (void)mode;
    return RSS_ERR_NOTSUP;
#endif
}

/*
 * hal_enc_get_qpg_mode -- get current QP control mode.
 */
int hal_enc_get_qpg_mode(void *ctx, int chn, int *mode)
{
    (void)ctx;
    if (!mode)
        return RSS_ERR_INVAL;
#if defined(PLATFORM_T21) || defined(HAL_HYBRID_SDK)
    IMPEncoderQpgMode qpg;
    int ret = IMP_Encoder_GetQpgMode(chn, &qpg);
    if (ret != 0) {
        HAL_LOG_ERR("GetQpgMode(%d) failed: %d", chn, ret);
        return ret;
    }
    *mode = (int)qpg;
    return RSS_OK;
#else
    (void)chn;
    return RSS_ERR_NOTSUP;
#endif
}

/*
 * hal_enc_set_qpg_ai -- set AI-based quality map.
 *
 * T32 only: IMP_Encoder_SetChnQpgAI.
 */
int hal_enc_set_qpg_ai(void *ctx, int chn, const uint8_t *map, uint32_t w, uint32_t h, int mode,
                       int mark_level)
{
    (void)ctx;
    if (!map)
        return RSS_ERR_INVAL;

#if defined(HAL_HYBRID_SDK)
    IMPEncoderQpgAICfg aiCfg;
    memset(&aiCfg, 0, sizeof(aiCfg));
    aiCfg.map = (uint8_t *)map;
    aiCfg.width = w;
    aiCfg.height = h;
    aiCfg.foreBackMode = (uint8_t)mode;
    aiCfg.markLvl = (uint8_t)mark_level;

    int ret = IMP_Encoder_SetChnQpgAI(chn, &aiCfg);
    if (ret != 0)
        HAL_LOG_ERR("SetChnQpgAI(%d) failed: %d", chn, ret);
    return ret;
#else
    (void)chn;
    (void)w;
    (void)h;
    (void)mode;
    (void)mark_level;
    return RSS_ERR_NOTSUP;
#endif
}

/*
 * hal_enc_set_mbrc -- enable/disable macroblock-level rate control.
 *
 * T21 only: IMP_Encoder_SetMbRC.
 */
int hal_enc_set_mbrc(void *ctx, int chn, bool enable)
{
    (void)ctx;
#if defined(PLATFORM_T21)
    int ret = IMP_Encoder_SetMbRC(chn, enable ? 1 : 0);
    if (ret != 0)
        HAL_LOG_ERR("SetMbRC(%d, %d) failed: %d", chn, (int)enable, ret);
    return ret;
#else
    (void)chn;
    (void)enable;
    return RSS_ERR_NOTSUP;
#endif
}

/*
 * hal_enc_get_mbrc -- get macroblock-level rate control state.
 */
int hal_enc_get_mbrc(void *ctx, int chn, bool *enable)
{
    (void)ctx;
    if (!enable)
        return RSS_ERR_INVAL;
#if defined(PLATFORM_T21)
    int val = 0;
    int ret = IMP_Encoder_GetMbRC(chn, &val);
    if (ret != 0) {
        HAL_LOG_ERR("GetMbRC(%d) failed: %d", chn, ret);
        return ret;
    }
    *enable = (val != 0);
    return RSS_OK;
#else
    (void)chn;
    return RSS_ERR_NOTSUP;
#endif
}

/*
 * hal_enc_set_denoise -- configure encoder-level denoising.
 *
 * T21 only: IMP_Encoder_SetChnDenoise.
 */
int hal_enc_set_denoise(void *ctx, int chn, const rss_enc_denoise_cfg_t *cfg)
{
    (void)ctx;
    if (!cfg)
        return RSS_ERR_INVAL;
#if defined(PLATFORM_T21)
    IMPEncoderAttrDenoise dn;
    memset(&dn, 0, sizeof(dn));
    dn.enable = cfg->enable;
    dn.dnType = cfg->dn_type;
    dn.dnIQp = cfg->dn_i_qp;
    dn.dnPQp = cfg->dn_p_qp;
    int ret = IMP_Encoder_SetChnDenoise(chn, &dn);
    if (ret != 0)
        HAL_LOG_ERR("SetChnDenoise(%d) failed: %d", chn, ret);
    return ret;
#else
    (void)chn;
    return RSS_ERR_NOTSUP;
#endif
}

/*
 * hal_enc_get_denoise -- get encoder denoise configuration.
 */
int hal_enc_get_denoise(void *ctx, int chn, rss_enc_denoise_cfg_t *cfg)
{
    (void)ctx;
    if (!cfg)
        return RSS_ERR_INVAL;
#if defined(PLATFORM_T21)
    IMPEncoderAttrDenoise dn;
    int ret = IMP_Encoder_GetChnDenoise(chn, &dn);
    if (ret != 0) {
        HAL_LOG_ERR("GetChnDenoise(%d) failed: %d", chn, ret);
        return ret;
    }
    cfg->enable = dn.enable;
    cfg->dn_type = dn.dnType;
    cfg->dn_i_qp = dn.dnIQp;
    cfg->dn_p_qp = dn.dnPQp;
    return RSS_OK;
#else
    (void)chn;
    return RSS_ERR_NOTSUP;
#endif
}

/* ═════════════════���══════════════════════════��═════════════════════════
 * 13. Phase 3 — Error Recovery Features
 * ═════��═════════════════���══════════════════════════════════════════════ */

int hal_enc_set_gdr(void *ctx, int chn, const rss_enc_gdr_cfg_t *cfg)
{
    (void)ctx;
    if (!cfg)
        return RSS_ERR_INVAL;
#if defined(HAL_HYBRID_SDK)
    IMPEncoderGDRCfg gdr;
    memset(&gdr, 0, sizeof(gdr));
    gdr.enable = cfg->enable;
    gdr.gdrCycle = cfg->gdr_cycle;
    gdr.gdrFrames = cfg->gdr_frames;
    int ret = IMP_Encoder_SetGDRCfg(chn, &gdr);
    if (ret != 0)
        HAL_LOG_ERR("SetGDRCfg(%d) failed: %d", chn, ret);
    return ret;
#else
    (void)chn;
    return RSS_ERR_NOTSUP;
#endif
}

int hal_enc_get_gdr(void *ctx, int chn, rss_enc_gdr_cfg_t *cfg)
{
    (void)ctx;
    if (!cfg)
        return RSS_ERR_INVAL;
#if defined(HAL_HYBRID_SDK)
    IMPEncoderGDRCfg gdr;
    int ret = IMP_Encoder_GetGDRCfg(chn, &gdr);
    if (ret != 0) {
        HAL_LOG_ERR("GetGDRCfg(%d) failed: %d", chn, ret);
        return ret;
    }
    cfg->enable = gdr.enable;
    cfg->gdr_cycle = gdr.gdrCycle;
    cfg->gdr_frames = gdr.gdrFrames;
    return RSS_OK;
#else
    (void)chn;
    return RSS_ERR_NOTSUP;
#endif
}

int hal_enc_request_gdr(void *ctx, int chn, int gdr_frames)
{
    (void)ctx;
#if defined(HAL_HYBRID_SDK)
    int ret = IMP_Encoder_RequestGDR(chn, gdr_frames);
    if (ret != 0)
        HAL_LOG_ERR("RequestGDR(%d, %d) failed: %d", chn, gdr_frames, ret);
    return ret;
#else
    (void)chn;
    (void)gdr_frames;
    return RSS_ERR_NOTSUP;
#endif
}

int hal_enc_insert_userdata(void *ctx, int chn, const void *data, uint32_t len)
{
    (void)ctx;
    if (!data || len == 0)
        return RSS_ERR_INVAL;
#if defined(PLATFORM_T21) || defined(HAL_HYBRID_SDK)
    int ret = IMP_Encoder_InsertUserData(chn, (void *)data, len);
    if (ret != 0)
        HAL_LOG_ERR("InsertUserData(%d, %u) failed: %d", chn, len, ret);
    return ret;
#else
    (void)chn;
    return RSS_ERR_NOTSUP;
#endif
}

/* ═══════════════════════════════���═════════════════════════════���════════
 * 14. Phase 4 ��� Codec Compliance Features
 * ═══════════════════��══════════════════════════════��═══════════════════ */

int hal_enc_set_h264_vui(void *ctx, int chn, const void *vui)
{
    (void)ctx;
    if (!vui)
        return RSS_ERR_INVAL;
#if defined(HAL_HYBRID_SDK)
    int ret = IMP_Encoder_SetH264Vui(chn, (const IMPEncoderH264Vui *)vui);
    if (ret != 0)
        HAL_LOG_ERR("SetH264Vui(%d) failed: %d", chn, ret);
    return ret;
#else
    (void)chn;
    return RSS_ERR_NOTSUP;
#endif
}

int hal_enc_get_h264_vui(void *ctx, int chn, void *vui)
{
    (void)ctx;
    if (!vui)
        return RSS_ERR_INVAL;
#if defined(HAL_HYBRID_SDK)
    int ret = IMP_Encoder_GetH264Vui(chn, (IMPEncoderH264Vui *)vui);
    if (ret != 0)
        HAL_LOG_ERR("GetH264Vui(%d) failed: %d", chn, ret);
    return ret;
#else
    (void)chn;
    return RSS_ERR_NOTSUP;
#endif
}

int hal_enc_set_h265_vui(void *ctx, int chn, const void *vui)
{
    (void)ctx;
    if (!vui)
        return RSS_ERR_INVAL;
#if defined(HAL_HYBRID_SDK)
    int ret = IMP_Encoder_SetH265Vui(chn, (const IMPEncoderH265Vui *)vui);
    if (ret != 0)
        HAL_LOG_ERR("SetH265Vui(%d) failed: %d", chn, ret);
    return ret;
#else
    (void)chn;
    return RSS_ERR_NOTSUP;
#endif
}

int hal_enc_get_h265_vui(void *ctx, int chn, void *vui)
{
    (void)ctx;
    if (!vui)
        return RSS_ERR_INVAL;
#if defined(HAL_HYBRID_SDK)
    int ret = IMP_Encoder_GetH265Vui(chn, (IMPEncoderH265Vui *)vui);
    if (ret != 0)
        HAL_LOG_ERR("GetH265Vui(%d) failed: %d", chn, ret);
    return ret;
#else
    (void)chn;
    return RSS_ERR_NOTSUP;
#endif
}

int hal_enc_set_h264_trans(void *ctx, int chn, const rss_enc_h264_trans_t *cfg)
{
    (void)ctx;
    if (!cfg)
        return RSS_ERR_INVAL;
#if defined(PLATFORM_T21) || defined(HAL_HYBRID_SDK)
    IMPEncoderH264TransCfg trans;
    trans.chroma_qp_index_offset = cfg->chroma_qp_index_offset;
    int ret = IMP_Encoder_SetH264TransCfg(chn, &trans);
    if (ret != 0)
        HAL_LOG_ERR("SetH264TransCfg(%d) failed: %d", chn, ret);
    return ret;
#else
    (void)chn;
    return RSS_ERR_NOTSUP;
#endif
}

int hal_enc_get_h264_trans(void *ctx, int chn, rss_enc_h264_trans_t *cfg)
{
    (void)ctx;
    if (!cfg)
        return RSS_ERR_INVAL;
#if defined(PLATFORM_T21) || defined(HAL_HYBRID_SDK)
    IMPEncoderH264TransCfg trans;
    int ret = IMP_Encoder_GetH264TransCfg(chn, &trans);
    if (ret != 0) {
        HAL_LOG_ERR("GetH264TransCfg(%d) failed: %d", chn, ret);
        return ret;
    }
    cfg->chroma_qp_index_offset = trans.chroma_qp_index_offset;
    return RSS_OK;
#else
    (void)chn;
    return RSS_ERR_NOTSUP;
#endif
}

int hal_enc_set_h265_trans(void *ctx, int chn, const rss_enc_h265_trans_t *cfg)
{
    (void)ctx;
    if (!cfg)
        return RSS_ERR_INVAL;
#if defined(PLATFORM_T21) || defined(HAL_HYBRID_SDK)
    IMPEncoderH265TransCfg trans;
    trans.chroma_cr_qp_offset = cfg->chroma_cr_qp_offset;
    trans.chroma_cb_qp_offset = cfg->chroma_cb_qp_offset;
    int ret = IMP_Encoder_SetH265TransCfg(chn, &trans);
    if (ret != 0)
        HAL_LOG_ERR("SetH265TransCfg(%d) failed: %d", chn, ret);
    return ret;
#else
    (void)chn;
    return RSS_ERR_NOTSUP;
#endif
}

int hal_enc_get_h265_trans(void *ctx, int chn, rss_enc_h265_trans_t *cfg)
{
    (void)ctx;
    if (!cfg)
        return RSS_ERR_INVAL;
#if defined(PLATFORM_T21) || defined(HAL_HYBRID_SDK)
    IMPEncoderH265TransCfg trans;
    int ret = IMP_Encoder_GetH265TransCfg(chn, &trans);
    if (ret != 0) {
        HAL_LOG_ERR("GetH265TransCfg(%d) failed: %d", chn, ret);
        return ret;
    }
    cfg->chroma_cr_qp_offset = trans.chroma_cr_qp_offset;
    cfg->chroma_cb_qp_offset = trans.chroma_cb_qp_offset;
    return RSS_OK;
#else
    (void)chn;
    return RSS_ERR_NOTSUP;
#endif
}

/* ═══════════════════════════════���══════════════════════════════��═══════
 * 15. Phase 5 — Operational Features
 * ═══��═══════════���═════════════════════════════════════��════════════════ */

int hal_enc_set_crop(void *ctx, int chn, const rss_enc_crop_cfg_t *cfg)
{
    (void)ctx;
    if (!cfg)
        return RSS_ERR_INVAL;
#if defined(HAL_HYBRID_SDK)
    IMPEncoderCropCfg crop;
    memset(&crop, 0, sizeof(crop));
    crop.enable = cfg->enable;
    crop.x = cfg->x;
    crop.y = cfg->y;
    crop.w = cfg->w;
    crop.h = cfg->h;
    int ret = IMP_Encoder_SetChnCrop(chn, &crop);
    if (ret != 0)
        HAL_LOG_ERR("SetChnCrop(%d) failed: %d", chn, ret);
    return ret;
#else
    (void)chn;
    return RSS_ERR_NOTSUP;
#endif
}

int hal_enc_get_crop(void *ctx, int chn, rss_enc_crop_cfg_t *cfg)
{
    (void)ctx;
    if (!cfg)
        return RSS_ERR_INVAL;
#if defined(HAL_HYBRID_SDK)
    IMPEncoderCropCfg crop;
    int ret = IMP_Encoder_GetChnCrop(chn, &crop);
    if (ret != 0) {
        HAL_LOG_ERR("GetChnCrop(%d) failed: %d", chn, ret);
        return ret;
    }
    cfg->enable = crop.enable;
    cfg->x = crop.x;
    cfg->y = crop.y;
    cfg->w = crop.w;
    cfg->h = crop.h;
    return RSS_OK;
#else
    (void)chn;
    return RSS_ERR_NOTSUP;
#endif
}

int hal_enc_get_eval_info(void *ctx, int chn, void *info)
{
    (void)ctx;
    if (!info)
        return RSS_ERR_INVAL;
#if defined(PLATFORM_T31)
    return IMP_Encoder_GetChnEvalInfo(chn, info);
#else
    (void)chn;
    return RSS_ERR_NOTSUP;
#endif
}

/* Weak declaration — T41 headers declare PollingModuleStream but
 * some T41 libimp.so builds don't export it. Weak symbol avoids
 * link failure; runtime check returns NOTSUP if absent. */
#if defined(PLATFORM_T21) || defined(PLATFORM_T31) || defined(PLATFORM_T41)
__attribute__((weak)) int IMP_Encoder_PollingModuleStream(uint32_t *, uint32_t);
#endif

int hal_enc_poll_module_stream(void *ctx, uint32_t *chn_bitmap, uint32_t timeout_ms)
{
    (void)ctx;
    if (!chn_bitmap)
        return RSS_ERR_INVAL;
#if defined(PLATFORM_T21) || defined(PLATFORM_T31) || defined(PLATFORM_T41)
    if (!IMP_Encoder_PollingModuleStream)
        return RSS_ERR_NOTSUP;
    return IMP_Encoder_PollingModuleStream(chn_bitmap, timeout_ms);
#else
    (void)timeout_ms;
    return RSS_ERR_NOTSUP;
#endif
}

int hal_enc_set_resize_mode(void *ctx, int chn, int enable)
{
    (void)ctx;
#if defined(PLATFORM_T31) || defined(PLATFORM_T41)
    return IMP_Encoder_SetChnResizeMode(chn, enable);
#else
    (void)chn;
    (void)enable;
    return RSS_ERR_NOTSUP;
#endif
}

/* ======================================================================
 * 16. Phase 6 -- JPEG Features
 * ====================================================================== */

#if defined(HAL_OLD_SDK)
/* Standard JPEG quantization tables (ITU-T T.81, Annex K) */
static const uint8_t jpeg_luma_quantizer[64] = {
    16, 11, 10, 16, 24,  40,  51,  61,  12, 12, 14, 19, 26,  58,  60,  55,
    14, 13, 16, 24, 40,  57,  69,  56,  14, 17, 22, 29, 51,  87,  80,  62,
    18, 22, 37, 56, 68,  109, 103, 77,  24, 35, 55, 64, 81,  104, 113, 92,
    49, 64, 78, 87, 103, 121, 120, 101, 72, 92, 95, 98, 112, 100, 103, 99};

static const uint8_t jpeg_chroma_quantizer[64] = {
    17, 18, 24, 47, 99, 99, 99, 99, 18, 21, 26, 66, 99, 99, 99, 99, 24, 26, 56, 99, 99, 99,
    99, 99, 47, 66, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99};

/*
 * Generate JPEG quantization tables from quality factor (1-99).
 * Same algorithm as libjpeg / Ingenic sample-Encoder-jpeg.c MakeTables().
 * Higher quality = lower quantizer values = larger files.
 */
static void hal_jpeg_make_tables(int quality, uint8_t *lqt, uint8_t *cqt)
{
    int q = quality;
    if (q < 1)
        q = 1;
    if (q > 99)
        q = 99;
    int scale = (q < 50) ? (5000 / q) : (200 - q * 2);
    for (int i = 0; i < 64; i++) {
        int lq = (jpeg_luma_quantizer[i] * scale + 50) / 100;
        int cq = (jpeg_chroma_quantizer[i] * scale + 50) / 100;
        lqt[i] = (uint8_t)(lq < 1 ? 1 : (lq > 255 ? 255 : lq));
        cqt[i] = (uint8_t)(cq < 1 ? 1 : (cq > 255 ? 255 : cq));
    }
}
#endif /* HAL_OLD_SDK */

/* Apply JPEG quality via QL table on platforms that support SetJpegeQl.
 * quality: 1-99 (higher = better). No-op on T31 (no QL table API). */
static int hal_jpeg_set_quality(int chn, int quality)
{
#if defined(HAL_OLD_SDK)
    IMPEncoderJpegeQl jql;
    memset(&jql, 0, sizeof(jql));
    jql.user_ql_en = 1;
    hal_jpeg_make_tables(quality, &jql.qmem_table[0], &jql.qmem_table[64]);
    int ret = IMP_Encoder_SetJpegeQl(chn, &jql);
    if (ret != 0)
        HAL_LOG_ERR("SetJpegeQl(%d, q=%d) failed: %d", chn, quality, ret);
    return ret;
#else
    /* T31/T41: quality set via SetDefaultParam iInitialQP at init */
    (void)chn;
    (void)quality;
    return RSS_OK;
#endif
}

int hal_enc_set_jpeg_ql(void *ctx, int chn, const rss_enc_jpeg_ql_t *ql)
{
    (void)ctx;
    if (!ql)
        return RSS_ERR_INVAL;
#if defined(HAL_OLD_SDK)
    IMPEncoderJpegeQl jql;
    jql.user_ql_en = ql->user_table_en;
    memcpy(jql.qmem_table, ql->qmem_table, sizeof(jql.qmem_table));
    int ret = IMP_Encoder_SetJpegeQl(chn, &jql);
    if (ret != 0)
        HAL_LOG_ERR("SetJpegeQl(%d) failed: %d", chn, ret);
    return ret;
#else
    (void)chn;
    return RSS_ERR_NOTSUP;
#endif
}

int hal_enc_get_jpeg_ql(void *ctx, int chn, rss_enc_jpeg_ql_t *ql)
{
    (void)ctx;
    if (!ql)
        return RSS_ERR_INVAL;
#if defined(HAL_OLD_SDK)
    IMPEncoderJpegeQl jql;
    int ret = IMP_Encoder_GetJpegeQl(chn, &jql);
    if (ret != 0) {
        HAL_LOG_ERR("GetJpegeQl(%d) failed: %d", chn, ret);
        return ret;
    }
    ql->user_table_en = jql.user_ql_en;
    memcpy(ql->qmem_table, jql.qmem_table, sizeof(ql->qmem_table));
    return RSS_OK;
#else
    (void)chn;
    return RSS_ERR_NOTSUP;
#endif
}

int hal_enc_set_jpeg_qp(void *ctx, int chn, int qp)
{
    (void)ctx;
#if defined(HAL_HYBRID_SDK)
    int ret = IMP_Encoder_SetJpegQp(chn, qp);
    if (ret != 0)
        HAL_LOG_ERR("SetJpegQp(%d, %d) failed: %d", chn, qp, ret);
    return ret;
#else
    (void)chn;
    (void)qp;
    return RSS_ERR_NOTSUP;
#endif
}

int hal_enc_get_jpeg_qp(void *ctx, int chn, int *qp)
{
    (void)ctx;
    if (!qp)
        return RSS_ERR_INVAL;
#if defined(HAL_HYBRID_SDK)
    return IMP_Encoder_GetJpegQp(chn, qp);
#else
    (void)chn;
    return RSS_ERR_NOTSUP;
#endif
}
