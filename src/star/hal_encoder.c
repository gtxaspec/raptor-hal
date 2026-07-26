/*
 * star/hal_encoder.c -- Raptor HAL video encoder, SigmaStar MI backend
 *
 * Counterpart to src/hal_encoder.c (Ingenic IMP Encoder).
 *
 * The mapping is direct: raptor encoder channel N is MI VENC channel N,
 * fed by a VPE output port through MI_SYS_BindChnPort2. What differs
 * from IMP is worth stating up front, because it shapes the file:
 *
 *  - MI has no encoder *groups*. IMP's model is group -> registered
 *    channel -> bind; MI's is a plain module-to-module bind. So
 *    enc_create_group / enc_register_channel and their inverses are
 *    accepted and recorded but issue no MI call, and the real
 *    connection is made by hal_bind in hal_common.c. Returning
 *    RSS_ERR_NOTSUP instead would fail rvd's pipeline setup for a
 *    concept the hardware simply does not have.
 *
 *  - MI packs several NAL units into one stream *pack*. IMP hands back
 *    one NAL per pack, which is what rss_frame_t's nals[] was shaped
 *    for. See star_enc_fill_nals for how that is reconciled, and why
 *    the pack -- not its packetInfo entries -- is what becomes an
 *    rss_nal_unit_t.
 *
 *  - Rate control is set at channel creation and changed by
 *    read-modify-write on the channel attributes. MI has no per-knob
 *    setter for bitrate/gop/fps, so enc_set_bitrate and friends all
 *    funnel through star_enc_reconfigure_rate.
 *
 * Ops that are absent from the vtable in hal_common.c rather than
 * stubbed here: everything under RC options, ROI, GDR, p-skip, SRD,
 * denoise, crop, super-frame, colour-to-grey, entropy mode, buffer
 * pools and stream-SHM injection. MI either has no equivalent or
 * exposes it through MI_VENC_SetParam* structures that nothing in rvd
 * asks for on this SoC; RSS_HAL_CALL turns each into RSS_ERR_NOTSUP.
 *
 * Two of those absences are load-bearing and must stay absent:
 * enc_get_rmem_info and enc_inject_stream_shm. They are what enable
 * rvd's *refmode*, where the ring carries an (offset, length) reference
 * into the encoder's output memory instead of a copy, and the consumer
 * reads it after the HAL has already released the frame. That is safe
 * on Ingenic, whose encoder writes each frame into a distinct rmem
 * slot. It is not safe here: star_probe -c saw three consecutive frames
 * of 61634, 2582 and 4073 bytes all land at the same address
 * (0xb607c000 / phys 0x302c3000), so MI reuses the stream buffer as
 * soon as MI_VENC_ReleaseStream hands it back. A reference published
 * across that boundary reads whatever the next frame overwrote it with.
 *
 * Leaving both ops unimplemented is what keeps refmode off: rvd's
 * encoder thread falls back to embedded (copying) publication when
 * ref_base is zero, which is exactly what a NOTSUP enc_get_rmem_info
 * produces. Anyone implementing either one later has to solve the
 * buffer lifetime first -- by holding the frame until the consumer is
 * done, or by copying into a region MI does not recycle.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "star_state.h"

#include <sys/select.h>

/* ================================================================
 * CHANNEL LOOKUP
 * ================================================================ */

static star_venc_chn_t *star_enc_chn(void *ctx, int chn)
{
    star_state_t *st = star_state(ctx);

    if (!st || chn < 0 || chn >= I6_VENC_CHN_NUM)
        return NULL;

    return &st->enc[chn];
}

#define STAR_ENC_ENTER(ctx, chn, st_var, enc_var)                                                  \
    star_state_t *st_var = star_state(ctx);                                                        \
    star_venc_chn_t *enc_var = star_enc_chn(ctx, chn);                                             \
    if (!st_var || !enc_var)                                                                       \
        return RSS_ERR_INVAL;                                                                      \
    if (!enc_var->created)                                                                         \
    return RSS_ERR_NOENT

/* ================================================================
 * CONFIGURATION TRANSLATION
 * ================================================================ */

static i6_venc_codec star_enc_codec(rss_codec_t codec)
{
    switch (codec) {
    case RSS_CODEC_H264:
        return I6_VENC_CODEC_H264;
    case RSS_CODEC_H265:
        return I6_VENC_CODEC_H265;
    case RSS_CODEC_JPEG:
    case RSS_CODEC_MJPEG:
        return I6_VENC_CODEC_MJPG;
    default:
        return I6_VENC_CODEC_END;
    }
}

/*
 * star_enc_ratemode -- rss_rc_mode_t to the MI mode for a codec.
 *
 * MI names its rate modes per codec, so the codec picks the half of the
 * enum and the mode picks the entry. Returns I6_VENC_RATEMODE_END for
 * combinations the hardware refuses, which the caller reports rather
 * than silently substituting -- a stream quietly running CBR when the
 * operator configured VBR is a bug that only shows up as a bandwidth
 * bill.
 *
 * The two "capped" modes have no MI equivalent and map to AVBR, which
 * is the closest thing MI offers: a VBR whose average is steered
 * towards a target. RSS_RC_SMART likewise. That substitution *is*
 * silent, but it is the documented cross-SDK mapping in raptor_hal.h
 * ("old SDK; mapped to CAPPED_VBR on new" and vice versa) rather than
 * something invented here.
 */
static i6_venc_ratemode star_enc_ratemode(rss_codec_t codec, rss_rc_mode_t mode)
{
    bool h265 = (codec == RSS_CODEC_H265);

    if (codec == RSS_CODEC_JPEG || codec == RSS_CODEC_MJPEG) {
        switch (mode) {
        case RSS_RC_CBR:
            return I6_VENC_RATEMODE_MJPGCBR;
        case RSS_RC_FIXQP:
            return I6_VENC_RATEMODE_MJPGQP;
        default:
            return I6_VENC_RATEMODE_END;
        }
    }

    switch (mode) {
    case RSS_RC_FIXQP:
        return h265 ? I6_VENC_RATEMODE_H265QP : I6_VENC_RATEMODE_H264QP;
    case RSS_RC_CBR:
        return h265 ? I6_VENC_RATEMODE_H265CBR : I6_VENC_RATEMODE_H264CBR;
    case RSS_RC_VBR:
        return h265 ? I6_VENC_RATEMODE_H265VBR : I6_VENC_RATEMODE_H264VBR;
    case RSS_RC_SMART:
    case RSS_RC_CAPPED_VBR:
    case RSS_RC_CAPPED_QUALITY:
        return h265 ? I6_VENC_RATEMODE_H265AVBR : I6_VENC_RATEMODE_H264AVBR;
    default:
        return I6_VENC_RATEMODE_END;
    }
}

/*
 * QP bounds for the quality-driven modes.
 *
 * rss_video_config_t carries min_qp/max_qp as int16_t with -1 meaning
 * "SDK default". MI has no such sentinel: whatever is in the struct is
 * programmed. So substitute the H.26x defaults divinus uses when the
 * caller declined to choose.
 */
#define STAR_ENC_QP_MIN_DEFAULT 20
#define STAR_ENC_QP_MAX_DEFAULT 45

static unsigned int star_enc_qp(int16_t qp, unsigned int fallback)
{
    return (qp < 0 || qp > 51) ? fallback : (unsigned int)qp;
}

/*
 * star_enc_fill_rate -- populate the rate half of an i6_venc_chn.
 *
 * Bitrates cross this boundary in bits per second (rss_video_config_t)
 * and land in MI as bits per second too. divinus multiplies by 1024
 * only because its own config layer carries kbps; there is no unit
 * change here, which is worth stating because getting it wrong by 2^10
 * produces a stream that looks plausible and is unusably wrong.
 */
static int star_enc_fill_rate(i6_venc_rate *rate, rss_codec_t codec,
                              const rss_video_config_t *cfg)
{
    i6_venc_ratemode mode = star_enc_ratemode(codec, cfg->rc_mode);
    unsigned int fps_num = cfg->fps_num ? cfg->fps_num : 30;
    unsigned int fps_den = cfg->fps_den ? cfg->fps_den : 1;
    unsigned int gop = cfg->gop_length ? cfg->gop_length : fps_num / fps_den * 2;
    unsigned int max_qp = star_enc_qp(cfg->max_qp, STAR_ENC_QP_MAX_DEFAULT);
    unsigned int min_qp = star_enc_qp(cfg->min_qp, STAR_ENC_QP_MIN_DEFAULT);
    unsigned int max_bitrate = cfg->max_bitrate ? cfg->max_bitrate : cfg->bitrate;

    if (mode == I6_VENC_RATEMODE_END) {
        HAL_LOG_ERR("venc: rate mode %d is not available for codec %d", cfg->rc_mode, codec);
        return RSS_ERR_NOTSUP;
    }

    memset(rate, 0, sizeof(*rate));
    rate->mode = mode;

    /*
     * The H.264 and H.265 arms below write through the h264* union
     * members for both codecs. That is not a copy-paste slip: MI
     * declares h264Cbr and h265Cbr as the same type (and likewise Vbr,
     * Avbr and Qp), so they are the same bytes of the same union, and
     * `mode` is what tells the encoder which codec it is configuring.
     * Only ABR is genuinely H.264-only, and it has its own arm.
     */
    switch (mode) {
    case I6_VENC_RATEMODE_MJPGCBR:
        rate->mjpgCbr.bitrate = cfg->bitrate;
        rate->mjpgCbr.fpsNum = fps_num;
        rate->mjpgCbr.fpsDen = fps_den;
        break;
    case I6_VENC_RATEMODE_MJPGQP:
        rate->mjpgQp.fpsNum = fps_num;
        rate->mjpgQp.fpsDen = fps_den;
        rate->mjpgQp.quality = max_qp;
        break;
    case I6_VENC_RATEMODE_H264CBR:
    case I6_VENC_RATEMODE_H265CBR:
        /* statTime is the rate-control averaging window in seconds and
         * avgLvl the smoothing level; 1/1 is what both references use. */
        rate->h264Cbr.gop = gop;
        rate->h264Cbr.statTime = 1;
        rate->h264Cbr.fpsNum = fps_num;
        rate->h264Cbr.fpsDen = fps_den;
        rate->h264Cbr.bitrate = cfg->bitrate;
        rate->h264Cbr.avgLvl = 1;
        break;
    case I6_VENC_RATEMODE_H264VBR:
    case I6_VENC_RATEMODE_H265VBR:
    case I6_VENC_RATEMODE_H264AVBR:
    case I6_VENC_RATEMODE_H265AVBR:
        rate->h264Vbr.gop = gop;
        rate->h264Vbr.statTime = 1;
        rate->h264Vbr.fpsNum = fps_num;
        rate->h264Vbr.fpsDen = fps_den;
        rate->h264Vbr.maxBitrate = max_bitrate;
        rate->h264Vbr.maxQual = max_qp;
        rate->h264Vbr.minQual = min_qp;
        break;
    case I6_VENC_RATEMODE_H264QP:
    case I6_VENC_RATEMODE_H265QP:
        /* interQual is the I-frame QP, predQual the P-frame QP. init_qp
         * applies to both when the caller gave one. */
        rate->h264Qp.gop = gop;
        rate->h264Qp.fpsNum = fps_num;
        rate->h264Qp.fpsDen = fps_den;
        rate->h264Qp.interQual = star_enc_qp(cfg->init_qp, max_qp);
        rate->h264Qp.predQual = star_enc_qp(cfg->init_qp, min_qp);
        break;
    case I6_VENC_RATEMODE_H264ABR:
        rate->h264Abr.gop = gop;
        rate->h264Abr.statTime = 1;
        rate->h264Abr.fpsNum = fps_num;
        rate->h264Abr.fpsDen = fps_den;
        rate->h264Abr.avgBitrate = cfg->bitrate;
        rate->h264Abr.maxBitrate = max_bitrate;
        break;
    default:
        return RSS_ERR_NOTSUP;
    }

    return RSS_OK;
}

/*
 * star_enc_fill_attrib -- populate the codec half of an i6_venc_chn.
 *
 * bufSize is the encoder's output buffer: width * height, as both
 * references size it. That is 1.5x the compressed worst case for
 * H.264 at these resolutions and MI rejects visibly smaller values.
 *
 * profile is clamped rather than rejected. MI's H.264 accepts 0..2
 * (baseline/main/high) and H.265 only 0..1, so a config asking for
 * "high" H.265 gets main instead of a failed channel -- the profile is
 * a quality/compatibility preference, not a correctness requirement.
 */
static int star_enc_fill_attrib(i6_venc_attrib *attrib, rss_codec_t codec,
                                const rss_video_config_t *cfg)
{
    i6_venc_codec mi_codec = star_enc_codec(codec);
    i6_venc_attr_h26x *h26x;

    if (mi_codec == I6_VENC_CODEC_END) {
        HAL_LOG_ERR("venc: codec %d is not supported by the hardware", codec);
        return RSS_ERR_NOTSUP;
    }

    memset(attrib, 0, sizeof(*attrib));
    attrib->codec = mi_codec;

    if (mi_codec == I6_VENC_CODEC_MJPG) {
        attrib->mjpg.maxWidth = cfg->width;
        attrib->mjpg.maxHeight = cfg->height;
        attrib->mjpg.bufSize = (unsigned int)cfg->width * cfg->height;
        attrib->mjpg.byFrame = 1;
        attrib->mjpg.width = cfg->width;
        attrib->mjpg.height = cfg->height;
        attrib->mjpg.dcfThumbs = 0;
        attrib->mjpg.markPerRow = 0;
        return RSS_OK;
    }

    h26x = (mi_codec == I6_VENC_CODEC_H265) ? &attrib->h265 : &attrib->h264;
    h26x->maxWidth = cfg->width;
    h26x->maxHeight = cfg->height;
    h26x->bufSize = (unsigned int)cfg->width * cfg->height;
    h26x->profile = (unsigned int)cfg->profile;
    if (mi_codec == I6_VENC_CODEC_H265 && h26x->profile > 1)
        h26x->profile = 1;
    else if (h26x->profile > 2)
        h26x->profile = 2;
    h26x->byFrame = 1;
    h26x->width = cfg->width;
    h26x->height = cfg->height;
    /* No B-frames and a single reference: the low-latency shape rvd
     * expects, and the only one divinus configures. */
    h26x->bFrameNum = 0;
    h26x->refNum = 1;

    return RSS_OK;
}

/* ================================================================
 * NAL TRANSLATION
 * ================================================================ */

static rss_nal_type_t star_enc_nal_h264(i6_venc_nalu_h264 type)
{
    switch (type) {
    case I6_VENC_NALU_H264_PSLICE:
        return RSS_NAL_H264_SLICE;
    case I6_VENC_NALU_H264_ISLICE:
        return RSS_NAL_H264_IDR;
    case I6_VENC_NALU_H264_SEI:
        return RSS_NAL_H264_SEI;
    case I6_VENC_NALU_H264_SPS:
        return RSS_NAL_H264_SPS;
    case I6_VENC_NALU_H264_PPS:
        return RSS_NAL_H264_PPS;
    case I6_VENC_NALU_H264_IPSLICE:
        return RSS_NAL_H264_SLICE;
    default:
        return RSS_NAL_UNKNOWN;
    }
}

static rss_nal_type_t star_enc_nal_h265(i6_venc_nalu_h265 type)
{
    switch (type) {
    case I6_VENC_NALU_H265_PSLICE:
        return RSS_NAL_H265_SLICE;
    case I6_VENC_NALU_H265_ISLICE:
        return RSS_NAL_H265_IDR;
    case I6_VENC_NALU_H265_VPS:
        return RSS_NAL_H265_VPS;
    case I6_VENC_NALU_H265_SPS:
        return RSS_NAL_H265_SPS;
    case I6_VENC_NALU_H265_PPS:
        return RSS_NAL_H265_PPS;
    case I6_VENC_NALU_H265_SEI:
        return RSS_NAL_H265_SEI;
    default:
        return RSS_NAL_UNKNOWN;
    }
}

static rss_nal_type_t star_enc_nal_type(rss_codec_t codec, i6_venc_nalu nalu)
{
    switch (codec) {
    case RSS_CODEC_H264:
        return star_enc_nal_h264(nalu.h264Nalu);
    case RSS_CODEC_H265:
        return star_enc_nal_h265(nalu.h265Nalu);
    default:
        return RSS_NAL_JPEG_FRAME;
    }
}

static bool star_enc_nal_is_key(rss_nal_type_t type)
{
    return type == RSS_NAL_H264_IDR || type == RSS_NAL_H265_IDR ||
           type == RSS_NAL_JPEG_FRAME;
}

/*
 * star_enc_fill_nals -- one MI pack becomes one rss_nal_unit_t.
 *
 * This is the one place the two encoder models genuinely disagree, so
 * the reasoning matters.
 *
 * IMP emits one NAL per pack and rss_nal_unit_t was shaped for that.
 * MI emits one pack per frame containing several NAL units, described
 * by pack->packetInfo[0..packNum-1] as {type, offset, length}.
 *
 * The addresses here come only from documented fields -- the vendor
 * sample's own expression, pu8Addr + u32Offset for u32Len - u32Offset
 * bytes -- and packetInfo is used exclusively to *type* the pack:
 * scanned for a slice NAL to report as the pack's type and to decide
 * is_key. Nothing downstream needs finer granularity, because rvd
 * publishes a frame's NALs as one concatenated byte run into the ring
 * and rsd's Annex-B transport re-splits on start codes anyway.
 *
 * star_probe -c measured what the vendor reference leaves unstated, on
 * an SSC30KQ:
 *
 *   - packetInfo offsets are relative to the start of the pack's valid
 *     data and tile it exactly -- an IDR pack of 61634 bytes came back
 *     as SPS at 0 (24 bytes), PPS at 24 (8), IDR at 32 (61602), each
 *     landing on an Annex-B start code. So per-NAL addressing is
 *     available to a later phase that wants it; divinus's reading is
 *     the correct one. (pack->offset was 0 on every pack, so "relative
 *     to pu8Addr" and "relative to pu8Addr + u32Offset" coincide and
 *     the run cannot separate them. They agree here, which is what
 *     matters.)
 *
 *   - pack->naluType is the pack's *primary* NAL type, not its first:
 *     that SPS+PPS+IDR pack reported ISLICE, not SPS. The refinement
 *     loop below therefore agrees with pack->naluType rather than
 *     correcting it -- it is kept because it costs nothing and does not
 *     depend on that behaviour holding for every frame shape.
 */
static void star_enc_fill_nals(star_venc_chn_t *enc, rss_frame_t *frame)
{
    unsigned int count = enc->strm.count;
    unsigned int i;

    /*
     * The pack array itself is always big enough (star_enc_packs), but
     * nals[] is fixed, so an unusually fragmented frame reports its
     * first STAR_VENC_MAX_PACKS packs. That matches rvd's own ceiling,
     * so nothing downstream loses anything it would have kept.
     */
    if (count > STAR_VENC_MAX_PACKS) {
        HAL_LOG_WARN("venc: %u packs in one frame, reporting %d", count, STAR_VENC_MAX_PACKS);
        count = STAR_VENC_MAX_PACKS;
    }

    frame->is_key = false;

    for (i = 0; i < count; i++) {
        i6_venc_pack *pack = &enc->strm.packet[i];
        rss_nal_unit_t *nal = &enc->nals[i];
        rss_nal_type_t type = star_enc_nal_type(enc->codec, pack->naluType);
        unsigned int offset = pack->offset;
        unsigned int k;

        /* A pack whose offset exceeds its length would make the length
         * arithmetic wrap. Treat it as "no offset" and keep the frame. */
        if (offset > pack->length) {
            HAL_LOG_WARN("venc: pack offset %u exceeds length %u, ignoring offset", offset,
                         pack->length);
            offset = 0;
        }

        nal->data = (const uint8_t *)pack->data + offset;
        nal->length = pack->length - offset;
        nal->frame_end = pack->endFrame ? true : false;
        nal->type = type;

        /*
         * Refine the type from the contained NAL units. A pack that
         * starts with SPS still *is* the keyframe, and reporting it as
         * SPS would have rvd's primary_nal_type fall through to a
         * parameter-set type for every IDR.
         */
        for (k = 0; k < pack->packNum && k < 8; k++) {
            rss_nal_type_t sub = star_enc_nal_type(enc->codec, pack->packetInfo[k].packType);

            if (star_enc_nal_is_key(sub))
                frame->is_key = true;
            if (star_enc_nal_is_key(sub) || sub == RSS_NAL_H264_SLICE ||
                sub == RSS_NAL_H265_SLICE)
                nal->type = sub;
        }

        if (star_enc_nal_is_key(type))
            frame->is_key = true;
    }

    frame->nals = enc->nals;
    frame->nal_count = count;
}

/* ================================================================
 * CHANNEL LIFECYCLE
 * ================================================================ */

/*
 * MI has no encoder groups. IMP's group/register/bind triple collapses
 * to a single MI_SYS bind, made by hal_bind, so these four ops exist
 * only to keep rvd's pipeline setup on its normal path.
 */
int hal_enc_create_group(void *ctx, int grp)
{
    star_state_t *st = star_state(ctx);

    if (!st || grp < 0 || grp >= I6_VENC_CHN_NUM)
        return RSS_ERR_INVAL;

    return RSS_OK;
}

int hal_enc_destroy_group(void *ctx, int grp)
{
    return hal_enc_create_group(ctx, grp);
}

int hal_enc_register_channel(void *ctx, int grp, int chn)
{
    star_venc_chn_t *enc = star_enc_chn(ctx, chn);

    if (!enc || grp < 0 || grp >= I6_VENC_CHN_NUM)
        return RSS_ERR_INVAL;
    if (!enc->created)
        return RSS_ERR_NOENT;

    return RSS_OK;
}

int hal_enc_unregister_channel(void *ctx, int chn)
{
    star_venc_chn_t *enc = star_enc_chn(ctx, chn);

    return enc ? RSS_OK : RSS_ERR_INVAL;
}

int hal_enc_create_channel(void *ctx, int chn, const rss_video_config_t *cfg)
{
    i6_venc_chn channel;
    unsigned int device = 0;
    int ret;

    star_state_t *st = star_state(ctx);
    star_venc_chn_t *enc = star_enc_chn(ctx, chn);

    if (!st || !enc || !cfg)
        return RSS_ERR_INVAL;
    if (!cfg->width || !cfg->height)
        return RSS_ERR_INVAL;
    if (enc->created) {
        HAL_LOG_ERR("venc chn %d: already created", chn);
        return RSS_ERR_BUSY;
    }

    memset(&channel, 0, sizeof(channel));

    ret = star_enc_fill_attrib(&channel.attrib, cfg->codec, cfg);
    if (ret)
        return ret;

    ret = star_enc_fill_rate(&channel.rate, cfg->codec, cfg);
    if (ret)
        return ret;

    ret = st->venc.fnCreateChannel(chn, &channel);
    if (ret) {
        HAL_LOG_ERR("MI_VENC_CreateChn(%d) failed: %d", chn, ret);
        return RSS_ERR_IO;
    }

    /*
     * The device id is only needed for the bind, but read it now: it
     * cannot be queried once the channel is destroyed, and teardown
     * unbinds after destroying nothing else has a use for it.
     */
    ret = st->venc.fnGetChannelDeviceId(chn, &device);
    if (ret) {
        HAL_LOG_WARN("MI_VENC_GetChnDevid(%d) failed: %d, assuming device 0", chn, ret);
        device = 0;
    }

    enc->created = true;
    enc->device = device;
    enc->src_port = -1;
    enc->fd = -1;
    enc->codec = cfg->codec;
    enc->width = cfg->width;
    enc->height = cfg->height;
    enc->fps_num = cfg->fps_num ? cfg->fps_num : 30;
    enc->fps_den = cfg->fps_den ? cfg->fps_den : 1;
    enc->gop = cfg->gop_length;
    enc->rc_mode = cfg->rc_mode;
    enc->bitrate = cfg->bitrate;
    enc->max_bitrate = cfg->max_bitrate;
    enc->init_qp = cfg->init_qp;
    enc->min_qp = cfg->min_qp;
    enc->max_qp = cfg->max_qp;

    HAL_LOG_INFO("venc chn %d up: %ux%u codec %d rc %d %u bps (device %u)", chn, cfg->width,
                 cfg->height, cfg->codec, cfg->rc_mode, cfg->bitrate, device);

    return RSS_OK;
}

/*
 * star_enc_packs -- a zeroed pack array of at least `count` entries.
 *
 * MI_VENC_GetStream writes into an array the caller sizes, and the
 * vendor reference never says whether it honours that size or copies
 * u32CurPacks entries regardless -- divinus mallocs to the queried count
 * rather than find out. So do the same: the inline array covers the
 * normal case with no allocation, and anything larger gets a heap array
 * sized to the full count. Under-sizing this is a buffer overflow in the
 * streaming path, which is not a thing to leave to a documentation
 * reading.
 */
static i6_venc_pack *star_enc_packs(star_venc_chn_t *enc, unsigned int count)
{
    if (count <= STAR_VENC_MAX_PACKS) {
        memset(enc->packs, 0, sizeof(enc->packs));
        return enc->packs;
    }

    if (count > enc->heap_count) {
        i6_venc_pack *grown = (i6_venc_pack *)realloc(enc->heap_packs, count * sizeof(*grown));

        if (!grown) {
            HAL_LOG_ERR("venc: cannot size a %u-pack array", count);
            return NULL;
        }
        enc->heap_packs = grown;
        enc->heap_count = count;
    }

    memset(enc->heap_packs, 0, enc->heap_count * sizeof(*enc->heap_packs));

    return enc->heap_packs;
}

/* Release a held stream without reporting -- used by teardown paths
 * where there is no caller left to tell. */
static void star_enc_drop_frame(star_state_t *st, int chn, star_venc_chn_t *enc)
{
    if (!enc->frame_held)
        return;

    st->venc.fnFreeStream(chn, &enc->strm);
    enc->frame_held = false;
    memset(&enc->strm, 0, sizeof(enc->strm));
}

static void star_enc_close_fd(star_state_t *st, int chn, star_venc_chn_t *enc)
{
    if (enc->fd < 0)
        return;

    st->venc.fnFreeDescriptor(chn);
    enc->fd = -1;
}

int hal_enc_destroy_channel(void *ctx, int chn)
{
    int ret;

    STAR_ENC_ENTER(ctx, chn, st, enc);

    star_enc_drop_frame(st, chn, enc);
    star_enc_close_fd(st, chn, enc);

    if (enc->receiving) {
        ret = st->venc.fnStopReceiving(chn);
        if (ret)
            HAL_LOG_WARN("MI_VENC_StopRecvPic(%d) failed: %d", chn, ret);
        enc->receiving = false;
    }

    /*
     * Unbind before destroy. divinus does the same (i6_video_destroy
     * unbinds, then destroys, then disables the VPE port): a channel
     * destroyed with a live bind leaves MI's kernel side referencing a
     * gone destination.
     */
    if (enc->bound && enc->src_port >= 0)
        star_enc_unbind_port(st, enc->src_port, chn);

    ret = st->venc.fnDestroyChannel(chn);
    if (ret)
        HAL_LOG_WARN("MI_VENC_DestroyChn(%d) failed: %d", chn, ret);

    /* Before the memset, which would otherwise lose the pointer. */
    free(enc->heap_packs);

    memset(enc, 0, sizeof(*enc));
    enc->fd = -1;
    enc->src_port = -1;

    return ret ? RSS_ERR_IO : RSS_OK;
}

/* ================================================================
 * BIND
 *
 * Called from hal_common.c's bind/unbind, which is where rvd's
 * FS -> ENC chain lands. The encoder side owns this because the bind
 * needs the VENC device id cached at channel creation.
 * ================================================================ */

static void star_enc_bind_cells(star_state_t *st, int port, int chn, i6_sys_bind *source,
                                i6_sys_bind *dest)
{
    memset(source, 0, sizeof(*source));
    source->module = I6_SYS_MOD_VPE;
    source->device = STAR_VPE_DEV;
    source->channel = STAR_VPE_CHN;
    source->port = port;

    memset(dest, 0, sizeof(*dest));
    dest->module = I6_SYS_MOD_VENC;
    dest->device = st->enc[chn].device;
    dest->channel = chn;
    dest->port = STAR_VENC_PORT;
}

int star_enc_bind_port(star_state_t *st, int port, int chn)
{
    i6_sys_bind source, dest;
    star_venc_chn_t *enc;
    unsigned int fps;
    int ret;

    if (!st || port < 0 || port >= STAR_VPE_PORT_NUM || chn < 0 || chn >= I6_VENC_CHN_NUM)
        return RSS_ERR_INVAL;

    enc = &st->enc[chn];
    if (!enc->created) {
        HAL_LOG_ERR("venc chn %d: bind before create", chn);
        return RSS_ERR_NOENT;
    }
    if (enc->bound)
        return enc->src_port == port ? RSS_OK : RSS_ERR_BUSY;

    /*
     * divinus enables the VPE port as part of binding (i6_channel_bind),
     * and rvd does not enable it until rvd_stream_start -- after the
     * bind. Enable it here so the two orders agree; hal_fs_enable_channel
     * is idempotent, so rvd's later call still does the right thing.
     */
    if (!st->port[port].enabled && st->port[port].configured) {
        ret = st->vpe.fnEnablePort(STAR_VPE_CHN, port);
        if (ret) {
            HAL_LOG_ERR("MI_VPE_EnablePort(%d, %d) failed: %d", STAR_VPE_CHN, port, ret);
            return RSS_ERR_IO;
        }
        st->port[port].enabled = true;
    }

    /*
     * FRAMEBASE, not REALTIME: the VIF->VPE link is realtime because
     * the ISP consumes pixels as they arrive, but VENC reads whole
     * frames out of DRAM. A realtime link here would hand the encoder
     * MI_SYS_REALTIME_MAGIC_PADDR instead of a frame.
     */
    fps = st->port[port].fps_num && st->port[port].fps_den
              ? st->port[port].fps_num / st->port[port].fps_den
              : st->fps;
    if (!fps)
        fps = enc->fps_num / (enc->fps_den ? enc->fps_den : 1);

    star_enc_bind_cells(st, port, chn, &source, &dest);
    ret = st->sys.fnBindExt(&source, &dest, fps, fps, I6_SYS_LINK_FRAMEBASE, 0);
    if (ret) {
        HAL_LOG_ERR("MI_SYS_BindChnPort2 VPE port %d -> VENC %d failed: %d", port, chn, ret);
        return RSS_ERR_IO;
    }

    enc->bound = true;
    enc->src_port = port;

    HAL_LOG_INFO("bind: VPE port %d -> VENC chn %d, framebase, %u fps", port, chn, fps);

    return RSS_OK;
}

int star_enc_unbind_port(star_state_t *st, int port, int chn)
{
    i6_sys_bind source, dest;
    star_venc_chn_t *enc;
    int ret;

    if (!st || port < 0 || port >= STAR_VPE_PORT_NUM || chn < 0 || chn >= I6_VENC_CHN_NUM)
        return RSS_ERR_INVAL;

    enc = &st->enc[chn];
    if (!enc->bound)
        return RSS_OK;

    star_enc_bind_cells(st, port, chn, &source, &dest);
    ret = st->sys.fnUnbind(&source, &dest);
    if (ret)
        HAL_LOG_WARN("MI_SYS_UnBindChnPort VPE port %d -> VENC %d failed: %d", port, chn, ret);

    enc->bound = false;
    enc->src_port = -1;

    return ret ? RSS_ERR_IO : RSS_OK;
}

void star_enc_release_all(star_state_t *st)
{
    int i;

    if (!st)
        return;

    for (i = 0; i < I6_VENC_CHN_NUM; i++) {
        star_venc_chn_t *enc = &st->enc[i];

        if (!enc->created)
            continue;

        star_enc_drop_frame(st, i, enc);
        star_enc_close_fd(st, i, enc);

        if (enc->receiving) {
            st->venc.fnStopReceiving(i);
            enc->receiving = false;
        }
        if (enc->bound && enc->src_port >= 0)
            star_enc_unbind_port(st, enc->src_port, i);

        st->venc.fnDestroyChannel(i);
        free(enc->heap_packs);
        memset(enc, 0, sizeof(*enc));
        enc->fd = -1;
        enc->src_port = -1;
    }
}

/* ================================================================
 * START / STOP
 * ================================================================ */

int hal_enc_start(void *ctx, int chn)
{
    int ret;

    STAR_ENC_ENTER(ctx, chn, st, enc);

    if (enc->receiving)
        return RSS_OK;

    ret = st->venc.fnStartReceiving(chn);
    if (ret) {
        HAL_LOG_ERR("MI_VENC_StartRecvPic(%d) failed: %d", chn, ret);
        return RSS_ERR_IO;
    }
    enc->receiving = true;

    return RSS_OK;
}

int hal_enc_stop(void *ctx, int chn)
{
    int ret;

    STAR_ENC_ENTER(ctx, chn, st, enc);

    if (!enc->receiving)
        return RSS_OK;

    /* Whatever is checked out belongs to a buffer the encoder is about
     * to stop refilling; hand it back first. */
    star_enc_drop_frame(st, chn, enc);

    ret = st->venc.fnStopReceiving(chn);
    enc->receiving = false;
    if (ret) {
        HAL_LOG_ERR("MI_VENC_StopRecvPic(%d) failed: %d", chn, ret);
        return RSS_ERR_IO;
    }

    return RSS_OK;
}

int hal_enc_request_idr(void *ctx, int chn)
{
    int ret;

    STAR_ENC_ENTER(ctx, chn, st, enc);

    /* instant = 1: insert the IDR at the next frame rather than at the
     * next GOP boundary, which is what a client joining a stream wants. */
    ret = st->venc.fnRequestIdr(chn, 1);
    if (ret) {
        HAL_LOG_ERR("MI_VENC_RequestIdr(%d) failed: %d", chn, ret);
        return RSS_ERR_IO;
    }

    return RSS_OK;
}

/* ================================================================
 * STREAM FETCH
 * ================================================================ */

int hal_enc_get_fd(void *ctx, int chn)
{
    int fd;

    STAR_ENC_ENTER(ctx, chn, st, enc);

    if (enc->fd >= 0)
        return enc->fd;

    fd = st->venc.fnGetDescriptor(chn);
    if (fd < 0) {
        HAL_LOG_ERR("MI_VENC_GetFd(%d) failed: %d", chn, fd);
        return RSS_ERR_IO;
    }
    enc->fd = fd;

    return fd;
}

/*
 * hal_enc_poll -- block until the channel has a frame.
 *
 * MI_VENC_GetStream's own timeout argument only covers the copy once a
 * frame exists; select on MI_VENC_GetFd is the vendor's documented wait
 * and is what divinus's video thread uses. The descriptor is cached
 * because MI_VENC_GetFd allocates one per call.
 */
int hal_enc_poll(void *ctx, int chn, uint32_t timeout_ms)
{
    struct timeval tv;
    fd_set fds;
    int fd, ret;

    STAR_ENC_ENTER(ctx, chn, st, enc);

    fd = hal_enc_get_fd(ctx, chn);
    if (fd < 0)
        return fd;

    FD_ZERO(&fds);
    FD_SET(fd, &fds);
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    ret = select(fd + 1, &fds, NULL, NULL, &tv);
    if (ret < 0)
        return RSS_ERR_IO;
    if (ret == 0)
        return RSS_ERR_TIMEOUT;

    return RSS_OK;
}

/*
 * hal_enc_get_frame -- check out one encoded frame.
 *
 * MI_VENC_GetStream needs the pack count filled in before the call --
 * it copies into an array the caller sizes -- so MI_VENC_Query comes
 * first. An empty frame (curPacks == 0) is normal: the descriptor
 * signals readiness slightly ahead of the pack being complete, and
 * divinus skips those too. Report it as -EAGAIN, the same "no frame
 * this time" the Ingenic backend returns and the only value rvd's
 * encoder thread treats as non-fatal.
 */
int hal_enc_get_frame(void *ctx, int chn, rss_frame_t *frame)
{
    i6_venc_stat stat;
    int ret;

    STAR_ENC_ENTER(ctx, chn, st, enc);

    if (!frame)
        return RSS_ERR_INVAL;
    if (enc->frame_held) {
        HAL_LOG_ERR("venc chn %d: get_frame with a frame still held", chn);
        return RSS_ERR_BUSY;
    }

    memset(&stat, 0, sizeof(stat));
    ret = st->venc.fnQuery(chn, &stat);
    if (ret) {
        HAL_LOG_ERR("MI_VENC_Query(%d) failed: %d", chn, ret);
        return RSS_ERR_IO;
    }
    if (!stat.curPacks)
        return -EAGAIN;

    memset(&enc->strm, 0, sizeof(enc->strm));
    enc->strm.packet = star_enc_packs(enc, stat.curPacks);
    if (!enc->strm.packet)
        return RSS_ERR_NOMEM;
    enc->strm.count = stat.curPacks;

    /*
     * Zero timeout: the descriptor already said a frame is ready, and
     * this call only moves descriptors, not pixels. divinus passes the
     * pack count here, which MI reads as milliseconds -- harmless, but
     * not what it looks like.
     */
    ret = st->venc.fnGetStream(chn, &enc->strm, 0);
    if (ret) {
        HAL_LOG_ERR("MI_VENC_GetStream(%d) failed: %d", chn, ret);
        return RSS_ERR_IO;
    }

    enc->frame_held = true;

    memset(frame, 0, sizeof(*frame));
    frame->codec = enc->codec;
    frame->seq = enc->strm.sequence;
    /* MI timestamps packs in microseconds, the same unit rss_frame_t
     * wants; the first pack carries the frame's capture time. */
    frame->timestamp = enc->strm.count ? (int64_t)enc->strm.packet[0].timestamp : 0;
    star_enc_fill_nals(enc, frame);
    frame->_priv = enc;

    return RSS_OK;
}

int hal_enc_release_frame(void *ctx, int chn, rss_frame_t *frame)
{
    int ret;

    STAR_ENC_ENTER(ctx, chn, st, enc);

    if (!enc->frame_held)
        return RSS_OK;

    ret = st->venc.fnFreeStream(chn, &enc->strm);
    enc->frame_held = false;
    memset(&enc->strm, 0, sizeof(enc->strm));
    if (frame) {
        frame->nals = NULL;
        frame->nal_count = 0;
        frame->_priv = NULL;
    }

    if (ret) {
        HAL_LOG_ERR("MI_VENC_ReleaseStream(%d) failed: %d", chn, ret);
        return RSS_ERR_IO;
    }

    return RSS_OK;
}

/* ================================================================
 * RUNTIME RECONFIGURATION
 *
 * MI exposes no per-knob setter: bitrate, GOP and frame rate all live
 * in the rate half of the channel attributes, which is read, modified
 * and written back whole.
 * ================================================================ */

/*
 * star_enc_reconfigure_rate -- rebuild this channel's rate config from
 * the cached settings and push it.
 *
 * Reading the current attributes first rather than constructing them
 * from scratch keeps whatever MI filled in that raptor does not model.
 */
static int star_enc_reconfigure_rate(star_state_t *st, int chn, star_venc_chn_t *enc)
{
    rss_video_config_t cfg;
    i6_venc_chn channel;
    int ret;

    memset(&channel, 0, sizeof(channel));
    ret = st->venc.fnGetChannelConfig(chn, &channel);
    if (ret) {
        HAL_LOG_ERR("MI_VENC_GetChnAttr(%d) failed: %d", chn, ret);
        return RSS_ERR_IO;
    }

    memset(&cfg, 0, sizeof(cfg));
    cfg.codec = enc->codec;
    cfg.width = enc->width;
    cfg.height = enc->height;
    cfg.rc_mode = enc->rc_mode;
    cfg.bitrate = enc->bitrate;
    cfg.max_bitrate = enc->max_bitrate;
    cfg.fps_num = enc->fps_num;
    cfg.fps_den = enc->fps_den;
    cfg.gop_length = enc->gop;
    cfg.init_qp = enc->init_qp;
    cfg.min_qp = enc->min_qp;
    cfg.max_qp = enc->max_qp;

    ret = star_enc_fill_rate(&channel.rate, enc->codec, &cfg);
    if (ret)
        return ret;

    ret = st->venc.fnSetChannelConfig(chn, &channel);
    if (ret) {
        HAL_LOG_ERR("MI_VENC_SetChnAttr(%d) failed: %d", chn, ret);
        return RSS_ERR_IO;
    }

    return RSS_OK;
}

int hal_enc_set_rc_mode(void *ctx, int chn, rss_rc_mode_t mode, uint32_t bitrate)
{
    rss_rc_mode_t prev_mode;
    unsigned int prev_bitrate;
    int ret;

    STAR_ENC_ENTER(ctx, chn, st, enc);

    prev_mode = enc->rc_mode;
    prev_bitrate = enc->bitrate;

    enc->rc_mode = mode;
    if (bitrate)
        enc->bitrate = bitrate;

    ret = star_enc_reconfigure_rate(st, chn, enc);
    if (ret) {
        enc->rc_mode = prev_mode;
        enc->bitrate = prev_bitrate;
    }

    return ret;
}

int hal_enc_set_bitrate(void *ctx, int chn, uint32_t bitrate)
{
    unsigned int prev;
    int ret;

    STAR_ENC_ENTER(ctx, chn, st, enc);

    if (!bitrate)
        return RSS_ERR_INVAL;

    prev = enc->bitrate;
    enc->bitrate = bitrate;

    ret = star_enc_reconfigure_rate(st, chn, enc);
    if (ret)
        enc->bitrate = prev;

    return ret;
}

int hal_enc_set_gop(void *ctx, int chn, uint32_t gop_length)
{
    unsigned int prev;
    int ret;

    STAR_ENC_ENTER(ctx, chn, st, enc);

    prev = enc->gop;
    enc->gop = gop_length;

    ret = star_enc_reconfigure_rate(st, chn, enc);
    if (ret)
        enc->gop = prev;

    return ret;
}

int hal_enc_set_gop_attr(void *ctx, int chn, uint32_t gop_length)
{
    return hal_enc_set_gop(ctx, chn, gop_length);
}

int hal_enc_set_fps(void *ctx, int chn, uint32_t fps_num, uint32_t fps_den)
{
    unsigned int prev_num, prev_den;
    int ret;

    STAR_ENC_ENTER(ctx, chn, st, enc);

    if (!fps_num || !fps_den)
        return RSS_ERR_INVAL;

    prev_num = enc->fps_num;
    prev_den = enc->fps_den;
    enc->fps_num = fps_num;
    enc->fps_den = fps_den;

    ret = star_enc_reconfigure_rate(st, chn, enc);
    if (ret) {
        enc->fps_num = prev_num;
        enc->fps_den = prev_den;
    }

    return ret;
}

/* ================================================================
 * QUERIES
 *
 * MI's getters return the encoder's view; where raptor tracks a value
 * MI has no getter for, the cached value is reported. Saying which is
 * which matters: get_channel_attr mixes both.
 * ================================================================ */

int hal_enc_get_channel_attr(void *ctx, int chn, rss_video_config_t *cfg)
{
    STAR_ENC_ENTER(ctx, chn, st, enc);

    if (!cfg)
        return RSS_ERR_INVAL;

    memset(cfg, 0, sizeof(*cfg));
    cfg->codec = enc->codec;
    cfg->width = enc->width;
    cfg->height = enc->height;
    cfg->rc_mode = enc->rc_mode;
    cfg->bitrate = enc->bitrate;
    cfg->max_bitrate = enc->max_bitrate;
    cfg->fps_num = enc->fps_num;
    cfg->fps_den = enc->fps_den;
    cfg->gop_length = enc->gop;
    cfg->init_qp = enc->init_qp;
    cfg->min_qp = enc->min_qp;
    cfg->max_qp = enc->max_qp;

    return RSS_OK;
}

int hal_enc_get_fps(void *ctx, int chn, uint32_t *fps_num, uint32_t *fps_den)
{
    STAR_ENC_ENTER(ctx, chn, st, enc);

    if (!fps_num || !fps_den)
        return RSS_ERR_INVAL;

    *fps_num = enc->fps_num;
    *fps_den = enc->fps_den;

    return RSS_OK;
}

int hal_enc_get_gop_attr(void *ctx, int chn, uint32_t *gop_length)
{
    STAR_ENC_ENTER(ctx, chn, st, enc);

    if (!gop_length)
        return RSS_ERR_INVAL;

    *gop_length = enc->gop;

    return RSS_OK;
}

int hal_enc_get_avg_bitrate(void *ctx, int chn, uint32_t *bitrate)
{
    i6_venc_stat stat;
    int ret;

    STAR_ENC_ENTER(ctx, chn, st, enc);

    if (!bitrate)
        return RSS_ERR_INVAL;

    memset(&stat, 0, sizeof(stat));
    ret = st->venc.fnQuery(chn, &stat);
    if (ret) {
        HAL_LOG_ERR("MI_VENC_Query(%d) failed: %d", chn, ret);
        return RSS_ERR_IO;
    }

    *bitrate = stat.bitrate;

    return RSS_OK;
}

/*
 * hal_enc_query -- is the encoder holding work?
 *
 * "Busy" here means frames are queued and not yet collected, which is
 * what rvd uses it for (deciding whether a channel is producing).
 */
int hal_enc_query(void *ctx, int chn, bool *busy)
{
    i6_venc_stat stat;
    int ret;

    STAR_ENC_ENTER(ctx, chn, st, enc);

    if (!busy)
        return RSS_ERR_INVAL;

    memset(&stat, 0, sizeof(stat));
    ret = st->venc.fnQuery(chn, &stat);
    if (ret) {
        HAL_LOG_ERR("MI_VENC_Query(%d) failed: %d", chn, ret);
        return RSS_ERR_IO;
    }

    *busy = stat.leftPics > 0 || stat.curPacks > 0;

    return RSS_OK;
}
