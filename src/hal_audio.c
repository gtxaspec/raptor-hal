/*
 * hal_audio.c -- Raptor HAL audio implementation
 *
 * Implements all audio vtable functions: AI init/deinit, volume/gain,
 * NS/HPF/AGC processing, frame read, encoder registration, and AO init.
 *
 * The audio API is mostly stable across SoCs.  Key differences:
 *   - IMPAudioIChnParam has 2 fields on T20/T21/T30/T31,
 *     3 fields on T23/T32/T40/T41 (adds aecChn).
 *   - NS/AGC/HPF APIs are identical in signature across all SoCs.
 *   - AENC register/unregister is identical across all SoCs.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: MIT
 */

#include "hal_internal.h"

/* Audio device and channel constants */
/* T31-X: analog mic is device 1; T31-N: device 0
 * TODO: make this configurable via rss_audio_config_t */
#define AI_DEV_ID 1
#define AI_CHN_ID 0
#define AO_DEV_ID 0
#define AO_CHN_ID 0
#define DMIC_DEV_ID 0
#define DMIC_CHN_ID 0

/* Default polling timeout in milliseconds */
#define AUDIO_POLL_TIMEOUT_MS 500

/* ================================================================
 * AUDIO INPUT INIT
 *
 * Build IMPAudioIOAttr from rss_audio_config_t, then:
 *   1. IMP_AI_SetPubAttr(devId, &attr)
 *   2. IMP_AI_Enable(devId)
 *   3. IMP_AI_SetChnParam(devId, chnId, &param)
 *   4. IMP_AI_EnableChn(devId, chnId)
 *
 * IMPAudioIChnParam layout differs:
 *   T20/T21/T30/T31: { usrFrmDepth, Rev }
 *   T23/T32/T40/T41: { usrFrmDepth, aecChn, Rev }
 * ================================================================ */

static int hal_audio_init_amic(const rss_audio_config_t *cfg)
{
    int ret;

    IMPAudioIOAttr attr;
    memset(&attr, 0, sizeof(attr));
    attr.samplerate = (IMPAudioSampleRate)cfg->sample_rate;
    attr.bitwidth = AUDIO_BIT_WIDTH_16;
    attr.soundmode = (cfg->chn_count >= 2) ? AUDIO_SOUND_MODE_STEREO : AUDIO_SOUND_MODE_MONO;
    attr.frmNum = (cfg->frame_depth > 0) ? cfg->frame_depth : 20;
    attr.numPerFrm = cfg->samples_per_frame;
    attr.chnCnt = 1;

    ret = IMP_AI_SetPubAttr(AI_DEV_ID, &attr);
    if (ret != 0) {
        HAL_LOG_ERR("IMP_AI_SetPubAttr failed: %d", ret);
        return ret;
    }

    ret = IMP_AI_Enable(AI_DEV_ID);
    if (ret != 0) {
        HAL_LOG_ERR("IMP_AI_Enable failed: %d", ret);
        return ret;
    }

    IMPAudioIChnParam param;
    memset(&param, 0, sizeof(param));
    param.usrFrmDepth = (cfg->frame_depth > 0) ? cfg->frame_depth : 20;
#if defined(PLATFORM_T23) || defined(PLATFORM_T32) || defined(PLATFORM_T33) ||                     \
    defined(PLATFORM_T40) || defined(PLATFORM_T41)
    param.aecChn = 0;
#endif

    ret = IMP_AI_SetChnParam(AI_DEV_ID, AI_CHN_ID, &param);
    if (ret != 0) {
        HAL_LOG_ERR("IMP_AI_SetChnParam failed: %d", ret);
        goto err_disable_dev;
    }

    ret = IMP_AI_EnableChn(AI_DEV_ID, AI_CHN_ID);
    if (ret != 0) {
        HAL_LOG_ERR("IMP_AI_EnableChn failed: %d", ret);
        goto err_disable_dev;
    }

    if (cfg->ai_vol != 0)
        IMP_AI_SetVol(AI_DEV_ID, AI_CHN_ID, cfg->ai_vol);
    if (cfg->ai_gain != 0)
        IMP_AI_SetGain(AI_DEV_ID, AI_CHN_ID, cfg->ai_gain);

    HAL_LOG_INFO("audio init: amic rate=%d samples=%d depth=%d", cfg->sample_rate,
                 cfg->samples_per_frame, cfg->frame_depth);
    return RSS_OK;

err_disable_dev:
    IMP_AI_Disable(AI_DEV_ID);
    return ret;
}

#ifdef HAL_HAS_DMIC
static int hal_audio_init_dmic(const rss_audio_config_t *cfg)
{
    int ret;
    int dmic_cnt = cfg->dmic_count > 0 ? cfg->dmic_count : 1;
    int aec_id = cfg->dmic_aec_id;
    int need_aec = (aec_id >= 0) ? 1 : 0;

    ret = IMP_DMIC_SetUserInfo(DMIC_DEV_ID, aec_id >= 0 ? aec_id : 0, need_aec);
    if (ret != 0) {
        HAL_LOG_ERR("IMP_DMIC_SetUserInfo failed: %d", ret);
        return ret;
    }

    IMPDmicAttr attr;
    memset(&attr, 0, sizeof(attr));
    attr.samplerate = (IMPDmicSampleRate)cfg->sample_rate;
    attr.bitwidth = DMIC_BIT_WIDTH_16;
    attr.soundmode = DMIC_SOUND_MODE_MONO;
    attr.frmNum = (cfg->frame_depth > 0) ? cfg->frame_depth : 25;
    attr.numPerFrm = cfg->samples_per_frame;
    attr.chnCnt = dmic_cnt;

    ret = IMP_DMIC_SetPubAttr(DMIC_DEV_ID, &attr);
    if (ret != 0) {
        HAL_LOG_ERR("IMP_DMIC_SetPubAttr failed: %d", ret);
        return ret;
    }

    ret = IMP_DMIC_Enable(DMIC_DEV_ID);
    if (ret != 0) {
        HAL_LOG_ERR("IMP_DMIC_Enable failed: %d", ret);
        return ret;
    }

    IMPDmicChnParam param;
    memset(&param, 0, sizeof(param));
    param.usrFrmDepth = (cfg->frame_depth > 0) ? cfg->frame_depth : 25;

    ret = IMP_DMIC_SetChnParam(DMIC_DEV_ID, DMIC_CHN_ID, &param);
    if (ret != 0) {
        HAL_LOG_ERR("IMP_DMIC_SetChnParam failed: %d", ret);
        goto err_disable_dev;
    }

    ret = IMP_DMIC_EnableChn(DMIC_DEV_ID, DMIC_CHN_ID);
    if (ret != 0) {
        HAL_LOG_ERR("IMP_DMIC_EnableChn failed: %d", ret);
        goto err_disable_dev;
    }

    if (cfg->ai_vol != 0)
        IMP_DMIC_SetVol(DMIC_DEV_ID, DMIC_CHN_ID, cfg->ai_vol);
    if (cfg->ai_gain != 0)
        IMP_DMIC_SetGain(DMIC_DEV_ID, DMIC_CHN_ID, cfg->ai_gain);

    HAL_LOG_INFO("audio init: dmic rate=%d samples=%d depth=%d cnt=%d", cfg->sample_rate,
                 cfg->samples_per_frame, cfg->frame_depth, dmic_cnt);
    return RSS_OK;

err_disable_dev:
    IMP_DMIC_Disable(DMIC_DEV_ID);
    return ret;
}
#endif

int hal_audio_init(void *ctx, const rss_audio_config_t *cfg)
{
    if (!cfg)
        return RSS_ERR_INVAL;

    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;
    c->audio_input_type = cfg->input_type;

#ifdef HAL_HAS_DMIC
    if (cfg->input_type == RSS_AUDIO_INPUT_DMIC)
        return hal_audio_init_dmic(cfg);
#endif
    return hal_audio_init_amic(cfg);
}

/* ================================================================
 * AUDIO INPUT DEINIT
 * ================================================================ */

int hal_audio_deinit(void *ctx)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;
    (void)c;
    int ret;
    int first_err = 0;

#ifdef HAL_HAS_DMIC
    if (c->audio_input_type == RSS_AUDIO_INPUT_DMIC) {
        ret = IMP_DMIC_DisableChn(DMIC_DEV_ID, DMIC_CHN_ID);
        if (ret != 0 && first_err == 0)
            first_err = ret;
        ret = IMP_DMIC_Disable(DMIC_DEV_ID);
        if (ret != 0 && first_err == 0)
            first_err = ret;
        return first_err;
    }
#endif

    ret = IMP_AI_DisableChn(AI_DEV_ID, AI_CHN_ID);
    if (ret != 0 && first_err == 0)
        first_err = ret;
    ret = IMP_AI_Disable(AI_DEV_ID);
    if (ret != 0 && first_err == 0)
        first_err = ret;
    return first_err;
}

/* ================================================================
 * VOLUME / GAIN
 *
 * Identical signature across all SoCs:
 *   IMP_AI_SetVol(devId, chnId, vol)      vol range: [-30..120]
 *   IMP_AI_SetGain(devId, chnId, gain)    gain range: [0..31]
 * ================================================================ */

int hal_audio_set_volume(void *ctx, int dev, int chn, int vol)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;
#ifdef HAL_HAS_DMIC
    if (c->audio_input_type == RSS_AUDIO_INPUT_DMIC)
        return IMP_DMIC_SetVol(DMIC_DEV_ID, DMIC_CHN_ID, vol);
#endif
    (void)c;
    return IMP_AI_SetVol(dev, chn, vol);
}

int hal_audio_set_gain(void *ctx, int dev, int chn, int gain)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;
#ifdef HAL_HAS_DMIC
    if (c->audio_input_type == RSS_AUDIO_INPUT_DMIC)
        return IMP_DMIC_SetGain(DMIC_DEV_ID, DMIC_CHN_ID, gain);
#endif
    (void)c;
    return IMP_AI_SetGain(dev, chn, gain);
}

/* ================================================================
 * NOISE SUPPRESSION
 *
 * IMP_AI_EnableNs(IMPAudioIOAttr *attr, int mode)
 *   - attr: audio IO attributes (the SDK needs them for NS init)
 *   - mode: Level_ns enum [0..3]
 * IMP_AI_DisableNs(void)
 *
 * Identical across all SoCs.
 * ================================================================ */

int hal_audio_enable_ns(void *ctx, rss_ns_level_t level)
{
    (void)ctx;

    /* Translate RSS ns level to SDK Level_ns */
    int ns_mode;
    switch (level) {
    case RSS_NS_LOW:
        ns_mode = NS_LOW;
        break;
    case RSS_NS_MODERATE:
        ns_mode = NS_MODERATE;
        break;
    case RSS_NS_HIGH:
        ns_mode = NS_HIGH;
        break;
    case RSS_NS_VERYHIGH:
        ns_mode = NS_VERYHIGH;
        break;
    default:
        ns_mode = NS_MODERATE;
        break;
    }

    /*
     * The SDK requires the current IO attributes for NS init.
     * Retrieve them from the device rather than caching.
     */
    IMPAudioIOAttr attr;
    memset(&attr, 0, sizeof(attr));
    int ret = IMP_AI_GetPubAttr(AI_DEV_ID, &attr);
    if (ret != 0) {
        HAL_LOG_ERR("IMP_AI_GetPubAttr for NS failed: %d", ret);
        return ret;
    }

    return IMP_AI_EnableNs(&attr, ns_mode);
}

int hal_audio_disable_ns(void *ctx)
{
    (void)ctx;
    return IMP_AI_DisableNs();
}

/* ================================================================
 * HIGH PASS FILTER
 *
 * IMP_AI_EnableHpf(IMPAudioIOAttr *attr)
 * IMP_AI_DisableHpf(void)
 *
 * Identical across all SoCs.
 * ================================================================ */

int hal_audio_enable_hpf(void *ctx)
{
    (void)ctx;

    IMPAudioIOAttr attr;
    memset(&attr, 0, sizeof(attr));
    int ret = IMP_AI_GetPubAttr(AI_DEV_ID, &attr);
    if (ret != 0) {
        HAL_LOG_ERR("IMP_AI_GetPubAttr for HPF failed: %d", ret);
        return ret;
    }

    return IMP_AI_EnableHpf(&attr);
}

int hal_audio_disable_hpf(void *ctx)
{
    (void)ctx;
    return IMP_AI_DisableHpf();
}

/* ================================================================
 * AUTOMATIC GAIN CONTROL
 *
 * IMP_AI_EnableAgc(IMPAudioIOAttr *attr, IMPAudioAgcConfig agcConfig)
 *   Note: agcConfig is passed BY VALUE on all SoCs.
 * IMP_AI_DisableAgc(void)
 *
 * Identical across all SoCs.
 * ================================================================ */

int hal_audio_enable_agc(void *ctx, const rss_agc_config_t *cfg)
{
    (void)ctx;

    if (!cfg)
        return RSS_ERR_INVAL;

    IMPAudioIOAttr attr;
    memset(&attr, 0, sizeof(attr));
    int ret = IMP_AI_GetPubAttr(AI_DEV_ID, &attr);
    if (ret != 0) {
        HAL_LOG_ERR("IMP_AI_GetPubAttr for AGC failed: %d", ret);
        return ret;
    }

    IMPAudioAgcConfig agc;
    memset(&agc, 0, sizeof(agc));
    agc.TargetLevelDbfs = cfg->target_level_dbfs;
    agc.CompressionGaindB = cfg->compression_gain_db;

    return IMP_AI_EnableAgc(&attr, agc);
}

int hal_audio_disable_agc(void *ctx)
{
    (void)ctx;
    return IMP_AI_DisableAgc();
}

/* ================================================================
 * FRAME READ
 *
 * IMP_AI_PollingFrame(devId, chnId, timeout_ms)
 * IMP_AI_GetFrame(devId, chnId, &frame, blockFlag)
 *
 * Fills rss_audio_frame_t from IMPAudioFrame.
 * ================================================================ */

int hal_audio_read_frame(void *ctx, int dev, int chn, rss_audio_frame_t *frame, bool block)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;

    if (!frame)
        return RSS_ERR_INVAL;

#ifdef HAL_HAS_DMIC
    if (c->audio_input_type == RSS_AUDIO_INPUT_DMIC) {
        (void)dev;
        (void)chn;
        int ret = IMP_DMIC_PollingFrame(DMIC_DEV_ID, DMIC_CHN_ID,
                                        block ? AUDIO_POLL_TIMEOUT_MS : 0);
        if (ret != 0)
            return (ret == -2) ? RSS_ERR_TIMEOUT : ret;

        memset(&c->dmic_frame_priv, 0, sizeof(c->dmic_frame_priv));
        ret = IMP_DMIC_GetFrame(DMIC_DEV_ID, DMIC_CHN_ID, &c->dmic_frame_priv,
                                block ? BLOCK : NOBLOCK);
        if (ret != 0)
            return ret;

        /* AEC-processed single-channel data if available, else raw */
        IMPDmicFrame *src = c->dmic_frame_priv.aecFrame.virAddr ? &c->dmic_frame_priv.aecFrame
                                                                : &c->dmic_frame_priv.rawFrame;
        frame->data = (const int16_t *)src->virAddr;
        frame->length = (uint32_t)src->len;
        frame->timestamp = src->timeStamp;
        frame->seq = (uint32_t)src->seq;
        frame->_priv = &c->dmic_frame_priv;
        return RSS_OK;
    }
#endif

    int ret = IMP_AI_PollingFrame(dev, chn, block ? AUDIO_POLL_TIMEOUT_MS : 0);
    if (ret != 0)
        return (ret == -2) ? RSS_ERR_TIMEOUT : ret;

    IMPAudioFrame ai_frame;
    memset(&ai_frame, 0, sizeof(ai_frame));
    ret = IMP_AI_GetFrame(dev, chn, &ai_frame, block ? BLOCK : NOBLOCK);
    if (ret != 0)
        return ret;

    frame->data = (const int16_t *)ai_frame.virAddr;
    frame->length = (uint32_t)ai_frame.len;
    frame->timestamp = ai_frame.timeStamp;
    frame->seq = (uint32_t)ai_frame.seq;

    memcpy(&c->ai_frame_priv, &ai_frame, sizeof(IMPAudioFrame));
    frame->_priv = &c->ai_frame_priv;
    return RSS_OK;
}

/* ================================================================
 * FRAME RELEASE
 *
 * Releases an audio frame previously obtained by read_frame.
 * The IMPAudioFrame is stored in frame->_priv by read_frame.
 * ================================================================ */

int hal_audio_release_frame(void *ctx, int dev, int chn, rss_audio_frame_t *frame)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;
    (void)c;

    if (!frame || !frame->_priv)
        return RSS_OK;

#ifdef HAL_HAS_DMIC
    if (c->audio_input_type == RSS_AUDIO_INPUT_DMIC) {
        (void)dev;
        (void)chn;
        int ret = IMP_DMIC_ReleaseFrame(DMIC_DEV_ID, DMIC_CHN_ID, frame->_priv);
        frame->_priv = NULL;
        return ret;
    }
#endif

    IMP_AI_ReleaseFrame(dev, chn, frame->_priv);
    frame->_priv = NULL;
    return RSS_OK;
}

/* ================================================================
 * ENCODER REGISTRATION
 *
 * IMP_AENC_RegisterEncoder(int *handle, IMPAudioEncEncoder *encoder)
 * IMP_AENC_UnRegisterEncoder(int *handle)
 *
 * The SDK's encoderFrm callback takes (void*, IMPAudioFrame*, uchar*, int*).
 * Our rss_audio_encoder_t.encode takes (void*, int16_t*, int, uint8_t*, int*).
 * We use a shim to bridge the two, extracting PCM from IMPAudioFrame.
 *
 * Identical across all SoCs.
 * ================================================================ */

/* User's encode callback — stored here so the shim can access it.
 * Limitation: only one custom encoder can be registered at a time. */
static int (*g_user_encode)(void *encoder, const int16_t *pcm, int pcm_len, uint8_t *out,
                            int *out_len);

/* Shim that bridges SDK callback signature to RSS signature */
static int hal_aenc_encode_shim(void *encoder, IMPAudioFrame *frame, unsigned char *outbuf,
                                int *outlen)
{
    if (!g_user_encode || !frame)
        return -1;
    return g_user_encode(encoder, (const int16_t *)(uintptr_t)frame->virAddr,
                         frame->len / (int)sizeof(int16_t), outbuf, outlen);
}

int hal_audio_register_encoder(void *ctx, const rss_audio_encoder_t *enc, int *handle)
{
    (void)ctx;

    if (!enc || !handle)
        return RSS_ERR_INVAL;

    IMPAudioEncEncoder sdk_enc;
    memset(&sdk_enc, 0, sizeof(sdk_enc));
    sdk_enc.type = PT_MAX;
    sdk_enc.maxFrmLen = enc->max_frame_len;
    snprintf(sdk_enc.name, sizeof(sdk_enc.name), "%s", enc->name);
    sdk_enc.openEncoder = enc->open;
    sdk_enc.encoderFrm = hal_aenc_encode_shim;
    sdk_enc.closeEncoder = enc->close;

    /* Store user callback so the shim can reach it */
    g_user_encode = enc->encode;

    return IMP_AENC_RegisterEncoder(handle, &sdk_enc);
}

int hal_audio_unregister_encoder(void *ctx, int handle)
{
    (void)ctx;
    return IMP_AENC_UnRegisterEncoder(&handle);
}

/* ================================================================
 * AUDIO OUTPUT INIT (for backchannel / speaker playback)
 *
 * IMP_AO_SetPubAttr(devId, &attr)
 * IMP_AO_Enable(devId)
 * IMP_AO_EnableChn(devId, chnId)
 *
 * Identical across all SoCs.
 * ================================================================ */

/* Not in the vtable as a separate function, but implemented here
 * for completeness.  The consumer would call this via a custom
 * extension or through the audio_init path with an output flag.
 */

/*
 * hal_audio_ao_init -- initialize audio output device.
 *
 * This function is not currently wired into the vtable but is
 * provided for future backchannel support.
 */
static int hal_audio_ao_init_internal(const rss_audio_config_t *cfg)
{
    int ret;

    if (!cfg)
        return RSS_ERR_INVAL;

    IMPAudioIOAttr attr;
    memset(&attr, 0, sizeof(attr));
    attr.samplerate = (IMPAudioSampleRate)cfg->sample_rate;
    attr.bitwidth = AUDIO_BIT_WIDTH_16;
    attr.soundmode = (cfg->chn_count >= 2) ? AUDIO_SOUND_MODE_STEREO : AUDIO_SOUND_MODE_MONO;
    attr.frmNum = (cfg->frame_depth > 0) ? cfg->frame_depth : 20;
    attr.numPerFrm = cfg->samples_per_frame;
    attr.chnCnt = 1;

    ret = IMP_AO_SetPubAttr(AO_DEV_ID, &attr);
    if (ret != 0) {
        HAL_LOG_ERR("IMP_AO_SetPubAttr failed: %d", ret);
        return ret;
    }

    ret = IMP_AO_Enable(AO_DEV_ID);
    if (ret != 0) {
        HAL_LOG_ERR("IMP_AO_Enable failed: %d", ret);
        return ret;
    }

    ret = IMP_AO_EnableChn(AO_DEV_ID, AO_CHN_ID);
    if (ret != 0) {
        HAL_LOG_ERR("IMP_AO_EnableChn failed: %d", ret);
        IMP_AO_Disable(AO_DEV_ID);
        return ret;
    }

    HAL_LOG_INFO("audio output init: rate=%d samples=%d", cfg->sample_rate, cfg->samples_per_frame);
    return RSS_OK;
}

/* ================================================================
 * AUDIO ENCODING PIPELINE (AENC)
 *
 * Used by RAD to encode captured PCM into AAC/Opus/G711 for
 * streaming and recording. API is identical across all SoCs.
 * ================================================================ */

int hal_aenc_create_channel(void *ctx, int chn, int codec_type)
{
    (void)ctx;
    IMPAudioEncChnAttr attr;
    memset(&attr, 0, sizeof(attr));
    attr.type = (IMPAudioPalyloadType)codec_type;
    attr.bufSize = 20; /* number of cached frames */
    return IMP_AENC_CreateChn(chn, &attr);
}

int hal_aenc_destroy_channel(void *ctx, int chn)
{
    (void)ctx;
    return IMP_AENC_DestroyChn(chn);
}

int hal_aenc_send_frame(void *ctx, int chn, rss_audio_frame_t *frame)
{
    (void)ctx;
    if (!frame || !frame->_priv)
        return RSS_ERR_INVAL;
    IMPAudioFrame *imp_frame = (IMPAudioFrame *)frame->_priv;
    return IMP_AENC_SendFrame(chn, imp_frame);
}

int hal_aenc_poll_stream(void *ctx, int chn, uint32_t timeout_ms)
{
    (void)ctx;
    return IMP_AENC_PollingStream(chn, timeout_ms);
}

int hal_aenc_get_stream(void *ctx, int chn, rss_audio_frame_t *stream)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;
    if (!stream)
        return RSS_ERR_INVAL;

    int ret = IMP_AENC_GetStream(chn, &c->aenc_stream_priv, 0);
    if (ret != 0)
        return ret;

    stream->data = (const int16_t *)c->aenc_stream_priv.stream;
    stream->length = c->aenc_stream_priv.len;
    stream->timestamp = c->aenc_stream_priv.timeStamp;
    stream->seq = c->aenc_stream_priv.seq;
    stream->_priv = &c->aenc_stream_priv;
    return RSS_OK;
}

int hal_aenc_release_stream(void *ctx, int chn, rss_audio_frame_t *stream)
{
    (void)ctx;
    if (!stream || !stream->_priv)
        return RSS_ERR_INVAL;
    IMPAudioStream *imp_stream = (IMPAudioStream *)stream->_priv;
    int ret = IMP_AENC_ReleaseStream(chn, imp_stream);
    stream->_priv = NULL;
    return ret;
}

/* ================================================================
 * AUDIO DECODING PIPELINE (ADEC)
 *
 * Used for backchannel / two-way audio. Decodes incoming
 * compressed audio for playback via AO.
 * API is identical across all SoCs.
 * ================================================================ */


int hal_adec_unregister_decoder(void *ctx, int handle)
{
    (void)ctx;
    if (handle > 0)
        return IMP_ADEC_UnRegisterDecoder(&handle);
    return RSS_OK;
}

int hal_adec_create_channel(void *ctx, int chn, int codec_type)
{
    (void)ctx;
    IMPAudioDecChnAttr attr;
    memset(&attr, 0, sizeof(attr));
    attr.type = (IMPAudioPalyloadType)codec_type;
    attr.bufSize = 20;
    attr.mode = ADEC_MODE_PACK;
    return IMP_ADEC_CreateChn(chn, &attr);
}

int hal_adec_destroy_channel(void *ctx, int chn)
{
    (void)ctx;
    return IMP_ADEC_DestroyChn(chn);
}

int hal_adec_send_stream(void *ctx, int chn, const uint8_t *data, uint32_t len, int64_t timestamp)
{
    (void)ctx;
    IMPAudioStream stream;
    memset(&stream, 0, sizeof(stream));
    stream.stream = (uint8_t *)data;
    stream.len = len;
    stream.timeStamp = timestamp;
    return IMP_ADEC_SendStream(chn, &stream, 0);
}

int hal_adec_clear_buf(void *ctx, int chn)
{
    (void)ctx;
    return IMP_ADEC_ClearChnBuf(chn);
}

/* ================================================================
 * AUDIO OUTPUT (AO) — speaker playback
 *
 * Used for backchannel. Wraps AO init/deinit/volume/gain.
 * ================================================================ */

int hal_ao_init(void *ctx, const rss_audio_config_t *cfg)
{
    (void)ctx;
    return hal_audio_ao_init_internal(cfg);
}

int hal_ao_deinit(void *ctx)
{
    (void)ctx;
    IMP_AO_DisableChn(AO_DEV_ID, AO_CHN_ID);
    IMP_AO_Disable(AO_DEV_ID);
    return RSS_OK;
}

int hal_ao_set_volume(void *ctx, int vol)
{
    (void)ctx;
    return IMP_AO_SetVol(AO_DEV_ID, AO_CHN_ID, vol);
}

int hal_ao_set_gain(void *ctx, int gain)
{
    (void)ctx;
    return IMP_AO_SetGain(AO_DEV_ID, AO_CHN_ID, gain);
}

/* ================================================================
 * AI ADDITIONAL: AEC, Volume/Gain getters, Mute, ALC Gain
 * ================================================================ */

/*
 * hal_audio_enable_aec -- enable acoustic echo cancellation.
 *
 * IMP_AI_EnableAec(aiDevId, aiChn, aoDevId, aoChn)
 * Identical across all SoCs that support it.
 */
int hal_audio_set_aec_profile_path(void *ctx, const char *dir)
{
    (void)ctx;
#if defined(PLATFORM_T23) || defined(PLATFORM_T31) || defined(PLATFORM_T32) ||                     \
    defined(PLATFORM_T33) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
    return IMP_AI_Set_WebrtcProfileIni_Path((char *)dir);
#else
    (void)dir;
    return 0;
#endif
}

int hal_audio_enable_aec(void *ctx, int ai_dev, int ai_chn, int ao_dev, int ao_chn)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;
#ifdef HAL_HAS_DMIC
    if (c->audio_input_type == RSS_AUDIO_INPUT_DMIC)
        return IMP_DMIC_EnableAec(DMIC_DEV_ID, DMIC_CHN_ID, ao_dev, ao_chn);
#endif
    (void)c;
    return IMP_AI_EnableAec(ai_dev, ai_chn, ao_dev, ao_chn);
}

int hal_audio_disable_aec(void *ctx)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;
#ifdef HAL_HAS_DMIC
    if (c->audio_input_type == RSS_AUDIO_INPUT_DMIC)
        return IMP_DMIC_DisableAec(DMIC_DEV_ID, DMIC_CHN_ID);
#endif
    (void)c;
    return IMP_AI_DisableAec(AI_DEV_ID, AI_CHN_ID);
}

int hal_audio_get_volume(void *ctx, int dev, int chn, int *vol)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;
    if (!vol)
        return RSS_ERR_INVAL;
#ifdef HAL_HAS_DMIC
    if (c->audio_input_type == RSS_AUDIO_INPUT_DMIC)
        return IMP_DMIC_GetVol(DMIC_DEV_ID, DMIC_CHN_ID, vol);
#endif
    (void)c;
    return IMP_AI_GetVol(dev, chn, vol);
}

int hal_audio_get_gain(void *ctx, int dev, int chn, int *gain)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;
    if (!gain)
        return RSS_ERR_INVAL;
#ifdef HAL_HAS_DMIC
    if (c->audio_input_type == RSS_AUDIO_INPUT_DMIC)
        return IMP_DMIC_GetGain(DMIC_DEV_ID, DMIC_CHN_ID, gain);
#endif
    (void)c;
    return IMP_AI_GetGain(dev, chn, gain);
}

int hal_audio_set_mute(void *ctx, int dev, int chn, int mute)
{
    (void)ctx;
    return IMP_AI_SetVolMute(dev, chn, mute);
}

int hal_audio_set_alc_gain(void *ctx, int dev, int chn, int gain)
{
    (void)ctx;
#if defined(PLATFORM_T21) || defined(PLATFORM_T31)
    return IMP_AI_SetAlcGain(dev, chn, gain);
#else
    (void)dev;
    (void)chn;
    (void)gain;
    return RSS_ERR_NOTSUP;
#endif
}

int hal_audio_get_alc_gain(void *ctx, int dev, int chn, int *gain)
{
    (void)ctx;
    if (!gain)
        return RSS_ERR_INVAL;
#if defined(PLATFORM_T21) || defined(PLATFORM_T31)
    return IMP_AI_GetAlcGain(dev, chn, gain);
#else
    (void)dev;
    (void)chn;
    return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * ADEC ADDITIONAL: poll/get/release stream
 * ================================================================ */

int hal_adec_poll_stream(void *ctx, int chn, uint32_t timeout_ms)
{
    (void)ctx;
    return IMP_ADEC_PollingStream(chn, timeout_ms);
}

int hal_adec_get_stream(void *ctx, int chn, rss_audio_frame_t *stream)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;
    if (!stream)
        return RSS_ERR_INVAL;

    int ret = IMP_ADEC_GetStream(chn, &c->adec_stream_priv, 0);
    if (ret != 0)
        return ret;

    stream->data = (const int16_t *)c->adec_stream_priv.stream;
    stream->length = c->adec_stream_priv.len;
    stream->timestamp = c->adec_stream_priv.timeStamp;
    stream->seq = c->adec_stream_priv.seq;
    stream->_priv = &c->adec_stream_priv;
    return RSS_OK;
}

int hal_adec_release_stream(void *ctx, int chn, rss_audio_frame_t *stream)
{
    (void)ctx;
    if (!stream || !stream->_priv)
        return RSS_ERR_INVAL;
    IMPAudioStream *imp_stream = (IMPAudioStream *)stream->_priv;
    int ret = IMP_ADEC_ReleaseStream(chn, imp_stream);
    stream->_priv = NULL;
    return ret;
}

/* ================================================================
 * AO ADDITIONAL: SendFrame, Pause, Resume, ClearBuf, etc.
 * ================================================================ */

int hal_ao_send_frame(void *ctx, const int16_t *data, uint32_t len, bool block)
{
    (void)ctx;
    IMPAudioFrame frame;

    if (!data)
        return RSS_ERR_INVAL;

    memset(&frame, 0, sizeof(frame));
    frame.virAddr = (uint32_t *)(uintptr_t)data;
    frame.len = (int)len;

    return IMP_AO_SendFrame(AO_DEV_ID, AO_CHN_ID, &frame, block ? BLOCK : NOBLOCK);
}

int hal_ao_pause(void *ctx)
{
    (void)ctx;
    return IMP_AO_PauseChn(AO_DEV_ID, AO_CHN_ID);
}

int hal_ao_resume(void *ctx)
{
    (void)ctx;
    return IMP_AO_ResumeChn(AO_DEV_ID, AO_CHN_ID);
}

int hal_ao_clear_buf(void *ctx)
{
    (void)ctx;
    return IMP_AO_ClearChnBuf(AO_DEV_ID, AO_CHN_ID);
}

int hal_ao_flush_buf(void *ctx)
{
    (void)ctx;
    return IMP_AO_FlushChnBuf(AO_DEV_ID, AO_CHN_ID);
}

int hal_ao_get_volume(void *ctx, int *vol)
{
    (void)ctx;
    if (!vol)
        return RSS_ERR_INVAL;
    return IMP_AO_GetVol(AO_DEV_ID, AO_CHN_ID, vol);
}

int hal_ao_get_gain(void *ctx, int *gain)
{
    (void)ctx;
    if (!gain)
        return RSS_ERR_INVAL;
    return IMP_AO_GetGain(AO_DEV_ID, AO_CHN_ID, gain);
}

int hal_ao_set_mute(void *ctx, int mute)
{
    (void)ctx;
    return IMP_AO_SetVolMute(AO_DEV_ID, AO_CHN_ID, mute);
}

int hal_ao_enable_hpf(void *ctx)
{
    (void)ctx;
    IMPAudioIOAttr attr;
    memset(&attr, 0, sizeof(attr));
    int ret = IMP_AO_GetPubAttr(AO_DEV_ID, &attr);
    if (ret != 0) {
        HAL_LOG_ERR("IMP_AO_GetPubAttr for HPF failed: %d", ret);
        return ret;
    }
    return IMP_AO_EnableHpf(&attr);
}

int hal_ao_disable_hpf(void *ctx)
{
    (void)ctx;
    return IMP_AO_DisableHpf();
}

int hal_ao_enable_agc(void *ctx)
{
    (void)ctx;
    IMPAudioIOAttr attr;
    IMPAudioAgcConfig agc;

    memset(&attr, 0, sizeof(attr));
    int ret = IMP_AO_GetPubAttr(AO_DEV_ID, &attr);
    if (ret != 0) {
        HAL_LOG_ERR("IMP_AO_GetPubAttr for AGC failed: %d", ret);
        return ret;
    }

    memset(&agc, 0, sizeof(agc));
    agc.TargetLevelDbfs = 0;
    agc.CompressionGaindB = 0;

    return IMP_AO_EnableAgc(&attr, agc);
}

int hal_ao_disable_agc(void *ctx)
{
    (void)ctx;
    return IMP_AO_DisableAgc();
}

/* ================================================================
 * AI ADDITIONAL: SetAgcMode, SetHpfCoFrequency, AecRefFrame,
 *                GetChnParam, GetFrameAndRef
 * ================================================================ */

/*
 * hal_audio_set_agc_mode -- set AGC mode.
 *
 * IMP_AI_SetAgcMode(int mode)
 * T31 only.
 */
int hal_audio_set_agc_mode(void *ctx, int mode)
{
    (void)ctx;
#if defined(PLATFORM_T31)
    return IMP_AI_SetAgcMode(mode);
#else
    (void)mode;
    return RSS_ERR_NOTSUP;
#endif
}

/*
 * hal_audio_set_hpf_co_freq -- set HPF cutoff frequency for AI.
 *
 * IMP_AI_SetHpfCoFrequency(int cofrequency)
 * T23+T31+T32+T40+T41.
 */
int hal_audio_set_hpf_co_freq(void *ctx, int freq)
{
    (void)ctx;
#if defined(PLATFORM_T23) || defined(PLATFORM_T31) || defined(PLATFORM_T32) ||                     \
    defined(PLATFORM_T33) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
    return IMP_AI_SetHpfCoFrequency(freq);
#else
    (void)freq;
    return RSS_ERR_NOTSUP;
#endif
}

/*
 * hal_audio_enable_aec_ref_frame -- enable AEC reference frame access.
 *
 * IMP_AI_EnableAecRefFrame(devId, chnId, aoDevId, aoChnId)
 * All SoCs.
 */
int hal_audio_enable_aec_ref_frame(void *ctx, int ai_dev, int ai_chn, int ao_dev, int ao_chn)
{
    (void)ctx;
    return IMP_AI_EnableAecRefFrame(ai_dev, ai_chn, ao_dev, ao_chn);
}

/*
 * hal_audio_disable_aec_ref_frame -- disable AEC reference frame access.
 *
 * IMP_AI_DisableAecRefFrame(devId, chnId, aoDevId, aoChnId)
 * All SoCs. Note: vendor API takes 4 params on T31 headers.
 */
int hal_audio_disable_aec_ref_frame(void *ctx, int ai_dev, int ai_chn)
{
    (void)ctx;
    return IMP_AI_DisableAecRefFrame(ai_dev, ai_chn, AO_DEV_ID, AO_CHN_ID);
}

/*
 * hal_audio_get_chn_param -- get audio input channel parameters.
 *
 * IMP_AI_GetChnParam(devId, chnId, IMPAudioIChnParam *param)
 * All SoCs.
 */
int hal_audio_get_chn_param(void *ctx, int dev, int chn, void *param)
{
    (void)ctx;
    if (!param)
        return RSS_ERR_INVAL;
    return IMP_AI_GetChnParam(dev, chn, (IMPAudioIChnParam *)param);
}

/*
 * hal_audio_get_frame_and_ref -- get audio frame and reference frame.
 *
 * IMP_AI_GetFrameAndRef(devId, chnId, frame*, ref*, block)
 * All SoCs.
 */
int hal_audio_get_frame_and_ref(void *ctx, int dev, int chn, void *frame, void *ref, int block)
{
    (void)ctx;
    if (!frame || !ref)
        return RSS_ERR_INVAL;
    return IMP_AI_GetFrameAndRef(dev, chn, (IMPAudioFrame *)frame, (IMPAudioFrame *)ref,
                                 block ? BLOCK : NOBLOCK);
}

/* ================================================================
 * AO ADDITIONAL: SetHpfCoFrequency, QueryChnStat, Soft_Mute,
 *                Soft_UNMute, CacheSwitch
 * ================================================================ */

/*
 * hal_ao_set_hpf_co_freq -- set HPF cutoff frequency for AO.
 *
 * IMP_AO_SetHpfCoFrequency(int cofrequency)
 * T23+T31+T32+T40+T41.
 */
int hal_ao_set_hpf_co_freq(void *ctx, int freq)
{
    (void)ctx;
#if defined(PLATFORM_T23) || defined(PLATFORM_T31) || defined(PLATFORM_T32) ||                     \
    defined(PLATFORM_T33) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
    return IMP_AO_SetHpfCoFrequency(freq);
#else
    (void)freq;
    return RSS_ERR_NOTSUP;
#endif
}

/*
 * hal_ao_query_chn_stat -- query audio output channel state.
 *
 * IMP_AO_QueryChnStat(devId, chnId, IMPAudioOChnState *status)
 * All SoCs.
 */
int hal_ao_query_chn_stat(void *ctx, int dev, int chn, void *stat)
{
    (void)ctx;
    if (!stat)
        return RSS_ERR_INVAL;
    return IMP_AO_QueryChnStat(dev, chn, (IMPAudioOChnState *)stat);
}

/*
 * hal_ao_soft_mute -- soft mute audio output.
 *
 * IMP_AO_Soft_Mute(devId, chnId)
 * All SoCs.
 */
int hal_ao_soft_mute(void *ctx, int dev, int chn)
{
    (void)ctx;
    return IMP_AO_Soft_Mute(dev, chn);
}

/*
 * hal_ao_soft_unmute -- soft unmute audio output.
 *
 * IMP_AO_Soft_UNMute(devId, chnId)
 * All SoCs.
 */
int hal_ao_soft_unmute(void *ctx, int dev, int chn)
{
    (void)ctx;
    return IMP_AO_Soft_UNMute(dev, chn);
}

/*
 * hal_ao_cache_switch -- enable/disable audio output cache.
 *
 * IMP_AO_CacheSwitch(devId, chnId, cache_en)
 * All SoCs.
 */
int hal_ao_cache_switch(void *ctx, int dev, int chn, int enable)
{
    (void)ctx;
    return IMP_AO_CacheSwitch(dev, chn, enable);
}

/* ================================================================
 * ADEC ADDITIONAL: RegisterDecoder (real implementation)
 *
 * IMP_ADEC_RegisterDecoder(int *handle, IMPAudioDecDecoder *decoder)
 * All SoCs.
 * ================================================================ */

int hal_adec_register_decoder_real(void *ctx, int *handle, void *decoder)
{
    (void)ctx;
    if (!handle || !decoder)
        return RSS_ERR_INVAL;
    return IMP_ADEC_RegisterDecoder(handle, (IMPAudioDecDecoder *)decoder);
}
