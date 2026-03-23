/*
 * hal_dmic.c -- Raptor HAL DMIC (Digital Microphone) implementation
 *
 * Implements all DMIC vtable functions: init/deinit, volume/gain,
 * channel parameters, frame read/release, and polling.
 *
 * DMIC is only available on T30/T31/T32/T40/T41 SoCs that have
 * imp_dmic.h.  On unsupported SoCs, all functions return
 * RSS_ERR_NOTSUP.
 *
 * The pattern mirrors hal_audio.c but uses IMP_DMIC_* functions.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: MIT
 */

#include "hal_internal.h"

/* DMIC is available on T30/T31/T32/T40/T41 */
#if defined(PLATFORM_T30) || defined(PLATFORM_T31) || defined(PLATFORM_T32) || \
    defined(PLATFORM_T40) || defined(PLATFORM_T41)
#define HAL_HAS_DMIC
#include <imp/imp_dmic.h>
/* T30 header declares EnableAec but not DisableAec/DisableAecRefFrame
 * even though the functions exist in libimp.so */
#if defined(PLATFORM_T30)
int IMP_DMIC_DisableAec(int dmicDevId, int dmicChnId);
int IMP_DMIC_DisableAecRefFrame(int dmicDevId, int dmicChnId);
#endif
#endif

/* DMIC device and channel constants */
#define DMIC_DEV_ID  0
#define DMIC_CHN_ID  0

#ifdef HAL_HAS_DMIC

/* ================================================================
 * DMIC INIT
 *
 * Build IMPDmicAttr from rss_audio_config_t, then:
 *   1. IMP_DMIC_SetPubAttr(devId, &attr)
 *   2. IMP_DMIC_Enable(devId)
 *   3. IMP_DMIC_SetChnParam(devId, chnId, &param)
 *   4. IMP_DMIC_EnableChn(devId, chnId)
 * ================================================================ */

int hal_dmic_init(void *ctx, const rss_audio_config_t *cfg)
{
	(void)ctx;
	int ret;

	if (!cfg)
		return RSS_ERR_INVAL;

	/* Build DMIC attributes */
	IMPDmicAttr attr;
	memset(&attr, 0, sizeof(attr));
	attr.samplerate = (IMPDmicSampleRate)cfg->sample_rate;
	attr.bitwidth   = DMIC_BIT_WIDTH_16;
	attr.soundmode  = (cfg->chn_count >= 2) ? DMIC_SOUND_MODE_STEREO
	                                        : DMIC_SOUND_MODE_MONO;
	attr.frmNum     = (cfg->frame_depth > 0) ? cfg->frame_depth : 25;
	attr.numPerFrm  = cfg->samples_per_frame;
	attr.chnCnt     = 1;

	/* Step 1: set device attributes */
	ret = IMP_DMIC_SetPubAttr(DMIC_DEV_ID, &attr);
	if (ret != 0) {
		HAL_LOG_ERR("IMP_DMIC_SetPubAttr failed: %d", ret);
		return ret;
	}

	/* Step 2: enable device */
	ret = IMP_DMIC_Enable(DMIC_DEV_ID);
	if (ret != 0) {
		HAL_LOG_ERR("IMP_DMIC_Enable failed: %d", ret);
		return ret;
	}

	/* Step 3: set channel parameters */
	IMPDmicChnParam param;
	memset(&param, 0, sizeof(param));
	param.usrFrmDepth = (cfg->frame_depth > 0) ? cfg->frame_depth : 25;

	ret = IMP_DMIC_SetChnParam(DMIC_DEV_ID, DMIC_CHN_ID, &param);
	if (ret != 0) {
		HAL_LOG_ERR("IMP_DMIC_SetChnParam failed: %d", ret);
		goto err_disable_dev;
	}

	/* Step 4: enable channel */
	ret = IMP_DMIC_EnableChn(DMIC_DEV_ID, DMIC_CHN_ID);
	if (ret != 0) {
		HAL_LOG_ERR("IMP_DMIC_EnableChn failed: %d", ret);
		goto err_disable_dev;
	}

	/* Apply initial volume and gain if non-default */
	if (cfg->ai_vol != 0)
		IMP_DMIC_SetVol(DMIC_DEV_ID, DMIC_CHN_ID, cfg->ai_vol);
	if (cfg->ai_gain != 0)
		IMP_DMIC_SetGain(DMIC_DEV_ID, DMIC_CHN_ID, cfg->ai_gain);

	HAL_LOG_INFO("dmic init: rate=%d samples=%d depth=%d",
	             cfg->sample_rate, cfg->samples_per_frame, cfg->frame_depth);
	return RSS_OK;

err_disable_dev:
	IMP_DMIC_Disable(DMIC_DEV_ID);
	return ret;
}

/* ================================================================
 * DMIC DEINIT
 * ================================================================ */

int hal_dmic_deinit(void *ctx)
{
	(void)ctx;
	int ret;
	int first_err = 0;

	ret = IMP_DMIC_DisableChn(DMIC_DEV_ID, DMIC_CHN_ID);
	if (ret != 0 && first_err == 0)
		first_err = ret;

	ret = IMP_DMIC_Disable(DMIC_DEV_ID);
	if (ret != 0 && first_err == 0)
		first_err = ret;

	return first_err;
}

/* ================================================================
 * VOLUME / GAIN
 *
 * IMP_DMIC_SetVol(devId, chnId, vol)    vol range: [-30..120]
 * IMP_DMIC_GetVol(devId, chnId, *vol)
 * IMP_DMIC_SetGain(devId, chnId, gain)  gain range: [0..31]
 * IMP_DMIC_GetGain(devId, chnId, *gain)
 * ================================================================ */

int hal_dmic_set_volume(void *ctx, int vol)
{
	(void)ctx;
	return IMP_DMIC_SetVol(DMIC_DEV_ID, DMIC_CHN_ID, vol);
}

int hal_dmic_get_volume(void *ctx, int *vol)
{
	(void)ctx;
	if (!vol)
		return RSS_ERR_INVAL;
	return IMP_DMIC_GetVol(DMIC_DEV_ID, DMIC_CHN_ID, vol);
}

int hal_dmic_set_gain(void *ctx, int gain)
{
	(void)ctx;
	return IMP_DMIC_SetGain(DMIC_DEV_ID, DMIC_CHN_ID, gain);
}

int hal_dmic_get_gain(void *ctx, int *gain)
{
	(void)ctx;
	if (!gain)
		return RSS_ERR_INVAL;
	return IMP_DMIC_GetGain(DMIC_DEV_ID, DMIC_CHN_ID, gain);
}

/* ================================================================
 * CHANNEL PARAMETERS
 *
 * IMP_DMIC_SetChnParam / IMP_DMIC_GetChnParam
 * ================================================================ */

int hal_dmic_set_chn_param(void *ctx, int chn, int frames_per_buf)
{
	(void)ctx;
	IMPDmicChnParam param;
	memset(&param, 0, sizeof(param));
	param.usrFrmDepth = frames_per_buf;
	return IMP_DMIC_SetChnParam(DMIC_DEV_ID, chn, &param);
}

int hal_dmic_get_chn_param(void *ctx, int chn, int *frames_per_buf)
{
	(void)ctx;
	if (!frames_per_buf)
		return RSS_ERR_INVAL;
	IMPDmicChnParam param;
	memset(&param, 0, sizeof(param));
	int ret = IMP_DMIC_GetChnParam(DMIC_DEV_ID, chn, &param);
	if (ret == 0)
		*frames_per_buf = param.usrFrmDepth;
	return ret;
}

/* ================================================================
 * FRAME POLLING / READ / RELEASE
 *
 * IMP_DMIC_PollingFrame(devId, chnId, timeout_ms)
 * IMP_DMIC_GetFrame(devId, chnId, &frame, block)
 * IMP_DMIC_ReleaseFrame(devId, chnId, &frame)
 *
 * Fills rss_audio_frame_t from IMPDmicChnFrame.
 * ================================================================ */

int hal_dmic_poll_frame(void *ctx, uint32_t timeout_ms)
{
	(void)ctx;
	return IMP_DMIC_PollingFrame(DMIC_DEV_ID, DMIC_CHN_ID, timeout_ms);
}

int hal_dmic_read_frame(void *ctx, rss_audio_frame_t *frame, bool block)
{
	(void)ctx;

	if (!frame)
		return RSS_ERR_INVAL;

	IMPDmicChnFrame chn_frame;
	memset(&chn_frame, 0, sizeof(chn_frame));
	int ret = IMP_DMIC_GetFrame(DMIC_DEV_ID, DMIC_CHN_ID, &chn_frame,
	                            block ? BLOCK : NOBLOCK);
	if (ret != 0)
		return ret;

	/* Fill the HAL frame from the raw DMIC frame */
	frame->data      = (const int16_t *)chn_frame.rawFrame.virAddr;
	frame->length    = (uint32_t)chn_frame.rawFrame.len;
	frame->timestamp = chn_frame.rawFrame.timeStamp;
	frame->seq       = (uint32_t)chn_frame.rawFrame.seq;

	/*
	 * Store the native frame in _priv so we can release it later.
	 * Allocate a copy of the IMPDmicChnFrame struct on the heap.
	 */
	IMPDmicChnFrame *saved = (IMPDmicChnFrame *)malloc(sizeof(IMPDmicChnFrame));
	if (!saved) {
		IMP_DMIC_ReleaseFrame(DMIC_DEV_ID, DMIC_CHN_ID, &chn_frame);
		return RSS_ERR_NOMEM;
	}
	memcpy(saved, &chn_frame, sizeof(IMPDmicChnFrame));
	frame->_priv = saved;

	return RSS_OK;
}

int hal_dmic_release_frame(void *ctx, rss_audio_frame_t *frame)
{
	(void)ctx;
	if (!frame || !frame->_priv)
		return RSS_ERR_INVAL;

	IMPDmicChnFrame *chn_frame = (IMPDmicChnFrame *)frame->_priv;
	int ret = IMP_DMIC_ReleaseFrame(DMIC_DEV_ID, DMIC_CHN_ID, chn_frame);
	free(chn_frame);
	frame->_priv = NULL;
	return ret;
}

/* ================================================================
 * DMIC AEC / AEC REF FRAME / GetPubAttr / GetFrameAndRef
 *
 * IMP_DMIC_EnableAec(devId, chn, aoDevId, aoChn)
 * IMP_DMIC_DisableAec(devId, chn)
 * IMP_DMIC_EnableAecRefFrame(devId, chn, aoDevId, aoChn)
 * IMP_DMIC_DisableAecRefFrame(devId, chn, aoDevId, aoChn)
 * IMP_DMIC_GetPubAttr(devId, attr*)
 * IMP_DMIC_GetFrameAndRef(devId, chn, frame*, ref*, block)
 * ================================================================ */

int hal_dmic_enable_aec(void *ctx, int dev, int chn, int ao_dev, int ao_chn)
{
	(void)ctx;
	return IMP_DMIC_EnableAec(dev, chn, ao_dev, ao_chn);
}

int hal_dmic_disable_aec(void *ctx, int dev, int chn)
{
	(void)ctx;
	return IMP_DMIC_DisableAec(dev, chn);
}

int hal_dmic_enable_aec_ref_frame(void *ctx, int dev, int chn,
                                  int ao_dev, int ao_chn)
{
	(void)ctx;
	return IMP_DMIC_EnableAecRefFrame(dev, chn, ao_dev, ao_chn);
}

int hal_dmic_disable_aec_ref_frame(void *ctx, int dev, int chn,
                                   int ao_dev, int ao_chn)
{
	(void)ctx;
#if defined(PLATFORM_T32) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
	return IMP_DMIC_DisableAecRefFrame(dev, chn, ao_dev, ao_chn);
#else
	(void)dev; (void)chn; (void)ao_dev; (void)ao_chn;
	return RSS_ERR_NOTSUP;
#endif
}

int hal_dmic_get_pub_attr(void *ctx, int dev, void *attr)
{
	(void)ctx;
	if (!attr)
		return RSS_ERR_INVAL;
	return IMP_DMIC_GetPubAttr(dev, (IMPDmicAttr *)attr);
}

int hal_dmic_get_frame_and_ref(void *ctx, int dev, int chn,
                               void *frame, void *ref, int block)
{
	(void)ctx;
	if (!frame || !ref)
		return RSS_ERR_INVAL;
	return IMP_DMIC_GetFrameAndRef(dev, chn,
	                               (IMPDmicChnFrame *)frame,
	                               (IMPDmicFrame *)ref,
	                               block ? BLOCK : NOBLOCK);
}

#else /* !HAL_HAS_DMIC -- unsupported SoCs */

/* ================================================================
 * STUB IMPLEMENTATIONS
 *
 * For SoCs without DMIC support (T20/T21/T23), all functions
 * return RSS_ERR_NOTSUP.
 * ================================================================ */

int hal_dmic_init(void *ctx, const rss_audio_config_t *cfg)
{
	(void)ctx; (void)cfg;
	return RSS_ERR_NOTSUP;
}

int hal_dmic_deinit(void *ctx)
{
	(void)ctx;
	return RSS_ERR_NOTSUP;
}

int hal_dmic_set_volume(void *ctx, int vol)
{
	(void)ctx; (void)vol;
	return RSS_ERR_NOTSUP;
}

int hal_dmic_get_volume(void *ctx, int *vol)
{
	(void)ctx; (void)vol;
	return RSS_ERR_NOTSUP;
}

int hal_dmic_set_gain(void *ctx, int gain)
{
	(void)ctx; (void)gain;
	return RSS_ERR_NOTSUP;
}

int hal_dmic_get_gain(void *ctx, int *gain)
{
	(void)ctx; (void)gain;
	return RSS_ERR_NOTSUP;
}

int hal_dmic_set_chn_param(void *ctx, int chn, int frames_per_buf)
{
	(void)ctx; (void)chn; (void)frames_per_buf;
	return RSS_ERR_NOTSUP;
}

int hal_dmic_get_chn_param(void *ctx, int chn, int *frames_per_buf)
{
	(void)ctx; (void)chn; (void)frames_per_buf;
	return RSS_ERR_NOTSUP;
}

int hal_dmic_read_frame(void *ctx, rss_audio_frame_t *frame, bool block)
{
	(void)ctx; (void)frame; (void)block;
	return RSS_ERR_NOTSUP;
}

int hal_dmic_release_frame(void *ctx, rss_audio_frame_t *frame)
{
	(void)ctx; (void)frame;
	return RSS_ERR_NOTSUP;
}

int hal_dmic_poll_frame(void *ctx, uint32_t timeout_ms)
{
	(void)ctx; (void)timeout_ms;
	return RSS_ERR_NOTSUP;
}

int hal_dmic_enable_aec(void *ctx, int dev, int chn, int ao_dev, int ao_chn)
{
	(void)ctx; (void)dev; (void)chn; (void)ao_dev; (void)ao_chn;
	return RSS_ERR_NOTSUP;
}

int hal_dmic_disable_aec(void *ctx, int dev, int chn)
{
	(void)ctx; (void)dev; (void)chn;
	return RSS_ERR_NOTSUP;
}

int hal_dmic_enable_aec_ref_frame(void *ctx, int dev, int chn,
                                  int ao_dev, int ao_chn)
{
	(void)ctx; (void)dev; (void)chn; (void)ao_dev; (void)ao_chn;
	return RSS_ERR_NOTSUP;
}

int hal_dmic_disable_aec_ref_frame(void *ctx, int dev, int chn,
                                   int ao_dev, int ao_chn)
{
	(void)ctx; (void)dev; (void)chn; (void)ao_dev; (void)ao_chn;
	return RSS_ERR_NOTSUP;
}

int hal_dmic_get_pub_attr(void *ctx, int dev, void *attr)
{
	(void)ctx; (void)dev; (void)attr;
	return RSS_ERR_NOTSUP;
}

int hal_dmic_get_frame_and_ref(void *ctx, int dev, int chn,
                               void *frame, void *ref, int block)
{
	(void)ctx; (void)dev; (void)chn; (void)frame; (void)ref; (void)block;
	return RSS_ERR_NOTSUP;
}

#endif /* HAL_HAS_DMIC */
