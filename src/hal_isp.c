/*
 * hal_isp.c -- Raptor HAL ISP tuning implementation
 *
 * Implements the ISP tuning vtable functions across the three-generation
 * Ingenic SDK signature break:
 *
 *   Gen1 (T20/T21/T30):       scalar args, no IMPVI_NUM
 *   Gen2 (T23/T31):           scalar args, no IMPVI_NUM, combined HVFLIP
 *   Gen3 (T32/T40/T41):       IMPVI_NUM + pointer args
 *
 * This single file is compiled for ALL platforms. The correct code path
 * is selected via #ifdef at compile time.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: MIT
 */

#include "hal_internal.h"

/* Per-SoC caps (defined in hal_caps.c) */
extern const rss_hal_caps_t g_hal_caps;

/* ================================================================
 * BASIC IMAGE CONTROLS: brightness, contrast, saturation, sharpness
 *
 * Gen1/Gen2: IMP_ISP_Tuning_Set*(unsigned char val)
 * Gen3:      IMP_ISP_Tuning_Set*(IMPVI_NUM, unsigned char *)
 * ================================================================ */

int hal_isp_set_brightness(void *ctx, uint8_t val)
{
	(void)ctx;
#if defined(HAL_ISP_PTR_ARGS)
	return IMP_ISP_Tuning_SetBrightness(IMPVI_MAIN, &val);
#else
	return IMP_ISP_Tuning_SetBrightness(val);
#endif
}

int hal_isp_set_contrast(void *ctx, uint8_t val)
{
	(void)ctx;
#if defined(HAL_ISP_PTR_ARGS)
	return IMP_ISP_Tuning_SetContrast(IMPVI_MAIN, &val);
#else
	return IMP_ISP_Tuning_SetContrast(val);
#endif
}

int hal_isp_set_saturation(void *ctx, uint8_t val)
{
	(void)ctx;
#if defined(HAL_ISP_PTR_ARGS)
	return IMP_ISP_Tuning_SetSaturation(IMPVI_MAIN, &val);
#else
	return IMP_ISP_Tuning_SetSaturation(val);
#endif
}

int hal_isp_set_sharpness(void *ctx, uint8_t val)
{
	(void)ctx;
#if defined(HAL_ISP_PTR_ARGS)
	return IMP_ISP_Tuning_SetSharpness(IMPVI_MAIN, &val);
#else
	return IMP_ISP_Tuning_SetSharpness(val);
#endif
}

/* ================================================================
 * HUE (SetBcshHue)
 *
 * Only present on T23, T31, T32, T40, T41.
 * Not available on T20, T21, T30.
 * ================================================================ */

int hal_isp_set_hue(void *ctx, uint8_t val)
{
	(void)ctx;
#if defined(HAL_ISP_PTR_ARGS)
	/* Gen3: T32/T40/T41 */
	return IMP_ISP_Tuning_SetBcshHue(IMPVI_MAIN, &val);
#elif defined(PLATFORM_T23) || defined(PLATFORM_T31)
	/* Gen2: scalar */
	return IMP_ISP_Tuning_SetBcshHue(val);
#else
	/* T20/T21/T30: not supported */
	(void)val;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * FLIP / MIRROR
 *
 * T20/T30:     SetISPHflip(OpsMode) + SetISPVflip(OpsMode)
 * T21:         SetISPHflip(OpsMode) + SetISPVflip(OpsMode)
 * T23/T31:     SetHVFLIP(IMPISPHVFLIP)
 * T40:         SetHVFLIP(IMPVI_NUM, IMPISPHVFLIP*)
 * T32/T41:     SetHVFLIP(IMPVI_NUM, IMPISPHVFLIPAttr*)
 *
 * The HAL exposes separate set_hflip / set_vflip.  We cache both
 * states in the context and re-apply the combined flip on each call.
 * ================================================================ */

int hal_isp_set_hflip(void *ctx, int enable)
{
	rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;

	c->hflip_state = enable ? 1 : 0;

#if defined(PLATFORM_T20) || defined(PLATFORM_T21) || defined(PLATFORM_T30)
	/* Separate H/V flip calls */
	IMPISPTuningOpsMode h = enable ? IMPISP_TUNING_OPS_MODE_ENABLE
	                               : IMPISP_TUNING_OPS_MODE_DISABLE;
	return IMP_ISP_Tuning_SetISPHflip(h);

#elif defined(PLATFORM_T23) || defined(PLATFORM_T31)
	/* Combined HVFLIP enum: 0=normal, 1=H, 2=V, 3=HV */
	int mode = (c->hflip_state ? 1 : 0) | (c->vflip_state ? 2 : 0);
	IMPISPHVFLIP hvflip = (IMPISPHVFLIP)mode;
	return IMP_ISP_Tuning_SetHVFLIP(hvflip);

#elif defined(PLATFORM_T40)
	int mode = (c->hflip_state ? 1 : 0) | (c->vflip_state ? 2 : 0);
	IMPISPHVFLIP hvflip = (IMPISPHVFLIP)mode;
	return IMP_ISP_Tuning_SetHVFLIP(IMPVI_MAIN, &hvflip);

#elif defined(PLATFORM_T32) || defined(PLATFORM_T41)
	int mode = (c->hflip_state ? 1 : 0) | (c->vflip_state ? 2 : 0);
	IMPISPHVFLIPAttr attr;
	memset(&attr, 0, sizeof(attr));
	attr.sensor_mode = (IMPISPHVFLIP)mode;
	attr.isp_mode[0] = (IMPISPHVFLIP)mode;
	return IMP_ISP_Tuning_SetHVFLIP(IMPVI_MAIN, &attr);
#endif
}

int hal_isp_set_vflip(void *ctx, int enable)
{
	rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;

	c->vflip_state = enable ? 1 : 0;

#if defined(PLATFORM_T20) || defined(PLATFORM_T21) || defined(PLATFORM_T30)
	IMPISPTuningOpsMode v = enable ? IMPISP_TUNING_OPS_MODE_ENABLE
	                               : IMPISP_TUNING_OPS_MODE_DISABLE;
	return IMP_ISP_Tuning_SetISPVflip(v);

#elif defined(PLATFORM_T23) || defined(PLATFORM_T31)
	int mode = (c->hflip_state ? 1 : 0) | (c->vflip_state ? 2 : 0);
	IMPISPHVFLIP hvflip = (IMPISPHVFLIP)mode;
	return IMP_ISP_Tuning_SetHVFLIP(hvflip);

#elif defined(PLATFORM_T40)
	int mode = (c->hflip_state ? 1 : 0) | (c->vflip_state ? 2 : 0);
	IMPISPHVFLIP hvflip = (IMPISPHVFLIP)mode;
	return IMP_ISP_Tuning_SetHVFLIP(IMPVI_MAIN, &hvflip);

#elif defined(PLATFORM_T32) || defined(PLATFORM_T41)
	int mode = (c->hflip_state ? 1 : 0) | (c->vflip_state ? 2 : 0);
	IMPISPHVFLIPAttr attr;
	memset(&attr, 0, sizeof(attr));
	attr.sensor_mode = (IMPISPHVFLIP)mode;
	attr.isp_mode[0] = (IMPISPHVFLIP)mode;
	return IMP_ISP_Tuning_SetHVFLIP(IMPVI_MAIN, &attr);
#endif
}

/* ================================================================
 * RUNNING MODE (day / night)
 *
 * Gen1/Gen2: SetISPRunningMode(IMPISPRunningMode mode)  -- by value
 * Gen3:      SetISPRunningMode(IMPVI_NUM, IMPISPRunningMode*)
 * ================================================================ */

int hal_isp_set_running_mode(void *ctx, rss_isp_mode_t mode)
{
	(void)ctx;

	IMPISPRunningMode imp_mode = (mode == RSS_ISP_NIGHT)
	    ? IMPISP_RUNNING_MODE_NIGHT
	    : IMPISP_RUNNING_MODE_DAY;

#if defined(HAL_ISP_PTR_ARGS)
	return IMP_ISP_Tuning_SetISPRunningMode(IMPVI_MAIN, &imp_mode);
#else
	return IMP_ISP_Tuning_SetISPRunningMode(imp_mode);
#endif
}

/* ================================================================
 * SENSOR FPS
 *
 * Gen1/Gen2: SetSensorFPS(uint32_t fps_num, uint32_t fps_den)
 * T40:       SetSensorFPS(IMPVI_NUM, uint32_t*, uint32_t*)
 * T32/T41:   SetSensorFPS(IMPVI_NUM, IMPISPSensorFps*)
 * ================================================================ */

int hal_isp_set_sensor_fps(void *ctx, uint32_t fps_num, uint32_t fps_den)
{
	(void)ctx;

#if defined(PLATFORM_T40)
	return IMP_ISP_Tuning_SetSensorFPS(IMPVI_MAIN, &fps_num, &fps_den);
#elif defined(PLATFORM_T32) || defined(PLATFORM_T41)
	IMPISPSensorFps fps = { .num = fps_num, .den = fps_den };
	return IMP_ISP_Tuning_SetSensorFPS(IMPVI_MAIN, &fps);
#else
	return IMP_ISP_Tuning_SetSensorFPS(fps_num, fps_den);
#endif
}

/* ================================================================
 * ANTI-FLICKER
 *
 * Gen1/Gen2: SetAntiFlickerAttr(IMPISPAntiflickerAttr attr)  -- by value
 * Gen3:      SetAntiFlickerAttr(IMPVI_NUM, IMPISPAntiflickerAttr*)
 * ================================================================ */

int hal_isp_set_antiflicker(void *ctx, rss_antiflicker_t mode)
{
	(void)ctx;

#if defined(PLATFORM_T32) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
	/* Gen3: IMPISPAntiflickerAttr is a struct with .mode and .freq */
	{
		IMPISPAntiflickerAttr attr;
		memset(&attr, 0, sizeof(attr));
		switch (mode) {
		case RSS_ANTIFLICKER_50HZ:
			attr.mode = IMPISP_ANTIFLICKER_NORMAL_MODE;
#if defined(PLATFORM_T32)
			attr.freq = IMPISP_ANTIFLICKER_FREQ_50HZ;
#else
			attr.freq = 50;
#endif
			break;
		case RSS_ANTIFLICKER_60HZ:
			attr.mode = IMPISP_ANTIFLICKER_NORMAL_MODE;
#if defined(PLATFORM_T32)
			attr.freq = IMPISP_ANTIFLICKER_FREQ_60HZ;
#else
			attr.freq = 60;
#endif
			break;
		default:
			attr.mode = IMPISP_ANTIFLICKER_DISABLE_MODE;
			break;
		}
		return IMP_ISP_Tuning_SetAntiFlickerAttr(IMPVI_MAIN, &attr);
	}
#else
	/* Gen1/Gen2: IMPISPAntiflickerAttr is an enum */
	IMPISPAntiflickerAttr attr;
	switch (mode) {
	case RSS_ANTIFLICKER_50HZ:
		attr = IMPISP_ANTIFLICKER_50HZ;
		break;
	case RSS_ANTIFLICKER_60HZ:
		attr = IMPISP_ANTIFLICKER_60HZ;
		break;
	default:
		attr = IMPISP_ANTIFLICKER_DISABLE;
		break;
	}
	return IMP_ISP_Tuning_SetAntiFlickerAttr(attr);
#endif
}

/* ================================================================
 * WHITE BALANCE
 *
 * SetWB is only present on T20-T31 (gen1/gen2).
 * T32/T40/T41 use SetAwbAttr instead.
 *
 * T20-T31: IMP_ISP_Tuning_SetWB(IMPISPWB *wb)
 * T32+:    IMP_ISP_Tuning_SetAwbAttr(IMPVI_NUM, IMPISPAWBAttr*)
 *
 * For gen3 we set the RGB coefficients via Awb_SetRgbCoefft as a
 * fallback since SetWB is absent.
 * ================================================================ */

int hal_isp_set_wb(void *ctx, const rss_wb_config_t *wb_cfg)
{
	(void)ctx;

	if (!wb_cfg)
		return RSS_ERR_INVAL;

#if defined(PLATFORM_T40) || defined(PLATFORM_T41)
	/*
	 * T40/T41: Use Awb_SetRgbCoefft for manual gain control.
	 */
	if (wb_cfg->mode == RSS_WB_AUTO)
		return RSS_OK;
	IMPISPCOEFFTWB coefft;
	memset(&coefft, 0, sizeof(coefft));
	coefft.rgb_coefft_wb_r = wb_cfg->r_gain;
	coefft.rgb_coefft_wb_b = wb_cfg->b_gain;
	coefft.rgb_coefft_wb_g = wb_cfg->g_gain;
	return IMP_ISP_Tuning_Awb_SetRgbCoefft(IMPVI_MAIN, &coefft);
#elif defined(PLATFORM_T32)
	/*
	 * T32: Awb_SetRgbCoefft is absent, use SetAwbAttr instead.
	 */
	if (wb_cfg->mode == RSS_WB_AUTO)
		return RSS_OK;
	{
		IMPISPWBAttr awb_attr;
		memset(&awb_attr, 0, sizeof(awb_attr));
		return IMP_ISP_Tuning_SetAwbAttr(IMPVI_MAIN, &awb_attr);
	}
#else
	/* Gen1/Gen2: SetWB takes IMPISPWB* */
	IMPISPWB wb;
	memset(&wb, 0, sizeof(wb));
	wb.mode = (wb_cfg->mode == RSS_WB_AUTO) ? ISP_CORE_WB_MODE_AUTO
	                                        : ISP_CORE_WB_MODE_MANUAL;
	wb.rgain = wb_cfg->r_gain;
	wb.bgain = wb_cfg->b_gain;
	return IMP_ISP_Tuning_SetWB(&wb);
#endif
}

/* ================================================================
 * EXPOSURE QUERY (for RIC daemon)
 *
 * Gen1/Gen2 (T20-T31):
 *   - IMP_ISP_Tuning_GetTotalGain(uint32_t *gain)
 *   - IMP_ISP_Tuning_GetExpr(IMPISPExpr *expr) for integration time
 *   - IMP_ISP_Tuning_GetAeLuma(int *luma) on T23/T31, else from EVAttr
 *
 * Gen3 (T32/T40/T41):
 *   - IMP_ISP_Tuning_GetAeExprInfo(IMPVI_NUM, IMPISPAeExprInfo*)
 *     which provides gain + exposure time + luma in one call
 * ================================================================ */

int hal_isp_get_exposure(void *ctx, rss_exposure_t *exposure)
{
	(void)ctx;

	if (!exposure)
		return RSS_ERR_INVAL;

	memset(exposure, 0, sizeof(*exposure));

#if defined(PLATFORM_T32) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
	/*
	 * Gen3: single call to GetAeExprInfo.
	 * The struct layout is SoC-specific but includes at minimum:
	 *   - total gain (analog * digital * isp_dgain)
	 *   - exposure time in microseconds
	 *   - AE luma
	 */
	IMPISPAeExprInfo expr_info;
	memset(&expr_info, 0, sizeof(expr_info));
	int ret = IMP_ISP_Tuning_GetAeExprInfo(IMPVI_MAIN, &expr_info);
	if (ret != 0)
		return ret;

	exposure->total_gain    = expr_info.AeIntegrationTime;
	exposure->exposure_time = expr_info.AeIntegrationTime;
	exposure->ae_luma       = expr_info.AeAGain;
	/*
	 * Note: field names vary between T32/T40/T41 headers.  The above
	 * is a best-effort mapping.  Concrete field names must be verified
	 * against each SoC's header at integration time.  The important
	 * thing is that all three values come from one SDK call.
	 */
	return RSS_OK;
#else
	/* Gen1/Gen2: two separate calls */
	int ret;

	/* Total gain: [24.8] fixed-point format */
	uint32_t total_gain = 0;
	ret = IMP_ISP_Tuning_GetTotalGain(&total_gain);
	if (ret != 0)
		return ret;
	exposure->total_gain = total_gain;

	/* Exposure time from GetExpr */
	IMPISPExpr expr;
	memset(&expr, 0, sizeof(expr));
	ret = IMP_ISP_Tuning_GetExpr(&expr);
	if (ret != 0)
		return ret;
	/* g_attr.one_line_expr_in_us * integration_time gives microseconds */
	exposure->exposure_time = (uint32_t)expr.g_attr.integration_time
	    * (uint32_t)expr.g_attr.one_line_expr_in_us;

	/* AE luma */
#if defined(PLATFORM_T23) || defined(PLATFORM_T31)
	int luma = 0;
	ret = IMP_ISP_Tuning_GetAeLuma(&luma);
	if (ret == 0)
		exposure->ae_luma = (uint32_t)luma;
#else
	/* T20/T21/T30: use EV attr for luma approximation */
	IMPISPEVAttr ev_attr;
	memset(&ev_attr, 0, sizeof(ev_attr));
	ret = IMP_ISP_Tuning_GetEVAttr(&ev_attr);
	if (ret == 0)
		exposure->ae_luma = ev_attr.ev;
#endif

	return RSS_OK;
#endif
}

/* ================================================================
 * SINTER (2D denoising) STRENGTH
 *
 * Only on T20-T31.  T32/T40/T41 manage denoising via SetModule_Ratio.
 * Signature: SetSinterStrength(uint32_t ratio)
 * ================================================================ */

int hal_isp_set_sinter_strength(void *ctx, uint8_t val)
{
	(void)ctx;

	if (!g_hal_caps.has_sinter)
		return RSS_ERR_NOTSUP;

#if defined(PLATFORM_T20) || defined(PLATFORM_T21) || \
    defined(PLATFORM_T23) || defined(PLATFORM_T30) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_SetSinterStrength((uint32_t)val);
#else
	(void)val;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * TEMPER (3D denoising) STRENGTH
 *
 * Only on T20-T31.  T32/T40/T41 manage denoising via SetModule_Ratio.
 * Signature: SetTemperStrength(uint32_t ratio)
 * ================================================================ */

int hal_isp_set_temper_strength(void *ctx, uint8_t val)
{
	(void)ctx;

	if (!g_hal_caps.has_temper)
		return RSS_ERR_NOTSUP;

#if defined(PLATFORM_T20) || defined(PLATFORM_T21) || \
    defined(PLATFORM_T23) || defined(PLATFORM_T30) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_SetTemperStrength((uint32_t)val);
#else
	(void)val;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * DEFOG
 *
 * Only on T23 and T31.  Uses EnableDefog(IMPISPTuningOpsMode).
 * T20/T21/T30 have SetAntiFogAttr which is a different API entirely.
 * T32/T40/T41 do not expose individual defog control.
 * ================================================================ */

int hal_isp_set_defog(void *ctx, int enable)
{
	(void)ctx;

	if (!g_hal_caps.has_defog)
		return RSS_ERR_NOTSUP;

#if defined(PLATFORM_T23) || defined(PLATFORM_T31)
	IMPISPTuningOpsMode mode = enable ? IMPISP_TUNING_OPS_MODE_ENABLE
	                                  : IMPISP_TUNING_OPS_MODE_DISABLE;
	return IMP_ISP_Tuning_EnableDefog(mode);
#else
	(void)enable;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * DPC (Dead Pixel Correction) STRENGTH
 *
 * Only on T23 and T31.
 * Signature: SetDPC_Strength(unsigned int ratio)
 * ================================================================ */

int hal_isp_set_dpc_strength(void *ctx, uint8_t val)
{
	(void)ctx;

	if (!g_hal_caps.has_dpc)
		return RSS_ERR_NOTSUP;

#if defined(PLATFORM_T23) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_SetDPC_Strength((unsigned int)val);
#else
	(void)val;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * DRC (Dynamic Range Compression) STRENGTH
 *
 * Only on T21, T23, T31.
 * Signature: SetDRC_Strength(unsigned int ratio)
 * ================================================================ */

int hal_isp_set_drc_strength(void *ctx, uint8_t val)
{
	(void)ctx;

	if (!g_hal_caps.has_drc)
		return RSS_ERR_NOTSUP;

#if defined(PLATFORM_T21) || defined(PLATFORM_T23) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_SetDRC_Strength((unsigned int)val);
#else
	(void)val;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * AE COMPENSATION
 *
 * Only on T20, T23, T30, T31 (absent on T21).
 * Signature: SetAeComp(int comp)
 * T32/T40/T41 do not have this function.
 * ================================================================ */

int hal_isp_set_ae_comp(void *ctx, int val)
{
	(void)ctx;

#if defined(PLATFORM_T20) || defined(PLATFORM_T23) || \
    defined(PLATFORM_T30) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_SetAeComp(val);
#else
	(void)val;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * MAX ANALOG GAIN
 *
 * Only on T20-T31.
 * Signature: SetMaxAgain(uint32_t gain)
 * T32/T40/T41 do not have this function.
 * ================================================================ */

int hal_isp_set_max_again(void *ctx, uint32_t gain)
{
	(void)ctx;

#if defined(PLATFORM_T20) || defined(PLATFORM_T21) || \
    defined(PLATFORM_T23) || defined(PLATFORM_T30) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_SetMaxAgain(gain);
#else
	(void)gain;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * MAX DIGITAL GAIN
 *
 * Only on T20-T31.
 * Signature: SetMaxDgain(uint32_t gain)
 * T32/T40/T41 do not have this function.
 * ================================================================ */

int hal_isp_set_max_dgain(void *ctx, uint32_t gain)
{
	(void)ctx;

#if defined(PLATFORM_T20) || defined(PLATFORM_T21) || \
    defined(PLATFORM_T23) || defined(PLATFORM_T30) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_SetMaxDgain(gain);
#else
	(void)gain;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * HIGHLIGHT DEPRESS
 *
 * Only on T20-T31.
 * Signature: SetHiLightDepress(uint32_t strength)
 * T32/T40/T41 do not have this function.
 * ================================================================ */

int hal_isp_set_highlight_depress(void *ctx, uint8_t val)
{
	(void)ctx;

#if defined(PLATFORM_T20) || defined(PLATFORM_T21) || \
    defined(PLATFORM_T23) || defined(PLATFORM_T30) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_SetHiLightDepress((uint32_t)val);
#else
	(void)val;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * ISP GETTERS: brightness, contrast, saturation, sharpness
 *
 * Gen1/Gen2: IMP_ISP_Tuning_Get*(unsigned char *val)
 * Gen3:      IMP_ISP_Tuning_Get*(IMPVI_NUM, unsigned char *)
 *
 * All generations take a pointer for getters -- only the Gen3
 * adds IMPVI_NUM as the first parameter.
 * ================================================================ */

int hal_isp_get_brightness(void *ctx, uint8_t *val)
{
	(void)ctx;
	if (!val) return RSS_ERR_INVAL;
#if defined(HAL_ISP_PTR_ARGS)
	return IMP_ISP_Tuning_GetBrightness(IMPVI_MAIN, val);
#else
	unsigned char v;
	int ret = IMP_ISP_Tuning_GetBrightness(&v);
	*val = v;
	return ret;
#endif
}

int hal_isp_get_contrast(void *ctx, uint8_t *val)
{
	(void)ctx;
	if (!val) return RSS_ERR_INVAL;
#if defined(HAL_ISP_PTR_ARGS)
	return IMP_ISP_Tuning_GetContrast(IMPVI_MAIN, val);
#else
	unsigned char v;
	int ret = IMP_ISP_Tuning_GetContrast(&v);
	*val = v;
	return ret;
#endif
}

int hal_isp_get_saturation(void *ctx, uint8_t *val)
{
	(void)ctx;
	if (!val) return RSS_ERR_INVAL;
#if defined(HAL_ISP_PTR_ARGS)
	return IMP_ISP_Tuning_GetSaturation(IMPVI_MAIN, val);
#else
	unsigned char v;
	int ret = IMP_ISP_Tuning_GetSaturation(&v);
	*val = v;
	return ret;
#endif
}

int hal_isp_get_sharpness(void *ctx, uint8_t *val)
{
	(void)ctx;
	if (!val) return RSS_ERR_INVAL;
#if defined(HAL_ISP_PTR_ARGS)
	return IMP_ISP_Tuning_GetSharpness(IMPVI_MAIN, val);
#else
	unsigned char v;
	int ret = IMP_ISP_Tuning_GetSharpness(&v);
	*val = v;
	return ret;
#endif
}

/* ================================================================
 * HUE GETTER
 *
 * Only present on T23, T31, T32, T40, T41.
 * ================================================================ */

int hal_isp_get_hue(void *ctx, uint8_t *val)
{
	(void)ctx;
	if (!val) return RSS_ERR_INVAL;
#if defined(HAL_ISP_PTR_ARGS)
	return IMP_ISP_Tuning_GetBcshHue(IMPVI_MAIN, val);
#elif defined(PLATFORM_T23) || defined(PLATFORM_T31)
	unsigned char v;
	int ret = IMP_ISP_Tuning_GetBcshHue(&v);
	*val = v;
	return ret;
#else
	(void)val;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * HVFLIP GETTER
 *
 * T20/T21/T30: GetISPHflip + GetISPVflip (separate calls)
 * T23/T31:     GetHVFLIP(IMPISPHVFLIP*) -- combined
 * T40:         GetHVFlip(IMPVI_NUM, IMPISPHVFLIP*)
 * T32/T41:     GetHVFlip(IMPVI_NUM, IMPISPHVFLIPAttr*)
 * ================================================================ */

int hal_isp_get_hvflip(void *ctx, int *hflip, int *vflip)
{
	(void)ctx;
	if (!hflip || !vflip) return RSS_ERR_INVAL;

#if defined(PLATFORM_T20) || defined(PLATFORM_T21) || defined(PLATFORM_T30)
	IMPISPTuningOpsMode h, v;
	int ret = IMP_ISP_Tuning_GetISPHflip(&h);
	if (ret != 0) return ret;
	ret = IMP_ISP_Tuning_GetISPVflip(&v);
	if (ret != 0) return ret;
	*hflip = (h == IMPISP_TUNING_OPS_MODE_ENABLE) ? 1 : 0;
	*vflip = (v == IMPISP_TUNING_OPS_MODE_ENABLE) ? 1 : 0;
	return RSS_OK;

#elif defined(PLATFORM_T23) || defined(PLATFORM_T31)
	IMPISPHVFLIP hvflip;
	int ret = IMP_ISP_Tuning_GetHVFlip(&hvflip);
	if (ret != 0) return ret;
	*hflip = ((int)hvflip & 1) ? 1 : 0;
	*vflip = ((int)hvflip & 2) ? 1 : 0;
	return RSS_OK;

#elif defined(PLATFORM_T40)
	IMPISPHVFLIP hvf;
	int ret = IMP_ISP_Tuning_GetHVFlip(IMPVI_MAIN, &hvf);
	if (ret != 0) return ret;
	*hflip = ((int)hvf & 1) ? 1 : 0;
	*vflip = ((int)hvf & 2) ? 1 : 0;
	return RSS_OK;

#elif defined(PLATFORM_T32) || defined(PLATFORM_T41)
	IMPISPHVFLIPAttr attr;
	memset(&attr, 0, sizeof(attr));
	int ret = IMP_ISP_Tuning_GetHVFLIP(IMPVI_MAIN, &attr);
	if (ret != 0) return ret;
	*hflip = ((int)attr.sensor_mode & 1) ? 1 : 0;
	*vflip = ((int)attr.sensor_mode & 2) ? 1 : 0;
	return RSS_OK;
#endif
}

/* ================================================================
 * RUNNING MODE GETTER
 *
 * Gen1/Gen2: GetISPRunningMode(IMPISPRunningMode*)
 * Gen3:      GetISPRunningMode(IMPVI_NUM, IMPISPRunningMode*)
 * ================================================================ */

int hal_isp_get_running_mode(void *ctx, rss_isp_mode_t *mode)
{
	(void)ctx;
	if (!mode) return RSS_ERR_INVAL;

	IMPISPRunningMode imp_mode;
#if defined(HAL_ISP_PTR_ARGS)
	int ret = IMP_ISP_Tuning_GetISPRunningMode(IMPVI_MAIN, &imp_mode);
#else
	int ret = IMP_ISP_Tuning_GetISPRunningMode(&imp_mode);
#endif
	if (ret != 0) return ret;
	*mode = (imp_mode == IMPISP_RUNNING_MODE_NIGHT) ? RSS_ISP_NIGHT
	                                                : RSS_ISP_DAY;
	return RSS_OK;
}

/* ================================================================
 * SENSOR FPS GETTER
 *
 * Gen1/Gen2: GetSensorFPS(uint32_t*, uint32_t*)
 * T40:       GetSensorFPS(IMPVI_NUM, uint32_t*, uint32_t*)
 * T32/T41:   GetSensorFPS(IMPVI_NUM, uint32_t*, uint32_t*)
 * ================================================================ */

int hal_isp_get_sensor_fps(void *ctx, uint32_t *fps_num, uint32_t *fps_den)
{
	(void)ctx;
	if (!fps_num || !fps_den) return RSS_ERR_INVAL;

#if defined(PLATFORM_T40)
	return IMP_ISP_Tuning_GetSensorFPS(IMPVI_MAIN, fps_num, fps_den);
#elif defined(PLATFORM_T32) || defined(PLATFORM_T41)
	{
		IMPISPSensorFps fps;
		memset(&fps, 0, sizeof(fps));
		int ret = IMP_ISP_Tuning_GetSensorFPS(IMPVI_MAIN, &fps);
		if (ret != 0) return ret;
		*fps_num = fps.num;
		*fps_den = fps.den;
		return RSS_OK;
	}
#else
	return IMP_ISP_Tuning_GetSensorFPS(fps_num, fps_den);
#endif
}

/* ================================================================
 * ANTIFLICKER GETTER
 *
 * Gen1/Gen2: GetAntiFlickerAttr(IMPISPAntiflickerAttr*)
 *   On T31 this is an enum: DISABLE/50HZ/60HZ
 * Gen3 (T40): GetAntiFlickerAttr(IMPVI_NUM, IMPISPAntiflickerAttr*)
 *   On T40 this is a struct with .mode and .freq
 * ================================================================ */

int hal_isp_get_antiflicker(void *ctx, rss_antiflicker_t *mode)
{
	(void)ctx;
	if (!mode) return RSS_ERR_INVAL;

#if defined(PLATFORM_T32) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
	IMPISPAntiflickerAttr attr;
	memset(&attr, 0, sizeof(attr));
	int ret = IMP_ISP_Tuning_GetAntiFlickerAttr(IMPVI_MAIN, &attr);
	if (ret != 0) return ret;
	if (attr.freq == 50)
		*mode = RSS_ANTIFLICKER_50HZ;
	else if (attr.freq == 60)
		*mode = RSS_ANTIFLICKER_60HZ;
	else
		*mode = RSS_ANTIFLICKER_OFF;
	return RSS_OK;
#else
	IMPISPAntiflickerAttr attr;
	int ret = IMP_ISP_Tuning_GetAntiFlickerAttr(&attr);
	if (ret != 0) return ret;
	switch (attr) {
	case IMPISP_ANTIFLICKER_50HZ:
		*mode = RSS_ANTIFLICKER_50HZ;
		break;
	case IMPISP_ANTIFLICKER_60HZ:
		*mode = RSS_ANTIFLICKER_60HZ;
		break;
	default:
		*mode = RSS_ANTIFLICKER_OFF;
		break;
	}
	return RSS_OK;
#endif
}

/* ================================================================
 * WHITE BALANCE GETTER
 *
 * Gen1/Gen2 (T20-T31): GetWB(IMPISPWB*)
 * Gen3 (T32/T40/T41):  GetAwbAttr(IMPVI_NUM, IMPISPWBAttr*)
 * ================================================================ */

int hal_isp_get_wb(void *ctx, rss_wb_config_t *wb_cfg)
{
	(void)ctx;
	if (!wb_cfg) return RSS_ERR_INVAL;

	memset(wb_cfg, 0, sizeof(*wb_cfg));

#if defined(PLATFORM_T32) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
	IMPISPWBAttr attr;
	memset(&attr, 0, sizeof(attr));
	int ret = IMP_ISP_Tuning_GetAwbAttr(IMPVI_MAIN, &attr);
	if (ret != 0) return ret;
	wb_cfg->mode = (attr.mode == ISP_CORE_WB_MODE_AUTO) ? RSS_WB_AUTO
	                                                     : RSS_WB_MANUAL;
	wb_cfg->r_gain = attr.gain_val.rgain;
	wb_cfg->b_gain = attr.gain_val.bgain;
	return RSS_OK;
#else
	IMPISPWB wb;
	memset(&wb, 0, sizeof(wb));
	int ret = IMP_ISP_Tuning_GetWB(&wb);
	if (ret != 0) return ret;
	wb_cfg->mode = (wb.mode == ISP_CORE_WB_MODE_AUTO) ? RSS_WB_AUTO
	                                                   : RSS_WB_MANUAL;
	wb_cfg->r_gain = wb.rgain;
	wb_cfg->b_gain = wb.bgain;
	return RSS_OK;
#endif
}

/* ================================================================
 * MAX AGAIN / DGAIN GETTERS
 *
 * Only on T20-T31.
 * Signature: GetMaxAgain(uint32_t*), GetMaxDgain(uint32_t*)
 * ================================================================ */

int hal_isp_get_max_again(void *ctx, uint32_t *gain)
{
	(void)ctx;
	if (!gain) return RSS_ERR_INVAL;

#if defined(PLATFORM_T20) || defined(PLATFORM_T21) || \
    defined(PLATFORM_T23) || defined(PLATFORM_T30) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_GetMaxAgain(gain);
#else
	(void)gain;
	return RSS_ERR_NOTSUP;
#endif
}

int hal_isp_get_max_dgain(void *ctx, uint32_t *gain)
{
	(void)ctx;
	if (!gain) return RSS_ERR_INVAL;

#if defined(PLATFORM_T20) || defined(PLATFORM_T21) || \
    defined(PLATFORM_T23) || defined(PLATFORM_T30) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_GetMaxDgain(gain);
#else
	(void)gain;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * SENSOR ATTR GETTER
 *
 * Gen3 (T32/T40/T41): GetSensorAttr(IMPVI_NUM, IMPISPSENSORAttr*)
 * Gen1/Gen2: not available -- return NOTSUP
 * ================================================================ */

int hal_isp_get_sensor_attr(void *ctx, uint32_t *width, uint32_t *height)
{
	(void)ctx;
	if (!width || !height) return RSS_ERR_INVAL;

#if defined(PLATFORM_T32) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
	IMPISPSENSORAttr attr;
	memset(&attr, 0, sizeof(attr));
	int ret = IMP_ISP_Tuning_GetSensorAttr(IMPVI_MAIN, &attr);
	if (ret != 0) return ret;
	*width = attr.width;
	*height = attr.height;
	return RSS_OK;
#else
	(void)width;
	(void)height;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * AE COMP GETTER
 *
 * Only on T20, T23, T30, T31.
 * Signature: GetAeComp(int*)
 * ================================================================ */

int hal_isp_get_ae_comp(void *ctx, int *val)
{
	(void)ctx;
	if (!val) return RSS_ERR_INVAL;

#if defined(PLATFORM_T20) || defined(PLATFORM_T23) || \
    defined(PLATFORM_T30) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_GetAeComp(val);
#else
	(void)val;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * MODULE CONTROL GET / SET
 *
 * Gen1/Gen2 (T31 only): Get/SetModuleControl(IMPISPModuleCtl*)
 * Gen3 (T32/T40/T41):   Get/SetModuleControl(IMPVI_NUM, IMPISPModuleCtl*)
 * ================================================================ */

int hal_isp_get_module_control(void *ctx, uint32_t *modules)
{
	(void)ctx;
	if (!modules) return RSS_ERR_INVAL;

#if defined(PLATFORM_T32) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
	IMPISPModuleCtl ctl;
	memset(&ctl, 0, sizeof(ctl));
	int ret = IMP_ISP_Tuning_GetModuleControl(IMPVI_MAIN, &ctl);
	if (ret != 0) return ret;
	*modules = ctl.key;
	return RSS_OK;
#elif defined(PLATFORM_T31)
	IMPISPModuleCtl ctl;
	memset(&ctl, 0, sizeof(ctl));
	int ret = IMP_ISP_Tuning_GetModuleControl(&ctl);
	if (ret != 0) return ret;
	*modules = ctl.key;
	return RSS_OK;
#else
	(void)modules;
	return RSS_ERR_NOTSUP;
#endif
}

int hal_isp_set_module_control(void *ctx, uint32_t modules)
{
	(void)ctx;

#if defined(PLATFORM_T32) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
	IMPISPModuleCtl ctl;
	memset(&ctl, 0, sizeof(ctl));
	ctl.key = modules;
	return IMP_ISP_Tuning_SetModuleControl(IMPVI_MAIN, &ctl);
#elif defined(PLATFORM_T31)
	IMPISPModuleCtl ctl;
	memset(&ctl, 0, sizeof(ctl));
	ctl.key = modules;
	return IMP_ISP_Tuning_SetModuleControl(&ctl);
#else
	(void)modules;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * SINTER / TEMPER / DEFOG / DPC / DRC STRENGTH GETTERS
 *
 * T31 has GetSinterStrength/GetTemperStrength but they
 * are not in all header versions. The HAL returns NOTSUP
 * for platforms that lack the getter.
 *
 * Gen3 uses GetModule_Ratio instead.
 * ================================================================ */

int hal_isp_get_sinter_strength(void *ctx, uint8_t *val)
{
	(void)ctx;
	if (!val) return RSS_ERR_INVAL;

	/* No getter API available on any generation for sinter strength */
	(void)val;
	return RSS_ERR_NOTSUP;
}

int hal_isp_get_temper_strength(void *ctx, uint8_t *val)
{
	(void)ctx;
	if (!val) return RSS_ERR_INVAL;

	/* No getter API available on any generation for temper strength */
	(void)val;
	return RSS_ERR_NOTSUP;
}

int hal_isp_get_defog_strength(void *ctx, uint8_t *val)
{
	(void)ctx;
	if (!val) return RSS_ERR_INVAL;
	(void)val;
	return RSS_ERR_NOTSUP;
}

int hal_isp_get_dpc_strength(void *ctx, uint8_t *val)
{
	(void)ctx;
	if (!val) return RSS_ERR_INVAL;

#if defined(PLATFORM_T23) || defined(PLATFORM_T31)
	unsigned int ratio;
	int ret = IMP_ISP_Tuning_GetDPC_Strength(&ratio);
	if (ret != 0) return ret;
	*val = (uint8_t)ratio;
	return RSS_OK;
#else
	(void)val;
	return RSS_ERR_NOTSUP;
#endif
}

int hal_isp_get_drc_strength(void *ctx, uint8_t *val)
{
	(void)ctx;
	if (!val) return RSS_ERR_INVAL;

#if defined(PLATFORM_T21) || defined(PLATFORM_T23) || defined(PLATFORM_T31)
	unsigned int ratio;
	int ret = IMP_ISP_Tuning_GetDRC_Strength(&ratio);
	if (ret != 0) return ret;
	*val = (uint8_t)ratio;
	return RSS_OK;
#else
	(void)val;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * HIGHLIGHT DEPRESS GETTER
 *
 * Only on T20-T31.
 * Signature: GetHiLightDepress(uint32_t*)
 * ================================================================ */

int hal_isp_get_highlight_depress(void *ctx, uint8_t *val)
{
	(void)ctx;
	if (!val) return RSS_ERR_INVAL;

#if defined(PLATFORM_T21) || defined(PLATFORM_T23) || defined(PLATFORM_T31)
	uint32_t strength;
	int ret = IMP_ISP_Tuning_GetHiLightDepress(&strength);
	if (ret != 0) return ret;
	*val = (uint8_t)strength;
	return RSS_OK;
#else
	(void)val;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * BACKLIGHT COMP GETTER
 *
 * Only on T31.
 * Signature: GetBacklightComp(uint32_t*)
 * ================================================================ */

int hal_isp_get_backlight_comp(void *ctx, uint8_t *val)
{
	(void)ctx;
	if (!val) return RSS_ERR_INVAL;

#if defined(PLATFORM_T31)
	uint32_t strength;
	int ret = IMP_ISP_Tuning_GetBacklightComp(&strength);
	if (ret != 0) return ret;
	*val = (uint8_t)strength;
	return RSS_OK;
#else
	(void)val;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * AE WEIGHT SET / GET
 *
 * Gen1/Gen2 (T20-T31): Set/GetAeWeight(IMPISPWeight*)
 * Gen3 (T32/T40/T41):  Set/GetAeWeight(IMPVI_NUM, IMPISPAEWeightAttr*)
 *   Gen3 wraps weight inside IMPISPAEWeightAttr.ae_weight
 * ================================================================ */

int hal_isp_set_ae_weight(void *ctx, const uint8_t weight[15][15])
{
	(void)ctx;
	if (!weight) return RSS_ERR_INVAL;

#if defined(PLATFORM_T32) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
	IMPISPAEWeightAttr attr;
	memset(&attr, 0, sizeof(attr));
	attr.weight_enable = IMPISP_TUNING_OPS_MODE_ENABLE;
	memcpy(attr.ae_weight.weight, weight, sizeof(attr.ae_weight.weight));
	return IMP_ISP_Tuning_SetAeWeight(IMPVI_MAIN, &attr);
#elif defined(PLATFORM_T20) || defined(PLATFORM_T21) || \
      defined(PLATFORM_T23) || defined(PLATFORM_T30) || defined(PLATFORM_T31)
	IMPISPWeight w;
	memcpy(w.weight, weight, sizeof(w.weight));
	return IMP_ISP_Tuning_SetAeWeight(&w);
#else
	return RSS_ERR_NOTSUP;
#endif
}

int hal_isp_get_ae_weight(void *ctx, uint8_t weight[15][15])
{
	(void)ctx;
	if (!weight) return RSS_ERR_INVAL;

#if defined(PLATFORM_T32) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
	IMPISPAEWeightAttr attr;
	memset(&attr, 0, sizeof(attr));
	int ret = IMP_ISP_Tuning_GetAeWeight(IMPVI_MAIN, &attr);
	if (ret != 0) return ret;
	memcpy(weight, attr.ae_weight.weight, 15 * 15);
	return RSS_OK;
#elif defined(PLATFORM_T20) || defined(PLATFORM_T21) || \
      defined(PLATFORM_T23) || defined(PLATFORM_T30) || defined(PLATFORM_T31)
	IMPISPWeight w;
	int ret = IMP_ISP_Tuning_GetAeWeight(&w);
	if (ret != 0) return ret;
	memcpy(weight, w.weight, 15 * 15);
	return RSS_OK;
#else
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * AE ZONE GETTER
 *
 * Gen1/Gen2 (T20-T31): GetAeZone(IMPISPZone*)
 * Gen3 (T32/T40/T41):  GetAeStatistics(IMPVI_NUM, IMPISPAEStatisInfo*)
 * ================================================================ */

int hal_isp_get_ae_zone(void *ctx, uint32_t zone[15][15])
{
	(void)ctx;
	if (!zone) return RSS_ERR_INVAL;

#if defined(PLATFORM_T32) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
	IMPISPAEStatisInfo info;
	memset(&info, 0, sizeof(info));
	int ret = IMP_ISP_Tuning_GetAeStatistics(IMPVI_MAIN, &info);
	if (ret != 0) return ret;
	memcpy(zone, info.ae_statis.statis, sizeof(info.ae_statis.statis));
	return RSS_OK;
#elif defined(PLATFORM_T20)
	IMPISPAEZone ae_zone;
	int ret = IMP_ISP_Tuning_GetAeZone(&ae_zone);
	if (ret != 0) return ret;
	memcpy(zone, ae_zone.ae_sta_zone, sizeof(ae_zone.ae_sta_zone));
	return RSS_OK;
#elif defined(PLATFORM_T21) || defined(PLATFORM_T23) || defined(PLATFORM_T31)
	IMPISPZone ae_zone;
	int ret = IMP_ISP_Tuning_GetAeZone(&ae_zone);
	if (ret != 0) return ret;
	memcpy(zone, ae_zone.zone, sizeof(ae_zone.zone));
	return RSS_OK;
#else
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * AE ROI SET / GET
 *
 * Gen1/Gen2 (T20-T31): AE_SetROI / AE_GetROI(IMPISPWeight*)
 * Gen3 (T32/T40/T41):  via IMPISPAEWeightAttr.ae_roi
 * ================================================================ */

int hal_isp_set_ae_roi(void *ctx, const uint8_t roi[15][15])
{
	(void)ctx;
	if (!roi) return RSS_ERR_INVAL;

#if defined(PLATFORM_T40) || defined(PLATFORM_T41)
	{
		IMPISPAEWeightAttr attr;
		memset(&attr, 0, sizeof(attr));
		attr.roi_enable = IMPISP_TUNING_OPS_MODE_ENABLE;
		memcpy(attr.ae_roi.weight, roi, sizeof(attr.ae_roi.weight));
		return IMP_ISP_Tuning_SetAeWeight(IMPVI_MAIN, &attr);
	}
#elif defined(PLATFORM_T32)
	{
		IMPISPAEWeightAttr attr;
		memset(&attr, 0, sizeof(attr));
		attr.weight_enable = IMPISP_TUNING_OPS_MODE_ENABLE;
		memcpy(attr.ae_weight.weight, roi, sizeof(attr.ae_weight.weight));
		return IMP_ISP_Tuning_SetAeWeight(IMPVI_MAIN, &attr);
	}
#elif defined(PLATFORM_T21) || defined(PLATFORM_T23) || defined(PLATFORM_T31)
	IMPISPWeight w;
	memcpy(w.weight, roi, sizeof(w.weight));
	return IMP_ISP_Tuning_AE_SetROI(&w);
#else
	/* T20/T30: AE_SetROI uses IMPISPAERoi, not IMPISPWeight; unsupported here */
	return RSS_ERR_NOTSUP;
#endif
}

int hal_isp_get_ae_roi(void *ctx, uint8_t roi[15][15])
{
	(void)ctx;
	if (!roi) return RSS_ERR_INVAL;

#if defined(PLATFORM_T40) || defined(PLATFORM_T41)
	{
		IMPISPAEWeightAttr attr;
		memset(&attr, 0, sizeof(attr));
		int ret = IMP_ISP_Tuning_GetAeWeight(IMPVI_MAIN, &attr);
		if (ret != 0) return ret;
		memcpy(roi, attr.ae_roi.weight, 15 * 15);
		return RSS_OK;
	}
#elif defined(PLATFORM_T32)
	{
		IMPISPAEWeightAttr attr;
		memset(&attr, 0, sizeof(attr));
		int ret = IMP_ISP_Tuning_GetAeWeight(IMPVI_MAIN, &attr);
		if (ret != 0) return ret;
		memcpy(roi, attr.ae_weight.weight, 15 * 15);
		return RSS_OK;
	}
#elif defined(PLATFORM_T21) || defined(PLATFORM_T23) || defined(PLATFORM_T31)
	IMPISPWeight w;
	int ret = IMP_ISP_Tuning_AE_GetROI(&w);
	if (ret != 0) return ret;
	memcpy(roi, w.weight, 15 * 15);
	return RSS_OK;
#else
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * AE HISTOGRAM SET / GET
 *
 * Gen1/Gen2: Set/GetAeHist(IMPISPAEHist*)
 * Gen3:      via GetAeStatistics (hist embedded in AEStatisInfo)
 * ================================================================ */

int hal_isp_set_ae_hist(void *ctx, const uint8_t thresholds[4])
{
	(void)ctx;
	if (!thresholds) return RSS_ERR_INVAL;

#if defined(PLATFORM_T20) || defined(PLATFORM_T21) || \
    defined(PLATFORM_T23) || defined(PLATFORM_T30) || defined(PLATFORM_T31)
	IMPISPAEHist hist;
	memset(&hist, 0, sizeof(hist));
	memcpy(hist.ae_histhresh, thresholds, 4);
	return IMP_ISP_Tuning_SetAeHist(&hist);
#else
	(void)thresholds;
	return RSS_ERR_NOTSUP;
#endif
}

int hal_isp_get_ae_hist(void *ctx, uint8_t thresholds[4], uint16_t bins[5])
{
	(void)ctx;
	if (!thresholds || !bins) return RSS_ERR_INVAL;

#if defined(PLATFORM_T32) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
	IMPISPAEStatisInfo info;
	memset(&info, 0, sizeof(info));
	int ret = IMP_ISP_Tuning_GetAeStatistics(IMPVI_MAIN, &info);
	if (ret != 0) return ret;
	memset(thresholds, 0, 4);  /* Gen3 thresholds set via StatisConfig */
	memcpy(bins, info.ae_hist_5bin, sizeof(info.ae_hist_5bin));
	return RSS_OK;
#elif defined(PLATFORM_T20) || defined(PLATFORM_T21) || \
      defined(PLATFORM_T23) || defined(PLATFORM_T30) || defined(PLATFORM_T31)
	IMPISPAEHist hist;
	memset(&hist, 0, sizeof(hist));
	int ret = IMP_ISP_Tuning_GetAeHist(&hist);
	if (ret != 0) return ret;
	memcpy(thresholds, hist.ae_histhresh, 4);
	memcpy(bins, hist.ae_hist, sizeof(hist.ae_hist));
	return RSS_OK;
#else
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * AE HISTOGRAM ORIGIN (256-bin)
 *
 * Gen1/Gen2 (T31): GetAeHist_Origin(IMPISPAEHistOrigin*)
 * Gen3:            via GetAeStatistics.ae_hist_256bin
 * ================================================================ */

int hal_isp_get_ae_hist_origin(void *ctx, uint32_t bins[256])
{
	(void)ctx;
	if (!bins) return RSS_ERR_INVAL;

#if defined(PLATFORM_T32) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
	IMPISPAEStatisInfo info;
	memset(&info, 0, sizeof(info));
	int ret = IMP_ISP_Tuning_GetAeStatistics(IMPVI_MAIN, &info);
	if (ret != 0) return ret;
	memcpy(bins, info.ae_hist_256bin, sizeof(info.ae_hist_256bin));
	return RSS_OK;
#elif defined(PLATFORM_T31)
	IMPISPAEHistOrigin hist;
	memset(&hist, 0, sizeof(hist));
	int ret = IMP_ISP_Tuning_GetAeHist_Origin(&hist);
	if (ret != 0) return ret;
	memcpy(bins, hist.ae_hist, sizeof(hist.ae_hist));
	return RSS_OK;
#else
	(void)bins;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * AE IT MAX SET / GET
 *
 * Gen1/Gen2 (T31): SetAe_IT_MAX(uint) / GetAE_IT_MAX(uint*)
 * Gen3:            via SetAeExprInfo / GetAeExprInfo
 * ================================================================ */

int hal_isp_set_ae_it_max(void *ctx, uint32_t it_max)
{
	(void)ctx;

#if defined(PLATFORM_T32) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
	IMPISPAEExprInfo info;
	memset(&info, 0, sizeof(info));
	int ret = IMP_ISP_Tuning_GetAeExprInfo(IMPVI_MAIN, &info);
	if (ret != 0) return ret;
	info.AeMaxIntegrationTimeMode = IMPISP_TUNING_OPS_TYPE_MANUAL;
	info.AeMaxIntegrationTime = it_max;
	return IMP_ISP_Tuning_SetAeExprInfo(IMPVI_MAIN, &info);
#elif defined(PLATFORM_T31)
	return IMP_ISP_Tuning_SetAe_IT_MAX(it_max);
#else
	(void)it_max;
	return RSS_ERR_NOTSUP;
#endif
}

int hal_isp_get_ae_it_max(void *ctx, uint32_t *it_max)
{
	(void)ctx;
	if (!it_max) return RSS_ERR_INVAL;

#if defined(PLATFORM_T32) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
	IMPISPAEExprInfo info;
	memset(&info, 0, sizeof(info));
	int ret = IMP_ISP_Tuning_GetAeExprInfo(IMPVI_MAIN, &info);
	if (ret != 0) return ret;
	*it_max = info.AeMaxIntegrationTime;
	return RSS_OK;
#elif defined(PLATFORM_T31)
	return IMP_ISP_Tuning_GetAE_IT_MAX(it_max);
#else
	(void)it_max;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * AE MIN SET / GET
 *
 * Gen1/Gen2 (T31): Set/GetAeMin(IMPISPAEMin*)
 * Gen3:            via Set/GetAeExprInfo
 * ================================================================ */

int hal_isp_set_ae_min(void *ctx, int min_it, int min_again)
{
	(void)ctx;

#if defined(PLATFORM_T32) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
	IMPISPAEExprInfo info;
	memset(&info, 0, sizeof(info));
	int ret = IMP_ISP_Tuning_GetAeExprInfo(IMPVI_MAIN, &info);
	if (ret != 0) return ret;
	info.AeMinIntegrationTimeMode = IMPISP_TUNING_OPS_TYPE_MANUAL;
	info.AeMinIntegrationTime = (uint32_t)min_it;
	info.AeMinAGainMode = IMPISP_TUNING_OPS_TYPE_MANUAL;
	info.AeMinAGain = (uint32_t)min_again;
	return IMP_ISP_Tuning_SetAeExprInfo(IMPVI_MAIN, &info);
#elif defined(PLATFORM_T31)
	IMPISPAEMin ae_min;
	memset(&ae_min, 0, sizeof(ae_min));
	ae_min.min_it = (unsigned int)min_it;
	ae_min.min_again = (unsigned int)min_again;
	return IMP_ISP_Tuning_SetAeMin(&ae_min);
#else
	(void)min_it;
	(void)min_again;
	return RSS_ERR_NOTSUP;
#endif
}

int hal_isp_get_ae_min(void *ctx, int *min_it, int *min_again)
{
	(void)ctx;
	if (!min_it || !min_again) return RSS_ERR_INVAL;

#if defined(PLATFORM_T32) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
	IMPISPAEExprInfo info;
	memset(&info, 0, sizeof(info));
	int ret = IMP_ISP_Tuning_GetAeExprInfo(IMPVI_MAIN, &info);
	if (ret != 0) return ret;
	*min_it = (int)info.AeMinIntegrationTime;
	*min_again = (int)info.AeMinAGain;
	return RSS_OK;
#elif defined(PLATFORM_T31)
	IMPISPAEMin ae_min;
	memset(&ae_min, 0, sizeof(ae_min));
	int ret = IMP_ISP_Tuning_GetAeMin(&ae_min);
	if (ret != 0) return ret;
	*min_it = (int)ae_min.min_it;
	*min_again = (int)ae_min.min_again;
	return RSS_OK;
#else
	(void)min_it;
	(void)min_again;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * AWB WEIGHT SET / GET
 *
 * Gen1/Gen2: Set/GetAwbWeight(IMPISPWeight*)
 * Gen3:      Set/GetAwbWeight(IMPVI_NUM, IMPISPWeight*)
 * ================================================================ */

int hal_isp_set_awb_weight(void *ctx, const uint8_t weight[15][15])
{
	(void)ctx;
	if (!weight) return RSS_ERR_INVAL;

#if defined(PLATFORM_T32)
	{
		IMPISPAwbWeight w;
		memset(&w, 0, sizeof(w));
		/* Copy 15x15 into the 32x32 array (zero-padded) */
		int r;
		for (r = 0; r < 15; r++)
			memcpy(w.weight[r], &weight[r * 15], 15);
		return IMP_ISP_Tuning_SetAwbWeight(IMPVI_MAIN, &w);
	}
#elif defined(PLATFORM_T40) || defined(PLATFORM_T41)
	{
		IMPISPWeight w;
		memcpy(w.weight, weight, sizeof(w.weight));
		return IMP_ISP_Tuning_SetAwbWeight(IMPVI_MAIN, &w);
	}
#else
	{
		IMPISPWeight w;
		memcpy(w.weight, weight, sizeof(w.weight));
		return IMP_ISP_Tuning_SetAwbWeight(&w);
	}
#endif
}

int hal_isp_get_awb_weight(void *ctx, uint8_t weight[15][15])
{
	(void)ctx;
	if (!weight) return RSS_ERR_INVAL;

#if defined(PLATFORM_T32)
	{
		IMPISPAwbWeight w;
		memset(&w, 0, sizeof(w));
		int ret = IMP_ISP_Tuning_GetAwbWeight(IMPVI_MAIN, &w);
		if (ret != 0) return ret;
		int r;
		for (r = 0; r < 15; r++)
			memcpy(&weight[r * 15], w.weight[r], 15);
		return RSS_OK;
	}
#elif defined(PLATFORM_T40) || defined(PLATFORM_T41)
	{
		IMPISPWeight w;
		memset(&w, 0, sizeof(w));
		int ret = IMP_ISP_Tuning_GetAwbWeight(IMPVI_MAIN, &w);
		if (ret != 0) return ret;
		memcpy(weight, w.weight, 15 * 15);
		return RSS_OK;
	}
#else
	{
		IMPISPWeight w;
		memset(&w, 0, sizeof(w));
		int ret = IMP_ISP_Tuning_GetAwbWeight(&w);
		if (ret != 0) return ret;
		memcpy(weight, w.weight, 15 * 15);
		return RSS_OK;
	}
#endif
}

/* ================================================================
 * AWB ZONE GETTER
 *
 * Gen1/Gen2 (T20-T31): GetAwbZone(IMPISPAWBZone*)
 * Gen3 (T32/T40/T41):  GetAwbStatistics(IMPVI_NUM, IMPISPAWBStatisInfo*)
 * ================================================================ */

int hal_isp_get_awb_zone(void *ctx, uint8_t zone_r[225],
                          uint8_t zone_g[225], uint8_t zone_b[225])
{
	(void)ctx;
	if (!zone_r || !zone_g || !zone_b) return RSS_ERR_INVAL;

#if defined(PLATFORM_T32)
	{
		IMPISPAWBStatisInfo info;
		memset(&info, 0, sizeof(info));
		int ret = IMP_ISP_Tuning_GetAwbStatistics(IMPVI_MAIN, &info);
		if (ret != 0) return ret;
		int i;
		for (i = 0; i < 225; i++) {
			zone_r[i] = (uint8_t)info.awb_r.zone[i / 15][i % 15];
			zone_g[i] = (uint8_t)info.awb_g.zone[i / 15][i % 15];
			zone_b[i] = (uint8_t)info.awb_b.zone[i / 15][i % 15];
		}
		return RSS_OK;
	}
#elif defined(PLATFORM_T40) || defined(PLATFORM_T41)
	{
		IMPISPAWBStatisInfo info;
		memset(&info, 0, sizeof(info));
		int ret = IMP_ISP_Tuning_GetAwbStatistics(IMPVI_MAIN, &info);
		if (ret != 0) return ret;
		int i;
		for (i = 0; i < 225; i++) {
			zone_r[i] = (uint8_t)info.awb_r.statis[i / 15][i % 15];
			zone_g[i] = (uint8_t)info.awb_g.statis[i / 15][i % 15];
			zone_b[i] = (uint8_t)info.awb_b.statis[i / 15][i % 15];
		}
		return RSS_OK;
	}
#elif defined(PLATFORM_T23) || defined(PLATFORM_T31)
	IMPISPAWBZone awb_zone;
	memset(&awb_zone, 0, sizeof(awb_zone));
	int ret = IMP_ISP_Tuning_GetAwbZone(&awb_zone);
	if (ret != 0) return ret;
	memcpy(zone_r, awb_zone.zone_r, 225);
	memcpy(zone_g, awb_zone.zone_g, 225);
	memcpy(zone_b, awb_zone.zone_b, 225);
	return RSS_OK;
#else
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * AWB COLOR TEMPERATURE GETTER
 *
 * Gen3 (T32/T40/T41): via GetAwbAttr -- ct field
 * Gen1/Gen2: not directly available
 * ================================================================ */

int hal_isp_get_awb_ct(void *ctx, uint32_t *color_temp)
{
	(void)ctx;
	if (!color_temp) return RSS_ERR_INVAL;

#if defined(PLATFORM_T40) || defined(PLATFORM_T41)
	{
		IMPISPWBAttr attr;
		memset(&attr, 0, sizeof(attr));
		int ret = IMP_ISP_Tuning_GetAwbAttr(IMPVI_MAIN, &attr);
		if (ret != 0) return ret;
		*color_temp = attr.ct;
		return RSS_OK;
	}
#else
	/* T32: no ct field in IMPISPWBAttr; T20-T31: no GetAwbAttr */
	(void)color_temp;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * AWB RGB COEFFT GETTER
 *
 * Gen1/Gen2: Awb_GetRgbCoefft(IMPISPCOEFFTWB*)
 * Gen3:      Awb_GetRgbCoefft(IMPVI_NUM, IMPISPCOEFFTWB*) [T40/T41]
 * T32:       no Awb_GetRgbCoefft, use GetAwbAttr
 * ================================================================ */

int hal_isp_get_awb_rgb_coefft(void *ctx, uint16_t *rgain,
                                uint16_t *ggain, uint16_t *bgain)
{
	(void)ctx;
	if (!rgain || !ggain || !bgain) return RSS_ERR_INVAL;

#if defined(PLATFORM_T40) || defined(PLATFORM_T41)
	IMPISPCOEFFTWB coefft;
	memset(&coefft, 0, sizeof(coefft));
	int ret = IMP_ISP_Tuning_Awb_GetRgbCoefft(IMPVI_MAIN, &coefft);
	if (ret != 0) return ret;
	*rgain = coefft.rgb_coefft_wb_r;
	*ggain = coefft.rgb_coefft_wb_g;
	*bgain = coefft.rgb_coefft_wb_b;
	return RSS_OK;
#elif defined(PLATFORM_T20) || defined(PLATFORM_T21) || \
      defined(PLATFORM_T23) || defined(PLATFORM_T30) || defined(PLATFORM_T31)
	IMPISPCOEFFTWB coefft;
	memset(&coefft, 0, sizeof(coefft));
	int ret = IMP_ISP_Tuning_Awb_GetRgbCoefft(&coefft);
	if (ret != 0) return ret;
	*rgain = coefft.rgb_coefft_wb_r;
	*ggain = coefft.rgb_coefft_wb_g;
	*bgain = coefft.rgb_coefft_wb_b;
	return RSS_OK;
#else
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * AWB HIST GETTER
 *
 * Gen1/Gen2 (T20-T31): GetAwbHist(IMPISPAWBHist*)
 * Gen3: not available via same API
 * ================================================================ */

int hal_isp_get_awb_hist(void *ctx, void *hist_data)
{
	(void)ctx;
	if (!hist_data) return RSS_ERR_INVAL;

#if defined(PLATFORM_T20) || defined(PLATFORM_T21) || \
    defined(PLATFORM_T23) || defined(PLATFORM_T30) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_GetAwbHist((IMPISPAWBHist *)hist_data);
#else
	(void)hist_data;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * GAMMA SET / GET
 *
 * Gen1/Gen2 (T20-T31): Set/GetGamma(IMPISPGamma*)
 * Gen3 (T32/T40/T41):  Set/GetGammaAttr(IMPVI_NUM, IMPISPGammaAttr*)
 * ================================================================ */

int hal_isp_set_gamma(void *ctx, const uint16_t gamma[129])
{
	(void)ctx;
	if (!gamma) return RSS_ERR_INVAL;

#if defined(PLATFORM_T32) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
	IMPISPGammaAttr attr;
	memset(&attr, 0, sizeof(attr));
	attr.Curve_type = IMP_ISP_GAMMA_CURVE_USER;
	memcpy(attr.gamma, gamma, sizeof(attr.gamma));
	return IMP_ISP_Tuning_SetGammaAttr(IMPVI_MAIN, &attr);
#elif defined(PLATFORM_T20) || defined(PLATFORM_T21) || \
      defined(PLATFORM_T23) || defined(PLATFORM_T30) || defined(PLATFORM_T31)
	IMPISPGamma g;
	memcpy(g.gamma, gamma, sizeof(g.gamma));
	return IMP_ISP_Tuning_SetGamma(&g);
#else
	return RSS_ERR_NOTSUP;
#endif
}

int hal_isp_get_gamma(void *ctx, uint16_t gamma[129])
{
	(void)ctx;
	if (!gamma) return RSS_ERR_INVAL;

#if defined(PLATFORM_T32) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
	IMPISPGammaAttr attr;
	memset(&attr, 0, sizeof(attr));
	int ret = IMP_ISP_Tuning_GetGammaAttr(IMPVI_MAIN, &attr);
	if (ret != 0) return ret;
	memcpy(gamma, attr.gamma, sizeof(attr.gamma));
	return RSS_OK;
#elif defined(PLATFORM_T20) || defined(PLATFORM_T21) || \
      defined(PLATFORM_T23) || defined(PLATFORM_T30) || defined(PLATFORM_T31)
	IMPISPGamma g;
	memset(&g, 0, sizeof(g));
	int ret = IMP_ISP_Tuning_GetGamma(&g);
	if (ret != 0) return ret;
	memcpy(gamma, g.gamma, sizeof(g.gamma));
	return RSS_OK;
#else
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * CCM SET / GET
 *
 * Gen3 (T32/T40/T41): Set/GetCCMAttr(IMPVI_NUM, IMPISPCCMAttr*)
 * Gen1/Gen2: not available
 * ================================================================ */

int hal_isp_set_ccm(void *ctx, const void *ccm_attr)
{
	(void)ctx;
	if (!ccm_attr) return RSS_ERR_INVAL;

#if defined(PLATFORM_T32) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
	return IMP_ISP_Tuning_SetCCMAttr(IMPVI_MAIN,
	    (IMPISPCCMAttr *)(uintptr_t)ccm_attr);
#else
	(void)ccm_attr;
	return RSS_ERR_NOTSUP;
#endif
}

int hal_isp_get_ccm(void *ctx, void *ccm_attr)
{
	(void)ctx;
	if (!ccm_attr) return RSS_ERR_INVAL;

#if defined(PLATFORM_T32) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
	return IMP_ISP_Tuning_GetCCMAttr(IMPVI_MAIN,
	    (IMPISPCCMAttr *)ccm_attr);
#else
	(void)ccm_attr;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * WDR MODE SET / GET  (running mode alias for WDR switching)
 *
 * This maps to the running mode API -- WDR is typically toggled
 * via the ISP running mode on Gen1/Gen2.
 * For Gen3, WDR is controlled via IMP_ISP_WDR_ENABLE.
 * ================================================================ */

int hal_isp_set_wdr_mode(void *ctx, int mode)
{
	(void)ctx;

#if defined(PLATFORM_T32) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
	IMPISPTuningOpsMode ops = mode ? IMPISP_TUNING_OPS_MODE_ENABLE
	                               : IMPISP_TUNING_OPS_MODE_DISABLE;
	return IMP_ISP_WDR_ENABLE(IMPVI_MAIN, &ops);
#elif defined(PLATFORM_T31)
	IMPISPTuningOpsMode ops = mode ? IMPISP_TUNING_OPS_MODE_ENABLE
	                               : IMPISP_TUNING_OPS_MODE_DISABLE;
	return IMP_ISP_WDR_ENABLE(ops);
#else
	(void)mode;
	return RSS_ERR_NOTSUP;
#endif
}

int hal_isp_get_wdr_mode(void *ctx, int *mode)
{
	(void)ctx;
	if (!mode) return RSS_ERR_INVAL;

#if defined(PLATFORM_T32) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
	IMPISPTuningOpsMode ops;
	int ret = IMP_ISP_WDR_ENABLE_GET(IMPVI_MAIN, &ops);
	if (ret != 0) return ret;
	*mode = (ops == IMPISP_TUNING_OPS_MODE_ENABLE) ? 1 : 0;
	return RSS_OK;
#elif defined(PLATFORM_T31)
	IMPISPTuningOpsMode ops;
	int ret = IMP_ISP_WDR_ENABLE_Get(&ops);
	if (ret != 0) return ret;
	*mode = (ops == IMPISP_TUNING_OPS_MODE_ENABLE) ? 1 : 0;
	return RSS_OK;
#else
	(void)mode;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * WDR ENABLE / GET ENABLE
 *
 * Aliases for set/get_wdr_mode for explicit enable/disable semantics.
 * ================================================================ */

int hal_isp_wdr_enable(void *ctx, int enable)
{
	return hal_isp_set_wdr_mode(ctx, enable);
}

int hal_isp_wdr_get_enable(void *ctx, int *enabled)
{
	return hal_isp_get_wdr_mode(ctx, enabled);
}

/* ================================================================
 * ISP BYPASS
 *
 * Gen1/Gen2 (T31): SetISPBypass(IMPISPTuningOpsMode)
 * Gen3:            SetISPBypass(IMPVI_NUM, IMPISPTuningOpsMode*)
 * ================================================================ */

int hal_isp_set_bypass(void *ctx, int enable)
{
	(void)ctx;

#if defined(PLATFORM_T40)
	IMPISPTuningOpsMode mode = enable ? IMPISP_TUNING_OPS_MODE_ENABLE
	                                  : IMPISP_TUNING_OPS_MODE_DISABLE;
	return IMP_ISP_Tuning_SetISPBypass(IMPVI_MAIN, &mode);
#elif defined(PLATFORM_T32) || defined(PLATFORM_T41)
	(void)enable;
	return RSS_ERR_NOTSUP;
#elif defined(PLATFORM_T20) || defined(PLATFORM_T21) || \
      defined(PLATFORM_T23) || defined(PLATFORM_T30) || defined(PLATFORM_T31)
	IMPISPTuningOpsMode mode = enable ? IMPISP_TUNING_OPS_MODE_ENABLE
	                                  : IMPISP_TUNING_OPS_MODE_DISABLE;
	return IMP_ISP_Tuning_SetISPBypass(mode);
#else
	(void)enable;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * DEFAULT BIN PATH SET / GET
 *
 * Gen1/Gen2 (T31): SetDefaultBinPath(char*)
 * Gen3:            SetDefaultBinPath(IMPVI_NUM, char*)
 * ================================================================ */

int hal_isp_set_default_bin_path(void *ctx, const char *path)
{
	(void)ctx;
	if (!path) return RSS_ERR_INVAL;

#if defined(PLATFORM_T32)
	{
		IMPISPDefaultBinAttr attr;
		memset(&attr, 0, sizeof(attr));
		strncpy(attr.bname, path, sizeof(attr.bname) - 1);
		return IMP_ISP_SetDefaultBinPath(IMPVI_MAIN, &attr);
	}
#elif defined(PLATFORM_T40) || defined(PLATFORM_T41)
	return IMP_ISP_SetDefaultBinPath(IMPVI_MAIN, (char *)(uintptr_t)path);
#elif defined(PLATFORM_T23) || defined(PLATFORM_T31)
	return IMP_ISP_SetDefaultBinPath((char *)(uintptr_t)path);
#else
	(void)path;
	return RSS_ERR_NOTSUP;
#endif
}

int hal_isp_get_default_bin_path(void *ctx, char *path, int path_len)
{
	(void)ctx;
	if (!path || path_len <= 0) return RSS_ERR_INVAL;

#if defined(PLATFORM_T32)
	{
		IMPISPDefaultBinAttr attr;
		memset(&attr, 0, sizeof(attr));
		int ret = IMP_ISP_GetDefaultBinPath(IMPVI_MAIN, &attr);
		if (ret != 0) return ret;
		strncpy(path, attr.bname, (size_t)path_len - 1);
		path[path_len - 1] = '\0';
		return RSS_OK;
	}
#elif defined(PLATFORM_T40) || defined(PLATFORM_T41)
	return IMP_ISP_GetDefaultBinPath(IMPVI_MAIN, path);
#elif defined(PLATFORM_T23) || defined(PLATFORM_T31)
	return IMP_ISP_GetDefaultBinPath(path);
#else
	(void)path;
	(void)path_len;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * FRAME DROP SET / GET
 *
 * T23+T31+T32+T40+T41 only.
 * Gen2 (T23/T31): Set/GetFrameDrop(IMPISPFrameDrop*)
 * Gen3 (T32/T40/T41): Set/GetFrameDrop(IMPVI_NUM, IMPISPFrameDrop*)
 * ================================================================ */

int hal_isp_set_frame_drop(void *ctx, int drop)
{
	(void)ctx;

#if defined(PLATFORM_T32) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
	IMPISPFrameDropAttr fd;
	memset(&fd, 0, sizeof(fd));
	/* Set frame drop on channel 0 */
	fd.fdrop[0].enable = drop ? IMPISP_TUNING_OPS_MODE_ENABLE
	                          : IMPISP_TUNING_OPS_MODE_DISABLE;
	fd.fdrop[0].lsize = (uint8_t)drop;
	return IMP_ISP_SetFrameDrop(IMPVI_MAIN, &fd);
#elif defined(PLATFORM_T23) || defined(PLATFORM_T31)
	/* FrameDrop uses opaque struct — caller must pass pre-built attr */
	(void)drop;
	return RSS_ERR_NOTSUP; /* TODO: expose via void* API */
#else
	(void)drop;
	return RSS_ERR_NOTSUP;
#endif
}

int hal_isp_get_frame_drop(void *ctx, int *drop)
{
	(void)ctx;
	if (!drop) return RSS_ERR_INVAL;

#if defined(PLATFORM_T32) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
	IMPISPFrameDropAttr fd;
	memset(&fd, 0, sizeof(fd));
	int ret = IMP_ISP_GetFrameDrop(IMPVI_MAIN, &fd);
	if (ret != 0) return ret;
	*drop = (int)fd.fdrop[0].lsize;
	return RSS_OK;
#elif defined(PLATFORM_T23) || defined(PLATFORM_T31)
	/* FrameDrop uses opaque struct — caller must use void* API */
	(void)drop;
	return RSS_ERR_NOTSUP; /* TODO: expose via void* API */
#else
	(void)drop;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * SENSOR REGISTER SET / GET
 *
 * Gen1/Gen2: SetSensorRegister(uint32_t reg, uint32_t val)
 *            GetSensorRegister(uint32_t reg, uint32_t *val)
 * Gen3:      SetSensorRegister(IMPVI_NUM, uint32_t*, uint32_t*)
 *            GetSensorRegister(IMPVI_NUM, uint32_t*, uint32_t*)
 * ================================================================ */

int hal_isp_set_sensor_register(void *ctx, uint32_t reg, uint32_t val)
{
	(void)ctx;

#if defined(PLATFORM_T40)
	/* T40: separate uint32_t* args */
	return IMP_ISP_SetSensorRegister(IMPVI_MAIN, &reg, &val);
#elif defined(PLATFORM_T32) || defined(PLATFORM_T41)
	/* T32/T41: IMPISPSensorRegister struct */
	{
		IMPISPSensorRegister sr;
		sr.addr = reg;
		sr.value = val;
		return IMP_ISP_SetSensorRegister(IMPVI_MAIN, &sr);
	}
#else
	return IMP_ISP_SetSensorRegister(reg, val);
#endif
}

int hal_isp_get_sensor_register(void *ctx, uint32_t reg, uint32_t *val)
{
	(void)ctx;
	if (!val) return RSS_ERR_INVAL;

#if defined(PLATFORM_T40)
	/* T40: separate uint32_t* args */
	return IMP_ISP_GetSensorRegister(IMPVI_MAIN, &reg, val);
#elif defined(PLATFORM_T32) || defined(PLATFORM_T41)
	/* T32/T41: IMPISPSensorRegister struct */
	{
		IMPISPSensorRegister sr;
		sr.addr = reg;
		sr.value = 0;
		int ret = IMP_ISP_GetSensorRegister(IMPVI_MAIN, &sr);
		if (ret != 0) return ret;
		*val = sr.value;
		return RSS_OK;
	}
#else
	return IMP_ISP_GetSensorRegister(reg, val);
#endif
}

/* ================================================================
 * AUTO ZOOM
 *
 * Gen1/Gen2 (T31): SetAutoZoom(IMPISPAutoZoom*)
 * Gen3: not available via same API
 * ================================================================ */

int hal_isp_set_auto_zoom(void *ctx, const void *zoom_attr)
{
	(void)ctx;
	if (!zoom_attr) return RSS_ERR_INVAL;

#if defined(PLATFORM_T23) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_SetAutoZoom(
	    (IMPISPAutoZoom *)(uintptr_t)zoom_attr);
#else
	(void)zoom_attr;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * VIDEO DROP CALLBACK
 *
 * All SoCs: SetVideoDrop(void (*cb)(void))
 * Gen3 (T40): SetVideoDrop(void (*cb)(void)) -- same signature
 * ================================================================ */

int hal_isp_set_video_drop(void *ctx, void (*callback)(void))
{
	(void)ctx;
	if (!callback) return RSS_ERR_INVAL;

	return IMP_ISP_Tuning_SetVideoDrop(callback);
}

/* ================================================================
 * MASK SET / GET
 *
 * T23+T31+T40 only.
 * Gen2 (T23/T31): Set/GetMask(IMPISPMASKAttr*)
 * Gen3 (T40):     Set/GetMask(IMPVI_NUM, IMPISPMASKAttr*)
 * ================================================================ */

int hal_isp_set_mask(void *ctx, const void *mask_attr)
{
	(void)ctx;
	if (!mask_attr) return RSS_ERR_INVAL;

#if defined(PLATFORM_T40)
	return IMP_ISP_Tuning_SetMask(IMPVI_MAIN,
	    (IMPISPMASKAttr *)(uintptr_t)mask_attr);
#elif defined(PLATFORM_T23) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_SetMask(
	    (IMPISPMASKAttr *)(uintptr_t)mask_attr);
#else
	(void)mask_attr;
	return RSS_ERR_NOTSUP;
#endif
}

int hal_isp_get_mask(void *ctx, void *mask_attr)
{
	(void)ctx;
	if (!mask_attr) return RSS_ERR_INVAL;

#if defined(PLATFORM_T40)
	return IMP_ISP_Tuning_GetMask(IMPVI_MAIN,
	    (IMPISPMASKAttr *)mask_attr);
#elif defined(PLATFORM_T23) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_GetMask((IMPISPMASKAttr *)mask_attr);
#else
	(void)mask_attr;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * FRAME DROP SET / GET (vendor API)
 *
 * T23+T31+T32+T40+T41 only.
 * Gen2 (T23/T31): SetFrameDrop(IMPISPFrameDrop*)
 * Gen3 (T32/T40/T41): SetFrameDrop(IMPVI_NUM, IMPISPFrameDrop*)
 *
 * Re-implemented to use actual vendor functions.
 * ================================================================ */

/* ================================================================
 * SET EXPR (SetExpr)
 *
 * T23+T31 only.
 * Signature: SetExpr(IMPISPExpr*)
 * ================================================================ */

int hal_isp_set_expr(void *ctx, const void *expr_attr)
{
	(void)ctx;
	if (!expr_attr) return RSS_ERR_INVAL;

#if defined(PLATFORM_T23) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_SetExpr(
	    (IMPISPExpr *)(uintptr_t)expr_attr);
#else
	(void)expr_attr;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * AE ATTR GET / SET
 *
 * T23+T31 only.
 * Signature: Get/SetAeAttr(IMPISPAEAttr*)
 * ================================================================ */

int hal_isp_get_ae_attr(void *ctx, void *ae_attr)
{
	(void)ctx;
	if (!ae_attr) return RSS_ERR_INVAL;

#if defined(PLATFORM_T23) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_GetAeAttr((IMPISPAEAttr *)ae_attr);
#else
	(void)ae_attr;
	return RSS_ERR_NOTSUP;
#endif
}

int hal_isp_set_ae_attr(void *ctx, const void *ae_attr)
{
	(void)ctx;
	if (!ae_attr) return RSS_ERR_INVAL;

#if defined(PLATFORM_T23) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_SetAeAttr(
	    (IMPISPAEAttr *)(uintptr_t)ae_attr);
#else
	(void)ae_attr;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * AE STATE GETTER
 *
 * T23+T31 only.
 * Signature: GetAeState(IMPISPAEState*)
 * ================================================================ */

int hal_isp_get_ae_state(void *ctx, void *ae_state)
{
	(void)ctx;
	if (!ae_state) return RSS_ERR_INVAL;

#if defined(PLATFORM_T23) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_GetAeState((IMPISPAEState *)ae_state);
#else
	(void)ae_state;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * AE TARGET LIST GET / SET
 *
 * T23+T31 only.
 * Signature: Get/SetAeTargetList(IMPISPAETargetList*)
 * ================================================================ */

int hal_isp_get_ae_target_list(void *ctx, void *target_list)
{
	(void)ctx;
	if (!target_list) return RSS_ERR_INVAL;

#if defined(PLATFORM_T23) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_GetAeTargetList(
	    (IMPISPAETargetList *)target_list);
#else
	(void)target_list;
	return RSS_ERR_NOTSUP;
#endif
}

int hal_isp_set_ae_target_list(void *ctx, const void *target_list)
{
	(void)ctx;
	if (!target_list) return RSS_ERR_INVAL;

#if defined(PLATFORM_T23) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_SetAeTargetList(
	    (IMPISPAETargetList *)(uintptr_t)target_list);
#else
	(void)target_list;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * AE FREEZE
 *
 * T23+T31 only.
 * Signature: SetAeFreeze(IMPISPTuningOpsMode)
 * ================================================================ */

int hal_isp_set_ae_freeze(void *ctx, int enable)
{
	(void)ctx;

#if defined(PLATFORM_T23) || defined(PLATFORM_T31)
	IMPISPTuningOpsMode mode = enable ? IMPISP_TUNING_OPS_MODE_ENABLE
	                                  : IMPISP_TUNING_OPS_MODE_DISABLE;
	return IMP_ISP_Tuning_SetAeFreeze(mode);
#else
	(void)enable;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * AF ZONE GETTER
 *
 * T23+T31 only.
 * Signature: GetAfZone(void*)
 * ================================================================ */

int hal_isp_get_af_zone(void *ctx, void *af_zone)
{
	(void)ctx;
	if (!af_zone) return RSS_ERR_INVAL;

#if defined(PLATFORM_T23) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_GetAfZone((void *)af_zone);
#else
	(void)af_zone;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * AWB CLUST GET / SET
 *
 * T23+T31 only.
 * Signature: Get/SetAwbClust(void*)
 * ================================================================ */

int hal_isp_get_awb_clust(void *ctx, void *clust)
{
	(void)ctx;
	if (!clust) return RSS_ERR_INVAL;

#if defined(PLATFORM_T23) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_GetAwbClust((void *)clust);
#else
	(void)clust;
	return RSS_ERR_NOTSUP;
#endif
}

int hal_isp_set_awb_clust(void *ctx, const void *clust)
{
	(void)ctx;
	if (!clust) return RSS_ERR_INVAL;

#if defined(PLATFORM_T23) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_SetAwbClust(
	    (void *)(uintptr_t)clust);
#else
	(void)clust;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * AWB CT (Color Temperature) GET / SET
 *
 * T23+T31 only.
 * Signature: GetAWBCt(void*) / SetAwbCt(void*)
 * ================================================================ */

int hal_isp_get_awb_ct_attr(void *ctx, void *ct_attr)
{
	(void)ctx;
	if (!ct_attr) return RSS_ERR_INVAL;

#if defined(PLATFORM_T23) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_GetAWBCt((void *)ct_attr);
#else
	(void)ct_attr;
	return RSS_ERR_NOTSUP;
#endif
}

int hal_isp_set_awb_ct_attr(void *ctx, const void *ct_attr)
{
	(void)ctx;
	if (!ct_attr) return RSS_ERR_INVAL;

#if defined(PLATFORM_T23) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_SetAwbCt(
	    (void *)(uintptr_t)ct_attr);
#else
	(void)ct_attr;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * AWB CT TREND GET / SET
 *
 * T23+T31 only.
 * Signature: Get/SetAwbCtTrend(IMPISPAWBCtTrend*)
 * ================================================================ */

int hal_isp_get_awb_ct_trend(void *ctx, void *trend)
{
	(void)ctx;
	if (!trend) return RSS_ERR_INVAL;

#if defined(PLATFORM_T23) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_GetAwbCtTrend((IMPISPAWBCtTrend *)trend);
#else
	(void)trend;
	return RSS_ERR_NOTSUP;
#endif
}

int hal_isp_set_awb_ct_trend(void *ctx, const void *trend)
{
	(void)ctx;
	if (!trend) return RSS_ERR_INVAL;

#if defined(PLATFORM_T23) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_SetAwbCtTrend(
	    (IMPISPAWBCtTrend *)(uintptr_t)trend);
#else
	(void)trend;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * BACKLIGHT COMP SETTER
 *
 * T23+T31 only.
 * Signature: SetBacklightComp(uint32_t strength)
 * ================================================================ */

int hal_isp_set_backlight_comp(void *ctx, uint32_t strength)
{
	(void)ctx;

#if defined(PLATFORM_T23) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_SetBacklightComp(strength);
#else
	(void)strength;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * DEFOG STRENGTH (advanced attr version)
 *
 * T23+T31 only.
 * Signature: Get/SetDefog_Strength(void*)
 * ================================================================ */

int hal_isp_get_defog_strength_adv(void *ctx, void *defog_attr)
{
	(void)ctx;
	if (!defog_attr) return RSS_ERR_INVAL;

#if defined(PLATFORM_T23) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_GetDefog_Strength(
	    (void *)defog_attr);
#else
	(void)defog_attr;
	return RSS_ERR_NOTSUP;
#endif
}

int hal_isp_set_defog_strength_adv(void *ctx, const void *defog_attr)
{
	(void)ctx;
	if (!defog_attr) return RSS_ERR_INVAL;

#if defined(PLATFORM_T23) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_SetDefog_Strength(
	    (void *)(uintptr_t)defog_attr);
#else
	(void)defog_attr;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * FRONT CROP GET / SET
 *
 * T23+T31 only.
 * Signature: Get/SetFrontCrop(IMPISPFrontCrop*)
 * ================================================================ */

int hal_isp_get_front_crop(void *ctx, void *crop_attr)
{
	(void)ctx;
	if (!crop_attr) return RSS_ERR_INVAL;

#if defined(PLATFORM_T23) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_GetFrontCrop((IMPISPFrontCrop *)crop_attr);
#else
	(void)crop_attr;
	return RSS_ERR_NOTSUP;
#endif
}

int hal_isp_set_front_crop(void *ctx, const void *crop_attr)
{
	(void)ctx;
	if (!crop_attr) return RSS_ERR_INVAL;

#if defined(PLATFORM_T23) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_SetFrontCrop(
	    (IMPISPFrontCrop *)(uintptr_t)crop_attr);
#else
	(void)crop_attr;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * BLC ATTR GETTER
 *
 * T23+T31 only.
 * Signature: GetBlcAttr(IMPISPBlcAttr*)
 * ================================================================ */

int hal_isp_get_blc_attr(void *ctx, void *blc_attr)
{
	(void)ctx;
	if (!blc_attr) return RSS_ERR_INVAL;

#if defined(PLATFORM_T23) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_GetBlcAttr((IMPISPBlcAttr *)blc_attr);
#else
	(void)blc_attr;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * CSC ATTR GET / SET
 *
 * T23+T31 only.
 * Signature: Get/SetCsc_Attr(IMPISPCscAttr*)
 * ================================================================ */

int hal_isp_get_csc_attr(void *ctx, void *csc_attr)
{
	(void)ctx;
	if (!csc_attr) return RSS_ERR_INVAL;

#if defined(PLATFORM_T23) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_GetCsc_Attr((IMPISPCscAttr *)csc_attr);
#else
	(void)csc_attr;
	return RSS_ERR_NOTSUP;
#endif
}

int hal_isp_set_csc_attr(void *ctx, const void *csc_attr)
{
	(void)ctx;
	if (!csc_attr) return RSS_ERR_INVAL;

#if defined(PLATFORM_T23) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_SetCsc_Attr(
	    (IMPISPCscAttr *)(uintptr_t)csc_attr);
#else
	(void)csc_attr;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * ISP CUSTOM MODE SET / GET
 *
 * T23+T31 only.
 * Signature: Set/GetISPCustomMode(IMPISPTuningOpsMode)
 * ================================================================ */

int hal_isp_set_custom_mode(void *ctx, int mode)
{
	(void)ctx;

#if defined(PLATFORM_T23) || defined(PLATFORM_T31)
	IMPISPTuningOpsMode ops = mode ? IMPISP_TUNING_OPS_MODE_ENABLE
	                               : IMPISP_TUNING_OPS_MODE_DISABLE;
	return IMP_ISP_Tuning_SetISPCustomMode(ops);
#else
	(void)mode;
	return RSS_ERR_NOTSUP;
#endif
}

int hal_isp_get_custom_mode(void *ctx, int *mode)
{
	(void)ctx;
	if (!mode) return RSS_ERR_INVAL;

#if defined(PLATFORM_T23) || defined(PLATFORM_T31)
	IMPISPTuningOpsMode ops;
	int ret = IMP_ISP_Tuning_GetISPCustomMode(&ops);
	if (ret != 0) return ret;
	*mode = (ops == IMPISP_TUNING_OPS_MODE_ENABLE) ? 1 : 0;
	return RSS_OK;
#else
	(void)mode;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * ENABLE DRC
 *
 * T23+T31 only.
 * Signature: EnableDRC(IMPISPTuningOpsMode)
 * ================================================================ */

int hal_isp_enable_drc(void *ctx, int enable)
{
	(void)ctx;

#if defined(PLATFORM_T23) || defined(PLATFORM_T31)
	IMPISPTuningOpsMode mode = enable ? IMPISP_TUNING_OPS_MODE_ENABLE
	                                  : IMPISP_TUNING_OPS_MODE_DISABLE;
	return IMP_ISP_Tuning_EnableDRC(mode);
#else
	(void)enable;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * AF HIST GET / SET (T20-T31, not T32+)
 *
 * Gen1/Gen2: Get/SetAfHist(IMPISPAFHist*)
 * ================================================================ */

int hal_isp_get_af_hist(void *ctx, void *af_hist)
{
	(void)ctx;
	if (!af_hist) return RSS_ERR_INVAL;

#if defined(PLATFORM_T20) || defined(PLATFORM_T21) || \
    defined(PLATFORM_T23) || defined(PLATFORM_T30) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_GetAfHist((IMPISPAFHist *)af_hist);
#else
	(void)af_hist;
	return RSS_ERR_NOTSUP;
#endif
}

int hal_isp_set_af_hist(void *ctx, const void *af_hist)
{
	(void)ctx;
	if (!af_hist) return RSS_ERR_INVAL;

#if defined(PLATFORM_T20) || defined(PLATFORM_T21) || \
    defined(PLATFORM_T23) || defined(PLATFORM_T30) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_SetAfHist(
	    (IMPISPAFHist *)(uintptr_t)af_hist);
#else
	(void)af_hist;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * AF METRICS GETTER (T20-T31, not T32+)
 *
 * Gen1/Gen2: GetAFMetrices(void*)
 * ================================================================ */

int hal_isp_get_af_metrics(void *ctx, void *metrics)
{
	(void)ctx;
	if (!metrics) return RSS_ERR_INVAL;

#if defined(PLATFORM_T21) || defined(PLATFORM_T23) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_GetAFMetrices((void *)metrics);
#else
	(void)metrics;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * MOVESTATE ENABLE / DISABLE (T20-T31, not T32+)
 *
 * Gen1/Gen2: EnableMovestate / DisableMovestate
 * ================================================================ */

int hal_isp_enable_movestate(void *ctx)
{
	(void)ctx;

#if defined(PLATFORM_T20) || defined(PLATFORM_T21) || \
    defined(PLATFORM_T23) || defined(PLATFORM_T30) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_EnableMovestate();
#else
	return RSS_ERR_NOTSUP;
#endif
}

int hal_isp_disable_movestate(void *ctx)
{
	(void)ctx;

#if defined(PLATFORM_T20) || defined(PLATFORM_T21) || \
    defined(PLATFORM_T23) || defined(PLATFORM_T30) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_DisableMovestate();
#else
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * SHADING SET (T20-T31, not T32+)
 *
 * Gen1/Gen2: SetShading(IMPISPShading*)
 * ================================================================ */

int hal_isp_set_shading(void *ctx, const void *shading_attr)
{
	(void)ctx;
	if (!shading_attr) return RSS_ERR_INVAL;

	/* Shading control not yet implemented */
	return RSS_ERR_NOTSUP;
}








/* ================================================================
 * WAIT FRAME (T20-T31, not T32+)
 *
 * Gen1/Gen2: WaitFrame(int timeout_ms)
 * ================================================================ */

int hal_isp_wait_frame(void *ctx, int timeout_ms)
{
	(void)ctx;
	(void)timeout_ms;

#if defined(PLATFORM_T20) || defined(PLATFORM_T21) || \
    defined(PLATFORM_T23) || defined(PLATFORM_T30) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_WaitFrame(NULL);
#else
	(void)timeout_ms;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * AF WEIGHT GET / SET (T21+T23+T31+T32+T40+T41)
 *
 * Gen1/Gen2 (T21/T23/T31): Get/SetAfWeight(IMPISPWeight*)
 * Gen3 (T32/T40/T41):      Get/SetAfWeight(IMPVI_NUM, IMPISPWeight*)
 * ================================================================ */

int hal_isp_get_af_weight(void *ctx, void *af_weight)
{
	(void)ctx;
	if (!af_weight) return RSS_ERR_INVAL;

#if defined(PLATFORM_T32) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
	return IMP_ISP_Tuning_GetAfWeight(IMPVI_MAIN,
	    (IMPISPWeight *)af_weight);
#elif defined(PLATFORM_T21) || defined(PLATFORM_T23) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_GetAfWeight((IMPISPWeight *)af_weight);
#else
	(void)af_weight;
	return RSS_ERR_NOTSUP;
#endif
}

int hal_isp_set_af_weight(void *ctx, const void *af_weight)
{
	(void)ctx;
	if (!af_weight) return RSS_ERR_INVAL;

#if defined(PLATFORM_T32) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
	return IMP_ISP_Tuning_SetAfWeight(IMPVI_MAIN,
	    (IMPISPWeight *)(uintptr_t)af_weight);
#elif defined(PLATFORM_T21) || defined(PLATFORM_T23) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_SetAfWeight(
	    (IMPISPWeight *)(uintptr_t)af_weight);
#else
	(void)af_weight;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * WB STATIS GETTER (T20-T31, not T32+)
 *
 * Gen1/Gen2: GetWB_Statis(IMPISPWB*)
 * ================================================================ */

int hal_isp_get_wb_statis(void *ctx, void *wb_statis)
{
	(void)ctx;
	if (!wb_statis) return RSS_ERR_INVAL;

#if defined(PLATFORM_T20) || defined(PLATFORM_T21) || \
    defined(PLATFORM_T23) || defined(PLATFORM_T30) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_GetWB_Statis((IMPISPWB *)wb_statis);
#else
	(void)wb_statis;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * AWB HIST SET (advanced version, T20-T31, not T32+)
 *
 * Gen1/Gen2: SetAwbHist(IMPISPAWBHist*)
 * ================================================================ */

int hal_isp_set_awb_hist_adv(void *ctx, const void *awb_hist)
{
	(void)ctx;
	if (!awb_hist) return RSS_ERR_INVAL;

#if defined(PLATFORM_T20) || defined(PLATFORM_T21) || \
    defined(PLATFORM_T23) || defined(PLATFORM_T30) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_SetAwbHist(
	    (IMPISPAWBHist *)(uintptr_t)awb_hist);
#else
	(void)awb_hist;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * WB GOL STATIS GETTER (T21+T23+T31)
 *
 * Signature: GetWB_GOL_Statis(IMPISPWB*)
 * ================================================================ */

int hal_isp_get_wb_gol_statis(void *ctx, void *gol_statis)
{
	(void)ctx;
	if (!gol_statis) return RSS_ERR_INVAL;

#if defined(PLATFORM_T21) || defined(PLATFORM_T23) || defined(PLATFORM_T31)
	return IMP_ISP_Tuning_GetWB_GOL_Statis((IMPISPWB *)gol_statis);
#else
	(void)gol_statis;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * WDR OUTPUT MODE SET / GET (T31 only)
 *
 * Signature: SetWdr_OutputMode(IMPISPWdrOutputMode) /
 *            GetWdr_OutputMode(IMPISPWdrOutputMode*)
 * ================================================================ */

int hal_isp_set_wdr_output_mode(void *ctx, int mode)
{
	(void)ctx;

#if defined(PLATFORM_T31)
	IMPISPWdrOutputMode wdr_mode = (IMPISPWdrOutputMode)mode;
	return IMP_ISP_Tuning_SetWdr_OutputMode(&wdr_mode);
#else
	(void)mode;
	return RSS_ERR_NOTSUP;
#endif
}

int hal_isp_get_wdr_output_mode(void *ctx, int *mode)
{
	(void)ctx;
	if (!mode) return RSS_ERR_INVAL;

#if defined(PLATFORM_T31)
	IMPISPWdrOutputMode wdr_mode;
	int ret = IMP_ISP_Tuning_GetWdr_OutputMode(&wdr_mode);
	if (ret != 0) return ret;
	*mode = (int)wdr_mode;
	return RSS_OK;
#else
	(void)mode;
	return RSS_ERR_NOTSUP;
#endif
}

/* ================================================================
 * SCALER LEVEL SET (T23+T31+T32+T41, not T20/T21/T30/T40)
 *
 * Gen2 (T23/T31): SetScalerLv(int chn, int level)
 * Gen3 (T32/T41): SetScalerLv(IMPVI_NUM, int chn, int level)
 * ================================================================ */

int hal_isp_set_scaler_lv(void *ctx, int chn, int level)
{
	(void)ctx;

#if defined(PLATFORM_T32) || defined(PLATFORM_T41)
	{
		IMPISPScalerLvAttr attr;
		memset(&attr, 0, sizeof(attr));
		attr.chx = (uint8_t)chn;
		attr.mode = IMPISP_SCALER_FIXED_WEIGHT;
		attr.level = (uint8_t)level;
		return IMP_ISP_Tuning_SetScalerLv(IMPVI_MAIN, &attr);
	}
#elif defined(PLATFORM_T23) || defined(PLATFORM_T31)
	IMPISPScalerLv lv;
	memset(&lv, 0, sizeof(lv));
	(void)chn;
	(void)level;
	return IMP_ISP_Tuning_SetScalerLv(&lv);
#else
	(void)chn;
	(void)level;
	return RSS_ERR_NOTSUP;
#endif
}
