/*
 * hal_common.c -- Raptor HAL common implementation
 *
 * Contains the ops vtable, factory functions, system lifecycle
 * (init/deinit), bind/unbind, and capability query.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: MIT
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <errno.h>

#include "hal_internal.h"

/* ── Default log function (stderr) ── */

static const char *hal_level_str[] = {"FTL", "ERR", "WRN", "INF", "DBG"};

static void hal_log_stderr(int level, const char *file, int line, const char *fmt, ...)
{
    if (level < 0)
        level = 0;
    if (level > 4)
        level = 4;
    fprintf(stderr, "[HAL %s] %s:%d: ", hal_level_str[level], file, line);
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputc('\n', stderr);
}

rss_hal_log_func_t rss_hal_log_fn = hal_log_stderr;

void rss_hal_set_log_func(rss_hal_log_func_t func)
{
    rss_hal_log_fn = func ? func : hal_log_stderr;
}

/* ── External function declarations (implemented in other files) ── */

/* Encoder (hal_encoder_old.c / hal_encoder_new.c) */
extern int hal_enc_create_group(void *ctx, int grp);
extern int hal_enc_destroy_group(void *ctx, int grp);
extern int hal_enc_create_channel(void *ctx, int chn, const rss_video_config_t *cfg);
extern int hal_enc_destroy_channel(void *ctx, int chn);
extern int hal_enc_register_channel(void *ctx, int grp, int chn);
extern int hal_enc_unregister_channel(void *ctx, int chn);
extern int hal_enc_start(void *ctx, int chn);
extern int hal_enc_stop(void *ctx, int chn);
extern int hal_enc_poll(void *ctx, int chn, uint32_t timeout_ms);
extern int hal_enc_get_frame(void *ctx, int chn, rss_frame_t *frame);
extern int hal_enc_release_frame(void *ctx, int chn, rss_frame_t *frame);
extern int hal_enc_request_idr(void *ctx, int chn);
extern int hal_enc_set_rc_mode(void *ctx, int chn, rss_rc_mode_t mode, uint32_t bitrate);
extern int hal_enc_set_bitrate(void *ctx, int chn, uint32_t bitrate);
extern int hal_enc_set_gop(void *ctx, int chn, uint32_t gop_length);
extern int hal_enc_set_fps(void *ctx, int chn, uint32_t fps_num, uint32_t fps_den);
extern int hal_enc_set_bufshare(void *ctx, int src_chn, int dst_chn);
extern int hal_enc_get_channel_attr(void *ctx, int chn, rss_video_config_t *cfg);
extern int hal_enc_get_fps(void *ctx, int chn, uint32_t *fps_num, uint32_t *fps_den);
extern int hal_enc_get_gop_attr(void *ctx, int chn, uint32_t *gop_length);
extern int hal_enc_set_gop_attr(void *ctx, int chn, uint32_t gop_length);
extern int hal_enc_get_avg_bitrate(void *ctx, int chn, uint32_t *bitrate);
extern int hal_enc_flush_stream(void *ctx, int chn);
extern int hal_enc_query(void *ctx, int chn, bool *busy);
extern int hal_enc_get_fd(void *ctx, int chn);
extern int hal_enc_set_qp(void *ctx, int chn, int qp);
extern int hal_enc_set_qp_bounds(void *ctx, int chn, int min_qp, int max_qp);
extern int hal_enc_set_qp_ip_delta(void *ctx, int chn, int delta);
extern int hal_enc_set_stream_buf_size(void *ctx, int chn, uint32_t size);
extern int hal_enc_get_stream_buf_size(void *ctx, int chn, uint32_t *size);
extern int hal_enc_get_chn_gop_attr(void *ctx, int chn, void *gop_attr);
extern int hal_enc_set_chn_gop_attr(void *ctx, int chn, const void *gop_attr);
extern int hal_enc_get_chn_enc_type(void *ctx, int chn, void *enc_type);
extern int hal_enc_get_chn_ave_bitrate(void *ctx, int chn, void *stream, int frames, double *br);
extern int hal_enc_set_chn_entropy_mode(void *ctx, int chn, int mode);
extern int hal_enc_get_max_stream_cnt(void *ctx, int chn, int *cnt);
extern int hal_enc_set_max_stream_cnt(void *ctx, int chn, int cnt);
extern int hal_enc_set_pool(void *ctx, int chn, int pool_id);
extern int hal_enc_get_pool(void *ctx, int chn);

/* Encoder: Phase 1 — Bandwidth reduction (hal_encoder.c) */
extern int hal_enc_set_gop_mode(void *ctx, int chn, rss_gop_mode_t mode);
extern int hal_enc_get_gop_mode(void *ctx, int chn, rss_gop_mode_t *mode);
extern int hal_enc_set_rc_options(void *ctx, int chn, uint32_t options);
extern int hal_enc_get_rc_options(void *ctx, int chn, uint32_t *options);
extern int hal_enc_set_max_same_scene_cnt(void *ctx, int chn, uint32_t count);
extern int hal_enc_get_max_same_scene_cnt(void *ctx, int chn, uint32_t *count);
extern int hal_enc_set_pskip(void *ctx, int chn, const rss_pskip_cfg_t *cfg);
extern int hal_enc_get_pskip(void *ctx, int chn, rss_pskip_cfg_t *cfg);
extern int hal_enc_request_pskip(void *ctx, int chn);
extern int hal_enc_set_srd(void *ctx, int chn, const rss_srd_cfg_t *cfg);
extern int hal_enc_get_srd(void *ctx, int chn, rss_srd_cfg_t *cfg);
extern int hal_enc_set_max_pic_size(void *ctx, int chn, uint32_t max_i_kbits, uint32_t max_p_kbits);
extern int hal_enc_set_super_frame(void *ctx, int chn, const rss_super_frame_cfg_t *cfg);
extern int hal_enc_get_super_frame(void *ctx, int chn, rss_super_frame_cfg_t *cfg);
extern int hal_enc_set_color2grey(void *ctx, int chn, bool enable);
extern int hal_enc_get_color2grey(void *ctx, int chn, bool *enable);

/* Encoder: Phase 2 — Quality improvement (hal_encoder.c) */
extern int hal_enc_set_roi(void *ctx, int chn, const rss_enc_roi_t *roi);
extern int hal_enc_get_roi(void *ctx, int chn, uint32_t index, rss_enc_roi_t *roi);
extern int hal_enc_set_map_roi(void *ctx, int chn, const uint8_t *map, uint32_t map_size, int type);
extern int hal_enc_set_qp_bounds_per_frame(void *ctx, int chn, int min_i, int max_i, int min_p,
                                           int max_p);
extern int hal_enc_set_qpg_mode(void *ctx, int chn, int mode);
extern int hal_enc_get_qpg_mode(void *ctx, int chn, int *mode);
extern int hal_enc_set_qpg_ai(void *ctx, int chn, const uint8_t *map, uint32_t w, uint32_t h,
                              int mode, int mark_level);
extern int hal_enc_set_mbrc(void *ctx, int chn, bool enable);
extern int hal_enc_get_mbrc(void *ctx, int chn, bool *enable);
extern int hal_enc_set_denoise(void *ctx, int chn, const rss_enc_denoise_cfg_t *cfg);
extern int hal_enc_get_denoise(void *ctx, int chn, rss_enc_denoise_cfg_t *cfg);
extern int hal_enc_set_gdr(void *ctx, int chn, const rss_enc_gdr_cfg_t *cfg);
extern int hal_enc_get_gdr(void *ctx, int chn, rss_enc_gdr_cfg_t *cfg);
extern int hal_enc_request_gdr(void *ctx, int chn, int gdr_frames);
extern int hal_enc_insert_userdata(void *ctx, int chn, const void *data, uint32_t len);
extern int hal_enc_set_h264_vui(void *ctx, int chn, const void *vui);
extern int hal_enc_get_h264_vui(void *ctx, int chn, void *vui);
extern int hal_enc_set_h265_vui(void *ctx, int chn, const void *vui);
extern int hal_enc_get_h265_vui(void *ctx, int chn, void *vui);
extern int hal_enc_set_h264_trans(void *ctx, int chn, const rss_enc_h264_trans_t *cfg);
extern int hal_enc_get_h264_trans(void *ctx, int chn, rss_enc_h264_trans_t *cfg);
extern int hal_enc_set_h265_trans(void *ctx, int chn, const rss_enc_h265_trans_t *cfg);
extern int hal_enc_get_h265_trans(void *ctx, int chn, rss_enc_h265_trans_t *cfg);
extern int hal_enc_set_crop(void *ctx, int chn, const rss_enc_crop_cfg_t *cfg);
extern int hal_enc_get_crop(void *ctx, int chn, rss_enc_crop_cfg_t *cfg);
extern int hal_enc_get_eval_info(void *ctx, int chn, void *info);
extern int hal_enc_poll_module_stream(void *ctx, uint32_t *chn_bitmap, uint32_t timeout_ms);
extern int hal_enc_set_resize_mode(void *ctx, int chn, int enable);
extern int hal_enc_set_jpeg_ql(void *ctx, int chn, const rss_enc_jpeg_ql_t *ql);
extern int hal_enc_get_jpeg_ql(void *ctx, int chn, rss_enc_jpeg_ql_t *ql);
extern int hal_enc_set_jpeg_qp(void *ctx, int chn, int qp);
extern int hal_enc_get_jpeg_qp(void *ctx, int chn, int *qp);

/* Framesource (hal_framesource.c) */
extern int hal_fs_create_channel(void *ctx, int chn, const rss_fs_config_t *cfg);
extern int hal_fs_destroy_channel(void *ctx, int chn);
extern int hal_fs_enable_channel(void *ctx, int chn);
extern int hal_fs_disable_channel(void *ctx, int chn);
extern int hal_fs_set_rotation(void *ctx, int chn, int degrees);
extern int hal_fs_set_fifo(void *ctx, int chn, int depth);
extern int hal_fs_get_frame(void *ctx, int chn, void **frame_data, rss_frame_info_t *info);
extern int hal_fs_release_frame(void *ctx, int chn, void *frame_data);
extern int hal_fs_snap_frame(void *ctx, int chn, void **frame_data, rss_frame_info_t *info);
extern int hal_fs_set_frame_depth(void *ctx, int chn, int depth);
extern int hal_fs_get_frame_depth(void *ctx, int chn, int *depth);
extern int hal_fs_get_fifo(void *ctx, int chn, int *depth);
extern int hal_fs_set_delay(void *ctx, int chn, int delay_ms);
extern int hal_fs_get_delay(void *ctx, int chn, int *delay_ms);
extern int hal_fs_set_max_delay(void *ctx, int chn, int max_delay_ms);
extern int hal_fs_get_max_delay(void *ctx, int chn, int *max_delay_ms);
extern int hal_fs_set_pool(void *ctx, int chn, int pool_id);
extern int hal_fs_get_pool(void *ctx, int chn, int *pool_id);
extern int hal_fs_get_timed_frame(void *ctx, int chn, void *framets, int block, void *framedata,
                                  void *frame);
extern int hal_fs_set_frame_offset(void *ctx, int chn, int offset);
extern int hal_fs_chn_stat_query(void *ctx, int chn, void *stat);
extern int hal_fs_enable_chn_undistort(void *ctx, int chn);
extern int hal_fs_disable_chn_undistort(void *ctx, int chn);

/* ISP (hal_isp_gen1.c / hal_isp_gen2.c / hal_isp_gen3.c) */
extern int hal_isp_set_brightness(void *ctx, uint8_t val);
extern int hal_isp_set_contrast(void *ctx, uint8_t val);
extern int hal_isp_set_saturation(void *ctx, uint8_t val);
extern int hal_isp_set_sharpness(void *ctx, uint8_t val);
extern int hal_isp_set_hue(void *ctx, uint8_t val);
extern int hal_isp_set_hflip(void *ctx, int enable);
extern int hal_isp_set_vflip(void *ctx, int enable);
extern int hal_isp_set_running_mode(void *ctx, rss_isp_mode_t mode);
extern int hal_isp_set_sensor_fps(void *ctx, uint32_t fps_num, uint32_t fps_den);
extern int hal_isp_set_antiflicker(void *ctx, rss_antiflicker_t mode);
extern int hal_isp_set_wb(void *ctx, const rss_wb_config_t *wb_cfg);
extern int hal_isp_get_exposure(void *ctx, rss_exposure_t *exposure);
extern int hal_isp_set_sinter_strength(void *ctx, uint8_t val);
extern int hal_isp_set_temper_strength(void *ctx, uint8_t val);
extern int hal_isp_set_defog(void *ctx, int enable);
extern int hal_isp_set_dpc_strength(void *ctx, uint8_t val);
extern int hal_isp_set_drc_strength(void *ctx, uint8_t val);
extern int hal_isp_set_ae_comp(void *ctx, int val);
extern int hal_isp_set_max_again(void *ctx, uint32_t gain);
extern int hal_isp_set_max_dgain(void *ctx, uint32_t gain);
extern int hal_isp_set_highlight_depress(void *ctx, uint8_t val);

/* ISP getters and advanced tuning (hal_isp.c) */
extern int hal_isp_get_brightness(void *ctx, uint8_t *val);
extern int hal_isp_get_contrast(void *ctx, uint8_t *val);
extern int hal_isp_get_saturation(void *ctx, uint8_t *val);
extern int hal_isp_get_sharpness(void *ctx, uint8_t *val);
extern int hal_isp_get_hue(void *ctx, uint8_t *val);
extern int hal_isp_get_hvflip(void *ctx, int *hflip, int *vflip);
extern int hal_isp_get_running_mode(void *ctx, rss_isp_mode_t *mode);
extern int hal_isp_get_sensor_fps(void *ctx, uint32_t *fps_num, uint32_t *fps_den);
extern int hal_isp_get_antiflicker(void *ctx, rss_antiflicker_t *mode);
extern int hal_isp_get_wb(void *ctx, rss_wb_config_t *wb_cfg);
extern int hal_isp_get_max_again(void *ctx, uint32_t *gain);
extern int hal_isp_get_max_dgain(void *ctx, uint32_t *gain);
extern int hal_isp_get_sensor_attr(void *ctx, uint32_t *width, uint32_t *height);
extern int hal_isp_get_ae_comp(void *ctx, int *val);
extern int hal_isp_get_module_control(void *ctx, uint32_t *modules);
extern int hal_isp_get_sinter_strength(void *ctx, uint8_t *val);
extern int hal_isp_get_temper_strength(void *ctx, uint8_t *val);
extern int hal_isp_get_defog_strength(void *ctx, uint8_t *val);
extern int hal_isp_get_dpc_strength(void *ctx, uint8_t *val);
extern int hal_isp_get_drc_strength(void *ctx, uint8_t *val);
extern int hal_isp_get_highlight_depress(void *ctx, uint8_t *val);
extern int hal_isp_get_backlight_comp(void *ctx, uint8_t *val);
extern int hal_isp_set_ae_weight(void *ctx, const uint8_t weight[15][15]);
extern int hal_isp_get_ae_weight(void *ctx, uint8_t weight[15][15]);
extern int hal_isp_get_ae_zone(void *ctx, uint32_t zone[15][15]);
extern int hal_isp_set_ae_roi(void *ctx, const uint8_t roi[15][15]);
extern int hal_isp_get_ae_roi(void *ctx, uint8_t roi[15][15]);
extern int hal_isp_set_ae_hist(void *ctx, const uint8_t thresholds[4]);
extern int hal_isp_get_ae_hist(void *ctx, uint8_t thresholds[4], uint16_t bins[5]);
extern int hal_isp_get_ae_hist_origin(void *ctx, uint32_t bins[256]);
extern int hal_isp_set_ae_it_max(void *ctx, uint32_t it_max);
extern int hal_isp_get_ae_it_max(void *ctx, uint32_t *it_max);
extern int hal_isp_set_ae_min(void *ctx, int min_it, int min_again);
extern int hal_isp_get_ae_min(void *ctx, int *min_it, int *min_again);
extern int hal_isp_set_awb_weight(void *ctx, const uint8_t weight[15][15]);
extern int hal_isp_get_awb_weight(void *ctx, uint8_t weight[15][15]);
extern int hal_isp_get_awb_zone(void *ctx, uint8_t zone_r[225], uint8_t zone_g[225],
                                uint8_t zone_b[225]);
extern int hal_isp_get_awb_ct(void *ctx, uint32_t *color_temp);
extern int hal_isp_get_awb_rgb_coefft(void *ctx, uint16_t *rgain, uint16_t *ggain, uint16_t *bgain);
extern int hal_isp_get_awb_hist(void *ctx, void *hist_data);
extern int hal_isp_set_gamma(void *ctx, const uint16_t gamma[129]);
extern int hal_isp_get_gamma(void *ctx, uint16_t gamma[129]);
extern int hal_isp_set_ccm(void *ctx, const void *ccm_attr);
extern int hal_isp_get_ccm(void *ctx, void *ccm_attr);
extern int hal_isp_set_wdr_mode(void *ctx, int mode);
extern int hal_isp_get_wdr_mode(void *ctx, int *mode);
extern int hal_isp_wdr_enable(void *ctx, int enable);
extern int hal_isp_wdr_get_enable(void *ctx, int *enabled);
extern int hal_isp_set_bypass(void *ctx, int enable);
extern int hal_isp_set_module_control(void *ctx, uint32_t modules);
extern int hal_isp_set_default_bin_path(void *ctx, const char *path);
extern int hal_isp_get_default_bin_path(void *ctx, char *path, int path_len);
extern int hal_isp_set_frame_drop(void *ctx, int drop);
extern int hal_isp_get_frame_drop(void *ctx, int *drop);
extern int hal_isp_set_sensor_register(void *ctx, uint32_t reg, uint32_t val);
extern int hal_isp_get_sensor_register(void *ctx, uint32_t reg, uint32_t *val);
extern int hal_isp_set_auto_zoom(void *ctx, const void *zoom_attr);
extern int hal_isp_set_video_drop(void *ctx, void (*callback)(void));
extern int hal_isp_set_mask(void *ctx, const void *mask_attr);
extern int hal_isp_get_mask(void *ctx, void *mask_attr);

/* ISP advanced (hal_isp.c) */
extern int hal_isp_set_expr(void *ctx, const void *expr_attr);
extern int hal_isp_get_ae_attr(void *ctx, void *ae_attr);
extern int hal_isp_set_ae_attr(void *ctx, const void *ae_attr);
extern int hal_isp_get_ae_state(void *ctx, void *ae_state);
extern int hal_isp_get_ae_target_list(void *ctx, void *target_list);
extern int hal_isp_set_ae_target_list(void *ctx, const void *target_list);
extern int hal_isp_set_ae_freeze(void *ctx, int enable);
extern int hal_isp_get_af_zone(void *ctx, void *af_zone);
extern int hal_isp_get_awb_clust(void *ctx, void *clust);
extern int hal_isp_set_awb_clust(void *ctx, const void *clust);
extern int hal_isp_get_awb_ct_attr(void *ctx, void *ct_attr);
extern int hal_isp_set_awb_ct_attr(void *ctx, const void *ct_attr);
extern int hal_isp_get_awb_ct_trend(void *ctx, void *trend);
extern int hal_isp_set_awb_ct_trend(void *ctx, const void *trend);
extern int hal_isp_set_backlight_comp(void *ctx, uint32_t strength);
extern int hal_isp_get_defog_strength_adv(void *ctx, void *defog_attr);
extern int hal_isp_set_defog_strength_adv(void *ctx, const void *defog_attr);
extern int hal_isp_get_front_crop(void *ctx, void *crop_attr);
extern int hal_isp_set_front_crop(void *ctx, const void *crop_attr);
extern int hal_isp_get_blc_attr(void *ctx, void *blc_attr);
extern int hal_isp_get_csc_attr(void *ctx, void *csc_attr);
extern int hal_isp_set_csc_attr(void *ctx, const void *csc_attr);
extern int hal_isp_set_custom_mode(void *ctx, int mode);
extern int hal_isp_get_custom_mode(void *ctx, int *mode);
extern int hal_isp_enable_drc(void *ctx, int enable);
extern int hal_isp_get_af_hist(void *ctx, void *af_hist);
extern int hal_isp_set_af_hist(void *ctx, const void *af_hist);
extern int hal_isp_get_af_metrics(void *ctx, void *metrics);
extern int hal_isp_enable_movestate(void *ctx);
extern int hal_isp_disable_movestate(void *ctx);
extern int hal_isp_set_shading(void *ctx, const void *shading_attr);
extern int hal_isp_wait_frame(void *ctx, int timeout_ms);
extern int hal_isp_get_af_weight(void *ctx, void *af_weight);
extern int hal_isp_set_af_weight(void *ctx, const void *af_weight);
extern int hal_isp_get_wb_statis(void *ctx, void *wb_statis);
extern int hal_isp_set_awb_hist_adv(void *ctx, const void *awb_hist);
extern int hal_isp_get_wb_gol_statis(void *ctx, void *gol_statis);
extern int hal_isp_set_wdr_output_mode(void *ctx, int mode);
extern int hal_isp_get_wdr_output_mode(void *ctx, int *mode);
extern int hal_isp_set_scaler_lv(void *ctx, int chn, int level);

/* Multi-sensor ISP tuning (hal_isp.c) */
extern int hal_isp_set_brightness_n(void *ctx, int sensor_idx, uint8_t val);
extern int hal_isp_set_contrast_n(void *ctx, int sensor_idx, uint8_t val);
extern int hal_isp_set_saturation_n(void *ctx, int sensor_idx, uint8_t val);
extern int hal_isp_set_sharpness_n(void *ctx, int sensor_idx, uint8_t val);
extern int hal_isp_set_hue_n(void *ctx, int sensor_idx, uint8_t val);
extern int hal_isp_set_hflip_n(void *ctx, int sensor_idx, int enable);
extern int hal_isp_set_vflip_n(void *ctx, int sensor_idx, int enable);
extern int hal_isp_set_running_mode_n(void *ctx, int sensor_idx, rss_isp_mode_t mode);
extern int hal_isp_set_sensor_fps_n(void *ctx, int sensor_idx, uint32_t fps_num, uint32_t fps_den);
extern int hal_isp_set_antiflicker_n(void *ctx, int sensor_idx, rss_antiflicker_t mode);
extern int hal_isp_set_sinter_strength_n(void *ctx, int sensor_idx, uint8_t val);
extern int hal_isp_set_temper_strength_n(void *ctx, int sensor_idx, uint8_t val);
extern int hal_isp_set_ae_comp_n(void *ctx, int sensor_idx, int val);
extern int hal_isp_set_max_again_n(void *ctx, int sensor_idx, uint32_t gain);
extern int hal_isp_set_max_dgain_n(void *ctx, int sensor_idx, uint32_t gain);
extern int hal_isp_get_exposure_n(void *ctx, int sensor_idx, rss_exposure_t *exposure);
extern int hal_isp_set_custom_mode_n(void *ctx, int sensor_idx, int mode);
extern int hal_isp_set_ae_freeze_n(void *ctx, int sensor_idx, int enable);

/* Audio (hal_audio.c) */
extern int hal_audio_init(void *ctx, const rss_audio_config_t *cfg);
extern int hal_audio_deinit(void *ctx);
extern int hal_audio_set_volume(void *ctx, int dev, int chn, int vol);
extern int hal_audio_set_gain(void *ctx, int dev, int chn, int gain);
extern int hal_audio_enable_ns(void *ctx, rss_ns_level_t level);
extern int hal_audio_disable_ns(void *ctx);
extern int hal_audio_enable_hpf(void *ctx);
extern int hal_audio_disable_hpf(void *ctx);
extern int hal_audio_enable_agc(void *ctx, const rss_agc_config_t *cfg);
extern int hal_audio_disable_agc(void *ctx);
extern int hal_audio_read_frame(void *ctx, int dev, int chn, rss_audio_frame_t *frame, bool block);
extern int hal_audio_release_frame(void *ctx, int dev, int chn, rss_audio_frame_t *frame);
extern int hal_audio_register_encoder(void *ctx, const rss_audio_encoder_t *enc, int *handle);
extern int hal_audio_unregister_encoder(void *ctx, int handle);
extern int hal_audio_enable_aec(void *ctx, int ai_dev, int ai_chn, int ao_dev, int ao_chn);
extern int hal_audio_disable_aec(void *ctx);
extern int hal_audio_get_volume(void *ctx, int dev, int chn, int *vol);
extern int hal_audio_get_gain(void *ctx, int dev, int chn, int *gain);
extern int hal_audio_set_mute(void *ctx, int dev, int chn, int mute);
extern int hal_audio_set_alc_gain(void *ctx, int dev, int chn, int gain);
extern int hal_audio_get_alc_gain(void *ctx, int dev, int chn, int *gain);
extern int hal_audio_set_agc_mode(void *ctx, int mode);
extern int hal_audio_set_hpf_co_freq(void *ctx, int freq);
extern int hal_audio_enable_aec_ref_frame(void *ctx, int ai_dev, int ai_chn, int ao_dev,
                                          int ao_chn);
extern int hal_audio_disable_aec_ref_frame(void *ctx, int ai_dev, int ai_chn);
extern int hal_audio_get_chn_param(void *ctx, int dev, int chn, void *param);
extern int hal_audio_get_frame_and_ref(void *ctx, int dev, int chn, void *frame, void *ref,
                                       int block);
extern int hal_aenc_create_channel(void *ctx, int chn, int codec_type);
extern int hal_aenc_destroy_channel(void *ctx, int chn);
extern int hal_aenc_send_frame(void *ctx, int chn, rss_audio_frame_t *frame);
extern int hal_aenc_poll_stream(void *ctx, int chn, uint32_t timeout_ms);
extern int hal_aenc_get_stream(void *ctx, int chn, rss_audio_frame_t *stream);
extern int hal_aenc_release_stream(void *ctx, int chn, rss_audio_frame_t *stream);
extern int hal_adec_register_decoder_real(void *ctx, int *handle, void *decoder);
extern int hal_adec_unregister_decoder(void *ctx, int handle);
extern int hal_adec_create_channel(void *ctx, int chn, int codec_type);
extern int hal_adec_destroy_channel(void *ctx, int chn);
extern int hal_adec_send_stream(void *ctx, int chn, const uint8_t *data, uint32_t len,
                                int64_t timestamp);
extern int hal_adec_clear_buf(void *ctx, int chn);
extern int hal_adec_poll_stream(void *ctx, int chn, uint32_t timeout_ms);
extern int hal_adec_get_stream(void *ctx, int chn, rss_audio_frame_t *stream);
extern int hal_adec_release_stream(void *ctx, int chn, rss_audio_frame_t *stream);
extern int hal_ao_init(void *ctx, const rss_audio_config_t *cfg);
extern int hal_ao_deinit(void *ctx);
extern int hal_ao_set_volume(void *ctx, int vol);
extern int hal_ao_set_gain(void *ctx, int gain);
extern int hal_ao_send_frame(void *ctx, const int16_t *data, uint32_t len, bool block);
extern int hal_ao_pause(void *ctx);
extern int hal_ao_resume(void *ctx);
extern int hal_ao_clear_buf(void *ctx);
extern int hal_ao_flush_buf(void *ctx);
extern int hal_ao_get_volume(void *ctx, int *vol);
extern int hal_ao_get_gain(void *ctx, int *gain);
extern int hal_ao_set_mute(void *ctx, int mute);
extern int hal_ao_enable_hpf(void *ctx);
extern int hal_ao_disable_hpf(void *ctx);
extern int hal_ao_enable_agc(void *ctx);
extern int hal_ao_disable_agc(void *ctx);
extern int hal_ao_set_hpf_co_freq(void *ctx, int freq);
extern int hal_ao_query_chn_stat(void *ctx, int dev, int chn, void *stat);
extern int hal_ao_soft_mute(void *ctx, int dev, int chn);
extern int hal_ao_soft_unmute(void *ctx, int dev, int chn);
extern int hal_ao_cache_switch(void *ctx, int dev, int chn, int enable);

/* OSD (hal_osd.c) */
extern int hal_osd_set_pool_size(void *ctx, uint32_t bytes);
extern int hal_osd_create_group(void *ctx, int grp);
extern int hal_osd_destroy_group(void *ctx, int grp);
extern int hal_osd_create_region(void *ctx, int *handle, const rss_osd_region_t *attr);
extern int hal_osd_destroy_region(void *ctx, int handle);
extern int hal_osd_register_region(void *ctx, int handle, int grp);
extern int hal_osd_unregister_region(void *ctx, int handle, int grp);
extern int hal_osd_set_region_attr(void *ctx, int handle, const rss_osd_region_t *attr);
extern int hal_osd_update_region_data(void *ctx, int handle, const uint8_t *data);
extern int hal_osd_show_region(void *ctx, int handle, int grp, int show, int layer);
extern int hal_osd_get_region_attr(void *ctx, int handle, rss_osd_region_t *attr);
extern int hal_osd_get_group_region_attr(void *ctx, int handle, int grp, rss_osd_region_t *attr);
extern int hal_osd_show(void *ctx, int handle, int grp, bool show);
extern int hal_osd_start(void *ctx, int grp);
extern int hal_osd_stop(void *ctx, int grp);
extern int hal_osd_set_region_attr_with_timestamp(void *ctx, int handle,
                                                  const rss_osd_region_t *attr, uint64_t timestamp);
extern int hal_osd_attach_to_group(void *ctx, int handle, int grp);

/* ISP OSD (hal_osd.c) — IMP_ISP_Tuning_*OsdRgn* API */
extern int hal_isp_osd_set_pool_size(void *ctx, int size);
extern int hal_isp_osd_create_region(void *ctx, int sensornum, int *handle_out);
extern int hal_isp_osd_destroy_region(void *ctx, int sensornum, int handle);
extern int hal_isp_osd_set_region_attr(void *ctx, int sensornum, int handle, int chx,
                                       const rss_osd_region_t *attr);
extern int hal_isp_osd_show_region(void *ctx, int sensornum, int handle, int show);
extern int hal_isp_osd_set_mask(void *ctx, int sensornum, int chx, int pinum, int enable, int x,
                                int y, int w, int h, uint32_t color);

/* GPIO (hal_gpio.c) */
extern int hal_gpio_set(void *ctx, int pin, int value);
extern int hal_gpio_get(void *ctx, int pin, int *value);
extern int hal_ircut_set(void *ctx, int state);

/* IVS (hal_ivs.c) */
extern int hal_ivs_create_group(void *ctx, int grp);
extern int hal_ivs_destroy_group(void *ctx, int grp);
extern int hal_ivs_create_channel(void *ctx, int chn, void *algo_handle);
extern int hal_ivs_destroy_channel(void *ctx, int chn);
extern int hal_ivs_register_channel(void *ctx, int grp, int chn);
extern int hal_ivs_unregister_channel(void *ctx, int chn);
extern int hal_ivs_start(void *ctx, int chn);
extern int hal_ivs_stop(void *ctx, int chn);
extern int hal_ivs_poll_result(void *ctx, int chn, uint32_t timeout_ms);
extern int hal_ivs_get_result(void *ctx, int chn, void **result);
extern int hal_ivs_release_result(void *ctx, int chn, void *result);
extern int hal_ivs_get_param(void *ctx, int chn, void *param);
extern int hal_ivs_set_param(void *ctx, int chn, void *param);
extern int hal_ivs_release_data(void *ctx, int chn, void *data);
extern void *hal_ivs_create_move_interface(void *ctx, void *param);
extern int hal_ivs_destroy_move_interface(void *ctx, void *handle);
extern void *hal_ivs_create_base_move_interface(void *ctx, void *param);
extern int hal_ivs_destroy_base_move_interface(void *ctx, void *handle);
extern void *hal_ivs_create_persondet_interface(void *ctx, void *param);
extern int hal_ivs_destroy_persondet_interface(void *ctx, void *handle);

/* DMIC (hal_dmic.c) */
extern int hal_dmic_init(void *ctx, const rss_audio_config_t *cfg);
extern int hal_dmic_deinit(void *ctx);
extern int hal_dmic_set_volume(void *ctx, int vol);
extern int hal_dmic_get_volume(void *ctx, int *vol);
extern int hal_dmic_set_gain(void *ctx, int gain);
extern int hal_dmic_get_gain(void *ctx, int *gain);
extern int hal_dmic_set_chn_param(void *ctx, int chn, int frames_per_buf);
extern int hal_dmic_get_chn_param(void *ctx, int chn, int *frames_per_buf);
extern int hal_dmic_read_frame(void *ctx, rss_audio_frame_t *frame, bool block);
extern int hal_dmic_release_frame(void *ctx, rss_audio_frame_t *frame);
extern int hal_dmic_poll_frame(void *ctx, uint32_t timeout_ms);
extern int hal_dmic_enable_aec(void *ctx, int dev, int chn, int ao_dev, int ao_chn);
extern int hal_dmic_disable_aec(void *ctx, int dev, int chn);
extern int hal_dmic_enable_aec_ref_frame(void *ctx, int dev, int chn, int ao_dev, int ao_chn);
extern int hal_dmic_disable_aec_ref_frame(void *ctx, int dev, int chn, int ao_dev, int ao_chn);
extern int hal_dmic_get_pub_attr(void *ctx, int dev, void *attr);
extern int hal_dmic_get_frame_and_ref(void *ctx, int dev, int chn, void *frame, void *ref,
                                      int block);

/* Memory (hal_memory.c) */
extern void *hal_mem_alloc(void *ctx, uint32_t size, const char *name);
extern void hal_mem_free(void *ctx, void *ptr);
extern int hal_mem_flush_cache(void *ctx, void *ptr, uint32_t size);
extern void *hal_mem_phys_to_virt(void *ctx, uint32_t phys_addr);
extern uint32_t hal_mem_virt_to_phys(void *ctx, void *virt_addr);
extern void *hal_mem_pool_phys_to_virt(void *ctx, uint32_t phys_addr);
extern uint32_t hal_mem_pool_virt_to_phys(void *ctx, void *virt_addr);
extern void *hal_mem_pool_alloc(void *ctx, uint32_t pool_id, uint32_t size);
extern void hal_mem_pool_free(void *ctx, void *ptr);
extern int hal_mem_pool_flush_cache(void *ctx, void *ptr, uint32_t size);

/* ── Forward declarations for functions in this file ─────────────── */

static int hal_init(void *ctx, const rss_multi_sensor_config_t *multi_cfg);
static int hal_deinit(void *ctx);
static const rss_hal_caps_t *hal_get_caps(void *ctx);
static int hal_bind(void *ctx, const rss_cell_t *src, const rss_cell_t *dst);
static int hal_unbind(void *ctx, const rss_cell_t *src, const rss_cell_t *dst);

/* ── Forward declaration needed by system utilities ────────────── */

static IMPDeviceID hal_translate_dev_id(rss_dev_id_t dev);

/* ── System utility implementations ─────────────────────────────── */

static int hal_sys_get_version(void *ctx, char *buf, int len)
{
    (void)ctx;
    if (!buf || len <= 0)
        return -EINVAL;

    IMPVersion version;
    memset(&version, 0, sizeof(version));
    int ret = IMP_System_GetVersion(&version);
    if (ret != 0)
        return ret;

    snprintf(buf, (size_t)len, "%s", version.aVersion);
    return 0;
}

static int hal_sys_get_cpu_info(void *ctx, char *buf, int len)
{
    (void)ctx;
    if (!buf || len <= 0)
        return -EINVAL;

    const char *info = IMP_System_GetCPUInfo();
    if (!info)
        return -EIO;

    snprintf(buf, (size_t)len, "%s", info);
    return 0;
}

static int hal_sys_get_timestamp(void *ctx, int64_t *ts)
{
    (void)ctx;
    if (!ts)
        return -EINVAL;
    *ts = IMP_System_GetTimeStamp();
    return 0;
}

static int hal_sys_rebase_timestamp(void *ctx, int64_t base)
{
    (void)ctx;
    return IMP_System_RebaseTimeStamp(base);
}

static int hal_sys_read_reg32(void *ctx, uint32_t addr, uint32_t *val)
{
    (void)ctx;
    if (!val)
        return -EINVAL;
    *val = IMP_System_ReadReg32(addr);
    return 0;
}

static int hal_sys_write_reg32(void *ctx, uint32_t addr, uint32_t val)
{
    (void)ctx;
    IMP_System_WriteReg32(addr, val);
    return 0;
}

static int hal_sys_get_bind_by_dest(void *ctx, rss_cell_t *dst, rss_cell_t *src)
{
    (void)ctx;
    if (!dst || !src)
        return -EINVAL;

    IMPCell imp_dst, imp_src;
    imp_dst.deviceID = hal_translate_dev_id(dst->device);
    imp_dst.groupID = dst->group;
    imp_dst.outputID = dst->output;

    int ret = IMP_System_GetBindbyDest(&imp_dst, &imp_src);
    if (ret != 0)
        return ret;

    /* Reverse translate IMPCell back to rss_cell_t */
    switch (imp_src.deviceID) {
    case DEV_ID_FS:
        src->device = RSS_DEV_FS;
        break;
    case DEV_ID_ENC:
        src->device = RSS_DEV_ENC;
        break;
    case DEV_ID_DEC:
        src->device = RSS_DEV_DEC;
        break;
    case DEV_ID_IVS:
        src->device = RSS_DEV_IVS;
        break;
    case DEV_ID_OSD:
        src->device = RSS_DEV_OSD;
        break;
    default:
        src->device = RSS_DEV_FS;
        break;
    }
    src->group = imp_src.groupID;
    src->output = imp_src.outputID;
    return 0;
}

/* ── Per-SoC capability data (defined in hal_caps.c) ─────────────── */

extern const rss_hal_caps_t g_hal_caps;

/* ================================================================
 * OPS VTABLE
 * ================================================================ */

static const rss_hal_ops_t g_ops = {
    /* System lifecycle */
    .init = hal_init,
    .deinit = hal_deinit,
    .get_caps = hal_get_caps,
    .bind = hal_bind,
    .unbind = hal_unbind,

    /* System utilities */
    .sys_get_version = hal_sys_get_version,
    .sys_get_cpu_info = hal_sys_get_cpu_info,
    .sys_get_timestamp = hal_sys_get_timestamp,
    .sys_rebase_timestamp = hal_sys_rebase_timestamp,
    .sys_read_reg32 = hal_sys_read_reg32,
    .sys_write_reg32 = hal_sys_write_reg32,
    .sys_get_bind_by_dest = hal_sys_get_bind_by_dest,

    /* Framesource */
    .fs_create_channel = hal_fs_create_channel,
    .fs_destroy_channel = hal_fs_destroy_channel,
    .fs_enable_channel = hal_fs_enable_channel,
    .fs_disable_channel = hal_fs_disable_channel,
    .fs_set_rotation = hal_fs_set_rotation,
    .fs_set_fifo = hal_fs_set_fifo,
    .fs_get_frame = hal_fs_get_frame,
    .fs_release_frame = hal_fs_release_frame,
    .fs_snap_frame = hal_fs_snap_frame,
    .fs_set_frame_depth = hal_fs_set_frame_depth,
    .fs_get_frame_depth = hal_fs_get_frame_depth,
    .fs_get_fifo = hal_fs_get_fifo,
    .fs_set_delay = hal_fs_set_delay,
    .fs_get_delay = hal_fs_get_delay,
    .fs_set_max_delay = hal_fs_set_max_delay,
    .fs_get_max_delay = hal_fs_get_max_delay,
    .fs_set_pool = hal_fs_set_pool,
    .fs_get_pool = hal_fs_get_pool,
    .fs_get_timed_frame = hal_fs_get_timed_frame,
    .fs_set_frame_offset = hal_fs_set_frame_offset,
    .fs_chn_stat_query = hal_fs_chn_stat_query,
    .fs_enable_chn_undistort = hal_fs_enable_chn_undistort,
    .fs_disable_chn_undistort = hal_fs_disable_chn_undistort,

    /* Encoder */
    .enc_create_group = hal_enc_create_group,
    .enc_destroy_group = hal_enc_destroy_group,
    .enc_create_channel = hal_enc_create_channel,
    .enc_destroy_channel = hal_enc_destroy_channel,
    .enc_register_channel = hal_enc_register_channel,
    .enc_unregister_channel = hal_enc_unregister_channel,
    .enc_start = hal_enc_start,
    .enc_stop = hal_enc_stop,
    .enc_poll = hal_enc_poll,
    .enc_get_frame = hal_enc_get_frame,
    .enc_release_frame = hal_enc_release_frame,
    .enc_request_idr = hal_enc_request_idr,
    .enc_set_rc_mode = hal_enc_set_rc_mode,
    .enc_set_bitrate = hal_enc_set_bitrate,
    .enc_set_gop = hal_enc_set_gop,
    .enc_set_fps = hal_enc_set_fps,
    .enc_set_bufshare = hal_enc_set_bufshare,
    .enc_get_channel_attr = hal_enc_get_channel_attr,
    .enc_get_fps = hal_enc_get_fps,
    .enc_get_gop_attr = hal_enc_get_gop_attr,
    .enc_set_gop_attr = hal_enc_set_gop_attr,
    .enc_get_avg_bitrate = hal_enc_get_avg_bitrate,
    .enc_flush_stream = hal_enc_flush_stream,
    .enc_query = hal_enc_query,
    .enc_get_fd = hal_enc_get_fd,
    .enc_set_qp = hal_enc_set_qp,
    .enc_set_qp_bounds = hal_enc_set_qp_bounds,
    .enc_set_qp_ip_delta = hal_enc_set_qp_ip_delta,
    .enc_set_stream_buf_size = hal_enc_set_stream_buf_size,
    .enc_get_stream_buf_size = hal_enc_get_stream_buf_size,
    .enc_get_chn_gop_attr = hal_enc_get_chn_gop_attr,
    .enc_set_chn_gop_attr = hal_enc_set_chn_gop_attr,
    .enc_get_chn_enc_type = hal_enc_get_chn_enc_type,
    .enc_get_chn_ave_bitrate = hal_enc_get_chn_ave_bitrate,
    .enc_set_chn_entropy_mode = hal_enc_set_chn_entropy_mode,
    .enc_get_max_stream_cnt = hal_enc_get_max_stream_cnt,
    .enc_set_max_stream_cnt = hal_enc_set_max_stream_cnt,
    .enc_set_pool = hal_enc_set_pool,
    .enc_get_pool = hal_enc_get_pool,

    /* Encoder: Phase 1 — Bandwidth reduction */
    .enc_set_gop_mode = hal_enc_set_gop_mode,
    .enc_get_gop_mode = hal_enc_get_gop_mode,
    .enc_set_rc_options = hal_enc_set_rc_options,
    .enc_get_rc_options = hal_enc_get_rc_options,
    .enc_set_max_same_scene_cnt = hal_enc_set_max_same_scene_cnt,
    .enc_get_max_same_scene_cnt = hal_enc_get_max_same_scene_cnt,
    .enc_set_pskip = hal_enc_set_pskip,
    .enc_get_pskip = hal_enc_get_pskip,
    .enc_request_pskip = hal_enc_request_pskip,
    .enc_set_srd = hal_enc_set_srd,
    .enc_get_srd = hal_enc_get_srd,
    .enc_set_max_pic_size = hal_enc_set_max_pic_size,
    .enc_set_super_frame = hal_enc_set_super_frame,
    .enc_get_super_frame = hal_enc_get_super_frame,
    .enc_set_color2grey = hal_enc_set_color2grey,
    .enc_get_color2grey = hal_enc_get_color2grey,

    /* Encoder: Phase 2 — Quality improvement */
    .enc_set_roi = hal_enc_set_roi,
    .enc_get_roi = hal_enc_get_roi,
    .enc_set_map_roi = hal_enc_set_map_roi,
    .enc_set_qp_bounds_per_frame = hal_enc_set_qp_bounds_per_frame,
    .enc_set_qpg_mode = hal_enc_set_qpg_mode,
    .enc_get_qpg_mode = hal_enc_get_qpg_mode,
    .enc_set_qpg_ai = hal_enc_set_qpg_ai,
    .enc_set_mbrc = hal_enc_set_mbrc,
    .enc_get_mbrc = hal_enc_get_mbrc,
    .enc_set_denoise = hal_enc_set_denoise,
    .enc_get_denoise = hal_enc_get_denoise,

    /* Encoder: Phase 3 — Error recovery */
    .enc_set_gdr = hal_enc_set_gdr,
    .enc_get_gdr = hal_enc_get_gdr,
    .enc_request_gdr = hal_enc_request_gdr,
    .enc_insert_userdata = hal_enc_insert_userdata,

    /* Encoder: Phase 4 — Codec compliance */
    .enc_set_h264_vui = hal_enc_set_h264_vui,
    .enc_get_h264_vui = hal_enc_get_h264_vui,
    .enc_set_h265_vui = hal_enc_set_h265_vui,
    .enc_get_h265_vui = hal_enc_get_h265_vui,
    .enc_set_h264_trans = hal_enc_set_h264_trans,
    .enc_get_h264_trans = hal_enc_get_h264_trans,
    .enc_set_h265_trans = hal_enc_set_h265_trans,
    .enc_get_h265_trans = hal_enc_get_h265_trans,

    /* Encoder: Phase 5 — Operational */
    .enc_set_crop = hal_enc_set_crop,
    .enc_get_crop = hal_enc_get_crop,
    .enc_get_eval_info = hal_enc_get_eval_info,
    .enc_poll_module_stream = hal_enc_poll_module_stream,
    .enc_set_resize_mode = hal_enc_set_resize_mode,

    /* Encoder: Phase 6 — JPEG */
    .enc_set_jpeg_ql = hal_enc_set_jpeg_ql,
    .enc_get_jpeg_ql = hal_enc_get_jpeg_ql,
    .enc_set_jpeg_qp = hal_enc_set_jpeg_qp,
    .enc_get_jpeg_qp = hal_enc_get_jpeg_qp,

    /* ISP tuning */
    .isp_set_brightness = hal_isp_set_brightness,
    .isp_set_contrast = hal_isp_set_contrast,
    .isp_set_saturation = hal_isp_set_saturation,
    .isp_set_sharpness = hal_isp_set_sharpness,
    .isp_set_hue = hal_isp_set_hue,
    .isp_set_hflip = hal_isp_set_hflip,
    .isp_set_vflip = hal_isp_set_vflip,
    .isp_set_running_mode = hal_isp_set_running_mode,
    .isp_set_sensor_fps = hal_isp_set_sensor_fps,
    .isp_set_antiflicker = hal_isp_set_antiflicker,
    .isp_set_wb = hal_isp_set_wb,
    .isp_get_exposure = hal_isp_get_exposure,
    .isp_set_sinter_strength = hal_isp_set_sinter_strength,
    .isp_set_temper_strength = hal_isp_set_temper_strength,
    .isp_set_defog = hal_isp_set_defog,
    .isp_set_dpc_strength = hal_isp_set_dpc_strength,
    .isp_set_drc_strength = hal_isp_set_drc_strength,
    .isp_set_ae_comp = hal_isp_set_ae_comp,
    .isp_set_max_again = hal_isp_set_max_again,
    .isp_set_max_dgain = hal_isp_set_max_dgain,
    .isp_set_highlight_depress = hal_isp_set_highlight_depress,

    /* ISP getters */
    .isp_get_brightness = hal_isp_get_brightness,
    .isp_get_contrast = hal_isp_get_contrast,
    .isp_get_saturation = hal_isp_get_saturation,
    .isp_get_sharpness = hal_isp_get_sharpness,
    .isp_get_hue = hal_isp_get_hue,
    .isp_get_hvflip = hal_isp_get_hvflip,
    .isp_get_running_mode = hal_isp_get_running_mode,
    .isp_get_sensor_fps = hal_isp_get_sensor_fps,
    .isp_get_antiflicker = hal_isp_get_antiflicker,
    .isp_get_wb = hal_isp_get_wb,
    .isp_get_max_again = hal_isp_get_max_again,
    .isp_get_max_dgain = hal_isp_get_max_dgain,
    .isp_get_sensor_attr = hal_isp_get_sensor_attr,
    .isp_get_ae_comp = hal_isp_get_ae_comp,
    .isp_get_module_control = hal_isp_get_module_control,
    .isp_get_sinter_strength = hal_isp_get_sinter_strength,
    .isp_get_temper_strength = hal_isp_get_temper_strength,
    .isp_get_defog_strength = hal_isp_get_defog_strength,
    .isp_get_dpc_strength = hal_isp_get_dpc_strength,
    .isp_get_drc_strength = hal_isp_get_drc_strength,
    .isp_get_highlight_depress = hal_isp_get_highlight_depress,
    .isp_get_backlight_comp = hal_isp_get_backlight_comp,

    /* ISP AE advanced */
    .isp_set_ae_weight = hal_isp_set_ae_weight,
    .isp_get_ae_weight = hal_isp_get_ae_weight,
    .isp_get_ae_zone = hal_isp_get_ae_zone,
    .isp_set_ae_roi = hal_isp_set_ae_roi,
    .isp_get_ae_roi = hal_isp_get_ae_roi,
    .isp_set_ae_hist = hal_isp_set_ae_hist,
    .isp_get_ae_hist = hal_isp_get_ae_hist,
    .isp_get_ae_hist_origin = hal_isp_get_ae_hist_origin,
    .isp_set_ae_it_max = hal_isp_set_ae_it_max,
    .isp_get_ae_it_max = hal_isp_get_ae_it_max,
    .isp_set_ae_min = hal_isp_set_ae_min,
    .isp_get_ae_min = hal_isp_get_ae_min,

    /* ISP AWB advanced */
    .isp_set_awb_weight = hal_isp_set_awb_weight,
    .isp_get_awb_weight = hal_isp_get_awb_weight,
    .isp_get_awb_zone = hal_isp_get_awb_zone,
    .isp_get_awb_ct = hal_isp_get_awb_ct,
    .isp_get_awb_rgb_coefft = hal_isp_get_awb_rgb_coefft,
    .isp_get_awb_hist = hal_isp_get_awb_hist,

    /* ISP gamma / CCM / WDR */
    .isp_set_gamma = hal_isp_set_gamma,
    .isp_get_gamma = hal_isp_get_gamma,
    .isp_set_ccm = hal_isp_set_ccm,
    .isp_get_ccm = hal_isp_get_ccm,
    .isp_set_wdr_mode = hal_isp_set_wdr_mode,
    .isp_get_wdr_mode = hal_isp_get_wdr_mode,
    .isp_wdr_enable = hal_isp_wdr_enable,
    .isp_wdr_get_enable = hal_isp_wdr_get_enable,
    .isp_set_bypass = hal_isp_set_bypass,
    .isp_set_module_control = hal_isp_set_module_control,

    /* ISP misc */
    .isp_set_default_bin_path = hal_isp_set_default_bin_path,
    .isp_get_default_bin_path = hal_isp_get_default_bin_path,
    .isp_set_frame_drop = hal_isp_set_frame_drop,
    .isp_get_frame_drop = hal_isp_get_frame_drop,
    .isp_set_sensor_register = hal_isp_set_sensor_register,
    .isp_get_sensor_register = hal_isp_get_sensor_register,
    .isp_set_auto_zoom = hal_isp_set_auto_zoom,
    .isp_set_video_drop = hal_isp_set_video_drop,
    .isp_set_mask = hal_isp_set_mask,
    .isp_get_mask = hal_isp_get_mask,

    /* ISP advanced */
    .isp_set_expr = hal_isp_set_expr,
    .isp_get_ae_attr = hal_isp_get_ae_attr,
    .isp_set_ae_attr = hal_isp_set_ae_attr,
    .isp_get_ae_state = hal_isp_get_ae_state,
    .isp_get_ae_target_list = hal_isp_get_ae_target_list,
    .isp_set_ae_target_list = hal_isp_set_ae_target_list,
    .isp_set_ae_freeze = hal_isp_set_ae_freeze,
    .isp_get_af_zone = hal_isp_get_af_zone,
    .isp_get_awb_clust = hal_isp_get_awb_clust,
    .isp_set_awb_clust = hal_isp_set_awb_clust,
    .isp_get_awb_ct_attr = hal_isp_get_awb_ct_attr,
    .isp_set_awb_ct_attr = hal_isp_set_awb_ct_attr,
    .isp_get_awb_ct_trend = hal_isp_get_awb_ct_trend,
    .isp_set_awb_ct_trend = hal_isp_set_awb_ct_trend,
    .isp_set_backlight_comp = hal_isp_set_backlight_comp,
    .isp_get_defog_strength_adv = hal_isp_get_defog_strength_adv,
    .isp_set_defog_strength_adv = hal_isp_set_defog_strength_adv,
    .isp_get_front_crop = hal_isp_get_front_crop,
    .isp_set_front_crop = hal_isp_set_front_crop,
    .isp_get_blc_attr = hal_isp_get_blc_attr,
    .isp_get_csc_attr = hal_isp_get_csc_attr,
    .isp_set_csc_attr = hal_isp_set_csc_attr,
    .isp_set_custom_mode = hal_isp_set_custom_mode,
    .isp_get_custom_mode = hal_isp_get_custom_mode,
    .isp_enable_drc = hal_isp_enable_drc,
    .isp_get_af_hist = hal_isp_get_af_hist,
    .isp_set_af_hist = hal_isp_set_af_hist,
    .isp_get_af_metrics = hal_isp_get_af_metrics,
    .isp_enable_movestate = hal_isp_enable_movestate,
    .isp_disable_movestate = hal_isp_disable_movestate,
    .isp_set_shading = hal_isp_set_shading,
    .isp_wait_frame = hal_isp_wait_frame,
    .isp_get_af_weight = hal_isp_get_af_weight,
    .isp_set_af_weight = hal_isp_set_af_weight,
    .isp_get_wb_statis = hal_isp_get_wb_statis,
    .isp_set_awb_hist_adv = hal_isp_set_awb_hist_adv,
    .isp_get_wb_gol_statis = hal_isp_get_wb_gol_statis,
    .isp_set_wdr_output_mode = hal_isp_set_wdr_output_mode,
    .isp_get_wdr_output_mode = hal_isp_get_wdr_output_mode,
    .isp_set_scaler_lv = hal_isp_set_scaler_lv,

    /* Multi-sensor ISP tuning */
    .isp_set_brightness_n = hal_isp_set_brightness_n,
    .isp_set_contrast_n = hal_isp_set_contrast_n,
    .isp_set_saturation_n = hal_isp_set_saturation_n,
    .isp_set_sharpness_n = hal_isp_set_sharpness_n,
    .isp_set_hue_n = hal_isp_set_hue_n,
    .isp_set_hflip_n = hal_isp_set_hflip_n,
    .isp_set_vflip_n = hal_isp_set_vflip_n,
    .isp_set_running_mode_n = hal_isp_set_running_mode_n,
    .isp_set_sensor_fps_n = hal_isp_set_sensor_fps_n,
    .isp_set_antiflicker_n = hal_isp_set_antiflicker_n,
    .isp_set_sinter_strength_n = hal_isp_set_sinter_strength_n,
    .isp_set_temper_strength_n = hal_isp_set_temper_strength_n,
    .isp_set_ae_comp_n = hal_isp_set_ae_comp_n,
    .isp_set_max_again_n = hal_isp_set_max_again_n,
    .isp_set_max_dgain_n = hal_isp_set_max_dgain_n,
    .isp_get_exposure_n = hal_isp_get_exposure_n,
    .isp_set_custom_mode_n = hal_isp_set_custom_mode_n,
    .isp_set_ae_freeze_n = hal_isp_set_ae_freeze_n,

    /* Audio */
    .audio_init = hal_audio_init,
    .audio_deinit = hal_audio_deinit,
    .audio_set_volume = hal_audio_set_volume,
    .audio_set_gain = hal_audio_set_gain,
    .audio_enable_ns = hal_audio_enable_ns,
    .audio_disable_ns = hal_audio_disable_ns,
    .audio_enable_hpf = hal_audio_enable_hpf,
    .audio_disable_hpf = hal_audio_disable_hpf,
    .audio_enable_agc = hal_audio_enable_agc,
    .audio_disable_agc = hal_audio_disable_agc,
    .audio_read_frame = hal_audio_read_frame,
    .audio_release_frame = hal_audio_release_frame,
    .audio_register_encoder = hal_audio_register_encoder,
    .audio_unregister_encoder = hal_audio_unregister_encoder,
    .audio_enable_aec = hal_audio_enable_aec,
    .audio_disable_aec = hal_audio_disable_aec,
    .audio_get_volume = hal_audio_get_volume,
    .audio_get_gain = hal_audio_get_gain,
    .audio_set_mute = hal_audio_set_mute,
    .audio_set_alc_gain = hal_audio_set_alc_gain,
    .audio_get_alc_gain = hal_audio_get_alc_gain,
    .audio_set_agc_mode = hal_audio_set_agc_mode,
    .audio_set_hpf_co_freq = hal_audio_set_hpf_co_freq,
    .audio_enable_aec_ref_frame = hal_audio_enable_aec_ref_frame,
    .audio_disable_aec_ref_frame = hal_audio_disable_aec_ref_frame,
    .audio_get_chn_param = hal_audio_get_chn_param,
    .audio_get_frame_and_ref = hal_audio_get_frame_and_ref,
    .aenc_create_channel = hal_aenc_create_channel,
    .aenc_destroy_channel = hal_aenc_destroy_channel,
    .aenc_send_frame = hal_aenc_send_frame,
    .aenc_poll_stream = hal_aenc_poll_stream,
    .aenc_get_stream = hal_aenc_get_stream,
    .aenc_release_stream = hal_aenc_release_stream,
    .adec_register_decoder = hal_adec_register_decoder_real,
    .adec_unregister_decoder = hal_adec_unregister_decoder,
    .adec_create_channel = hal_adec_create_channel,
    .adec_destroy_channel = hal_adec_destroy_channel,
    .adec_send_stream = hal_adec_send_stream,
    .adec_clear_buf = hal_adec_clear_buf,
    .adec_poll_stream = hal_adec_poll_stream,
    .adec_get_stream = hal_adec_get_stream,
    .adec_release_stream = hal_adec_release_stream,
    .ao_init = hal_ao_init,
    .ao_deinit = hal_ao_deinit,
    .ao_set_volume = hal_ao_set_volume,
    .ao_set_gain = hal_ao_set_gain,
    .ao_send_frame = hal_ao_send_frame,
    .ao_pause = hal_ao_pause,
    .ao_resume = hal_ao_resume,
    .ao_clear_buf = hal_ao_clear_buf,
    .ao_flush_buf = hal_ao_flush_buf,
    .ao_get_volume = hal_ao_get_volume,
    .ao_get_gain = hal_ao_get_gain,
    .ao_set_mute = hal_ao_set_mute,
    .ao_enable_hpf = hal_ao_enable_hpf,
    .ao_disable_hpf = hal_ao_disable_hpf,
    .ao_enable_agc = hal_ao_enable_agc,
    .ao_disable_agc = hal_ao_disable_agc,
    .ao_set_hpf_co_freq = hal_ao_set_hpf_co_freq,
    .ao_query_chn_stat = hal_ao_query_chn_stat,
    .ao_soft_mute = hal_ao_soft_mute,
    .ao_soft_unmute = hal_ao_soft_unmute,
    .ao_cache_switch = hal_ao_cache_switch,

    /* OSD */
    .osd_set_pool_size = hal_osd_set_pool_size,
    .osd_create_group = hal_osd_create_group,
    .osd_destroy_group = hal_osd_destroy_group,
    .osd_create_region = hal_osd_create_region,
    .osd_destroy_region = hal_osd_destroy_region,
    .osd_register_region = hal_osd_register_region,
    .osd_unregister_region = hal_osd_unregister_region,
    .osd_set_region_attr = hal_osd_set_region_attr,
    .osd_update_region_data = hal_osd_update_region_data,
    .osd_show_region = hal_osd_show_region,
    .osd_get_region_attr = hal_osd_get_region_attr,
    .osd_get_group_region_attr = hal_osd_get_group_region_attr,
    .osd_show = hal_osd_show,
    .osd_start = hal_osd_start,
    .osd_stop = hal_osd_stop,
    .osd_set_region_attr_with_timestamp = hal_osd_set_region_attr_with_timestamp,
    .osd_attach_to_group = hal_osd_attach_to_group,
    /* ISP OSD */
    .isp_osd_set_pool_size = hal_isp_osd_set_pool_size,
    .isp_osd_create_region = hal_isp_osd_create_region,
    .isp_osd_destroy_region = hal_isp_osd_destroy_region,
    .isp_osd_set_region_attr = hal_isp_osd_set_region_attr,
    .isp_osd_show_region = hal_isp_osd_show_region,
    .isp_osd_set_mask = hal_isp_osd_set_mask,

    /* GPIO / IR-cut */
    .gpio_set = hal_gpio_set,
    .gpio_get = hal_gpio_get,
    .ircut_set = hal_ircut_set,

    /* IVS */
    .ivs_create_group = hal_ivs_create_group,
    .ivs_destroy_group = hal_ivs_destroy_group,
    .ivs_create_channel = hal_ivs_create_channel,
    .ivs_destroy_channel = hal_ivs_destroy_channel,
    .ivs_register_channel = hal_ivs_register_channel,
    .ivs_unregister_channel = hal_ivs_unregister_channel,
    .ivs_start = hal_ivs_start,
    .ivs_stop = hal_ivs_stop,
    .ivs_poll_result = hal_ivs_poll_result,
    .ivs_get_result = hal_ivs_get_result,
    .ivs_release_result = hal_ivs_release_result,
    .ivs_get_param = hal_ivs_get_param,
    .ivs_set_param = hal_ivs_set_param,
    .ivs_release_data = hal_ivs_release_data,
    .ivs_create_move_interface = hal_ivs_create_move_interface,
    .ivs_destroy_move_interface = hal_ivs_destroy_move_interface,
    .ivs_create_base_move_interface = hal_ivs_create_base_move_interface,
    .ivs_destroy_base_move_interface = hal_ivs_destroy_base_move_interface,
    .ivs_create_persondet_interface = hal_ivs_create_persondet_interface,
    .ivs_destroy_persondet_interface = hal_ivs_destroy_persondet_interface,

    /* DMIC */
    .dmic_init = hal_dmic_init,
    .dmic_deinit = hal_dmic_deinit,
    .dmic_set_volume = hal_dmic_set_volume,
    .dmic_get_volume = hal_dmic_get_volume,
    .dmic_set_gain = hal_dmic_set_gain,
    .dmic_get_gain = hal_dmic_get_gain,
    .dmic_set_chn_param = hal_dmic_set_chn_param,
    .dmic_get_chn_param = hal_dmic_get_chn_param,
    .dmic_read_frame = hal_dmic_read_frame,
    .dmic_release_frame = hal_dmic_release_frame,
    .dmic_poll_frame = hal_dmic_poll_frame,
    .dmic_enable_aec = hal_dmic_enable_aec,
    .dmic_disable_aec = hal_dmic_disable_aec,
    .dmic_enable_aec_ref_frame = hal_dmic_enable_aec_ref_frame,
    .dmic_disable_aec_ref_frame = hal_dmic_disable_aec_ref_frame,
    .dmic_get_pub_attr = hal_dmic_get_pub_attr,
    .dmic_get_frame_and_ref = hal_dmic_get_frame_and_ref,

    /* Memory */
    .mem_alloc = hal_mem_alloc,
    .mem_free = hal_mem_free,
    .mem_flush_cache = hal_mem_flush_cache,
    .mem_phys_to_virt = hal_mem_phys_to_virt,
    .mem_virt_to_phys = hal_mem_virt_to_phys,
    .mem_pool_alloc = hal_mem_pool_alloc,
    .mem_pool_free = hal_mem_pool_free,
    .mem_pool_flush_cache = hal_mem_pool_flush_cache,
    .mem_pool_phys_to_virt = hal_mem_pool_phys_to_virt,
    .mem_pool_virt_to_phys = hal_mem_pool_virt_to_phys,
};

/* ================================================================
 * SENSOR INFO CONSTRUCTION
 * ================================================================ */

/*
 * Populate an IMPSensorInfo from our portable rss_sensor_config_t.
 *
 * The IMPSensorInfo struct layout varies across SoC generations:
 *   T20-T31: { name, cbus_type, i2c/spi union, rst_gpio (u16), pwdn_gpio (u16), power_gpio (u16) }
 *   T32/T40/T41: adds video_interface (IMPSensorVinType), mclk (IMPSensorMclk),
 *                default_boot, sensor_id; gpio fields are int instead of u16
 */
static void hal_fill_sensor_info(IMPSensorInfo *info, const rss_sensor_config_t *cfg)
{
    memset(info, 0, sizeof(*info));
    snprintf(info->name, sizeof(info->name), "%s", cfg->name);
    info->cbus_type = TX_SENSOR_CONTROL_INTERFACE_I2C;
    snprintf(info->i2c.type, sizeof(info->i2c.type), "%s", cfg->name);
    info->i2c.addr = cfg->i2c_addr;
    info->i2c.i2c_adapter_id = cfg->i2c_adapter;

#if defined(HAL_MULTI_SENSOR)
    /* T32/T40/T41: extended sensor info fields */
    info->rst_gpio = cfg->rst_gpio;
    info->pwdn_gpio = cfg->pwdn_gpio;
    info->power_gpio = cfg->power_gpio;
    info->sensor_id = cfg->sensor_id;
    info->video_interface = (IMPSensorVinType)cfg->vin_type;
    info->mclk = (IMPSensorMclk)cfg->mclk;
    info->default_boot = cfg->default_boot;
#elif defined(PLATFORM_T23)
    /* T23: has sensor_id but gpio fields are still unsigned short */
    info->rst_gpio = (unsigned short)cfg->rst_gpio;
    info->pwdn_gpio = (unsigned short)cfg->pwdn_gpio;
    info->sensor_id = cfg->sensor_id;
#else
    /* T20/T21/T30/T31: gpio fields are unsigned short, no sensor_id */
    info->rst_gpio = (unsigned short)cfg->rst_gpio;
    info->pwdn_gpio = (unsigned short)cfg->pwdn_gpio;
#endif
}

/* ================================================================
 * DEVICE ID TRANSLATION
 * ================================================================ */

static IMPDeviceID hal_translate_dev_id(rss_dev_id_t dev)
{
    switch (dev) {
    case RSS_DEV_FS:
        return DEV_ID_FS;
    case RSS_DEV_ENC:
        return DEV_ID_ENC;
    case RSS_DEV_DEC:
        return DEV_ID_DEC;
    case RSS_DEV_IVS:
        return DEV_ID_IVS;
    case RSS_DEV_OSD:
        return DEV_ID_OSD;
    default:
        return DEV_ID_FS;
    }
}

/* ================================================================
 * SYSTEM LIFECYCLE: init / deinit
 * ================================================================ */

/*
 * hal_init -- full hardware init sequence.
 *
 * Order:
 *   1. Construct IMPSensorInfo from rss_sensor_config_t
 *   2. IMP_ISP_Open()
 *   3. IMP_ISP_AddSensor()
 *   4. IMP_ISP_EnableSensor()
 *   5. IMP_OSD_SetPoolSize()
 *   6. IMP_System_Init()
 *   7. IMP_ISP_EnableTuning()
 */
static int hal_init(void *ctx, const rss_multi_sensor_config_t *multi_cfg)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;
    int ret = 0;

    if (!c || !multi_cfg || multi_cfg->sensor_count < 1 ||
        multi_cfg->sensor_count > RSS_MAX_SENSORS)
        return -EINVAL;

    /* Save full config for deinit */
    memcpy(&c->multi_cfg, multi_cfg, sizeof(c->multi_cfg));
    c->sensor_count = multi_cfg->sensor_count;
    for (int i = 0; i < multi_cfg->sensor_count; i++)
        memcpy(&c->sensors[i], &multi_cfg->sensors[i], sizeof(c->sensors[i]));

    /* Step 1: construct IMPSensorInfo for each sensor */
    for (int i = 0; i < c->sensor_count; i++)
        hal_fill_sensor_info(&c->imp_sensors[i], &c->sensors[i]);

    /* Step 2: open ISP -- same signature on all SoCs */
    HAL_CHECK(IMP_ISP_Open(), err_out);

    /* Step 3: add and enable sensors */
#if defined(HAL_MULTI_SENSOR)
    /* T32/T40/T41: IMPVI_NUM per sensor */
    HAL_CHECK(IMP_ISP_AddSensor((IMPVI_NUM)0, &c->imp_sensors[0]), err_isp_close);
    HAL_CHECK(IMP_ISP_EnableSensor((IMPVI_NUM)0, &c->imp_sensors[0]), err_del_sensors);
    for (int i = 1; i < c->sensor_count; i++) {
        HAL_CHECK(IMP_ISP_AddSensor((IMPVI_NUM)i, &c->imp_sensors[i]), err_del_sensors);
        HAL_CHECK(IMP_ISP_EnableSensor((IMPVI_NUM)i, &c->imp_sensors[i]), err_del_sensors);
    }
#elif defined(HAL_T23_MULTICAM)
    /* T23 1.3.0: MIPI switch GPIO config must precede AddSensor */
    if (c->sensor_count > 1 && multi_cfg->mipi_switch.enable) {
        IMPUserSwitchgpio sgpio;
        memset(&sgpio, 0, sizeof(sgpio));
        sgpio.enable = 1;
        sgpio.sensornum = (uint16_t)c->sensor_count;
        if (c->sensor_count == 2) {
            sgpio.d.switch_gpio = multi_cfg->mipi_switch.switch_gpio;
            sgpio.d.Msensor_gstate = multi_cfg->mipi_switch.main_gstate;
            sgpio.d.Ssensor_gstate = multi_cfg->mipi_switch.sec_gstate;
        } else if (c->sensor_count == 3) {
            sgpio.t.switch_gpio[0] = multi_cfg->mipi_switch.switch_gpio;
            sgpio.t.switch_gpio[1] = multi_cfg->mipi_switch.switch_gpio2;
            sgpio.t.Msensor_gstate[0] = multi_cfg->mipi_switch.main_gstate;
            sgpio.t.Msensor_gstate[1] = 0;
            sgpio.t.Ssensor_gstate[0] = multi_cfg->mipi_switch.sec_gstate;
            sgpio.t.Ssensor_gstate[1] = 0;
            sgpio.t.Tsensor_gstate[0] = multi_cfg->mipi_switch.thr_gstate[0];
            sgpio.t.Tsensor_gstate[1] = multi_cfg->mipi_switch.thr_gstate[1];
        }
        HAL_CHECK(IMP_ISP_MultiCamera_SetSwitchgpio(&sgpio), err_isp_close);
    }
    /* T23: old-style AddSensor (I2C bus 0 sensor should be added last per vendor) */
    for (int i = 0; i < c->sensor_count; i++)
        HAL_CHECK(IMP_ISP_AddSensor(&c->imp_sensors[i]), err_del_sensors);
    /* T23: single EnableSensor enables all registered sensors */
    HAL_CHECK(IMP_ISP_EnableSensor(), err_del_sensors);
#else
    /* T20/T21/T30/T31: single sensor only */
    HAL_CHECK(IMP_ISP_AddSensor(&c->imp_sensors[0]), err_isp_close);
    HAL_CHECK(IMP_ISP_EnableSensor(), err_del_sensors);
#endif

    /* Step 5: set OSD pool size before System_Init (prudynt pattern) */
    IMP_OSD_SetPoolSize(1024 * 1024);
#if defined(PLATFORM_T23) || defined(PLATFORM_T32) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
    IMP_ISP_Tuning_SetOsdPoolSize(1024 * 1024);
#endif

    /* Step 6: init system (must be before EnableTuning — prudynt order) */
    HAL_CHECK(IMP_System_Init(), err_disable_sensor);

    /* Step 7: enable ISP tuning */
    HAL_CHECK(IMP_ISP_EnableTuning(), err_system_exit);

    return 0;

    /* ── Cleanup on failure (reverse order) ── */
err_disable_tuning:
    __attribute__((unused)) IMP_ISP_DisableTuning();
err_system_exit:
    IMP_System_Exit();
err_disable_sensor:
#if defined(HAL_MULTI_SENSOR)
    for (int i = c->sensor_count - 1; i >= 0; i--)
        IMP_ISP_DisableSensor((IMPVI_NUM)i);
#else
    IMP_ISP_DisableSensor();
#endif
err_del_sensors:
#if defined(HAL_MULTI_SENSOR)
    for (int i = c->sensor_count - 1; i >= 0; i--)
        IMP_ISP_DelSensor((IMPVI_NUM)i, &c->imp_sensors[i]);
#elif defined(HAL_T23_MULTICAM)
    for (int i = c->sensor_count - 1; i >= 0; i--)
        IMP_ISP_DelSensor(&c->imp_sensors[i]);
#else
    IMP_ISP_DelSensor(&c->imp_sensors[0]);
#endif
err_isp_close:
    IMP_ISP_Close();
err_out:
    return ret;
}

/*
 * hal_deinit -- full hardware teardown.
 *
 * Order:
 *   1. IMP_ISP_DisableTuning()
 *   2. IMP_System_Exit()
 *   3. IMP_ISP_DisableSensor()
 *   4. IMP_ISP_DelSensor()
 *   5. IMP_ISP_Close()
 */
static int hal_deinit(void *ctx)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;
    int ret = 0;
    int first_err = 0;

    if (!c)
        return -EINVAL;

    /* Step 1: disable tuning -- must happen before System_Exit */
    ret = IMP_ISP_DisableTuning();
    if (ret < 0 && first_err == 0)
        first_err = ret;

    /* Step 2: system exit */
    ret = IMP_System_Exit();
    if (ret < 0 && first_err == 0)
        first_err = ret;

    /* Step 3: disable sensor(s) */
#if defined(HAL_MULTI_SENSOR)
    for (int i = c->sensor_count - 1; i >= 0; i--) {
        ret = IMP_ISP_DisableSensor((IMPVI_NUM)i);
        if (ret < 0 && first_err == 0)
            first_err = ret;
    }
#else
    ret = IMP_ISP_DisableSensor();
    if (ret < 0 && first_err == 0)
        first_err = ret;
#endif

    /* Step 4: delete sensor(s) */
#if defined(HAL_MULTI_SENSOR)
    for (int i = c->sensor_count - 1; i >= 0; i--) {
        ret = IMP_ISP_DelSensor((IMPVI_NUM)i, &c->imp_sensors[i]);
        if (ret < 0 && first_err == 0)
            first_err = ret;
    }
#elif defined(HAL_T23_MULTICAM)
    for (int i = c->sensor_count - 1; i >= 0; i--) {
        ret = IMP_ISP_DelSensor(&c->imp_sensors[i]);
        if (ret < 0 && first_err == 0)
            first_err = ret;
    }
#else
    ret = IMP_ISP_DelSensor(&c->imp_sensors[0]);
    if (ret < 0 && first_err == 0)
        first_err = ret;
#endif

    /* Step 5: close ISP -- same signature on all SoCs */
    ret = IMP_ISP_Close();
    if (ret < 0 && first_err == 0)
        first_err = ret;

    return first_err;
}

/* ================================================================
 * BIND / UNBIND
 * ================================================================ */

/*
 * hal_bind -- bind a source cell to a destination cell.
 *
 * Translates rss_cell_t to IMPCell and calls IMP_System_Bind().
 * Signature is identical on all SoCs.
 */
static int hal_bind(void *ctx, const rss_cell_t *src, const rss_cell_t *dst)
{
    (void)ctx;
    IMPCell imp_src, imp_dst;

    if (!src || !dst)
        return -EINVAL;

    imp_src.deviceID = hal_translate_dev_id(src->device);
    imp_src.groupID = src->group;
    imp_src.outputID = src->output;

    imp_dst.deviceID = hal_translate_dev_id(dst->device);
    imp_dst.groupID = dst->group;
    imp_dst.outputID = dst->output;

    return IMP_System_Bind(&imp_src, &imp_dst);
}

/*
 * hal_unbind -- unbind a source cell from a destination cell.
 *
 * Translates rss_cell_t to IMPCell and calls IMP_System_UnBind().
 * Signature is identical on all SoCs.
 */
static int hal_unbind(void *ctx, const rss_cell_t *src, const rss_cell_t *dst)
{
    (void)ctx;
    IMPCell imp_src, imp_dst;

    if (!src || !dst)
        return -EINVAL;

    imp_src.deviceID = hal_translate_dev_id(src->device);
    imp_src.groupID = src->group;
    imp_src.outputID = src->output;

    imp_dst.deviceID = hal_translate_dev_id(dst->device);
    imp_dst.groupID = dst->group;
    imp_dst.outputID = dst->output;

    return IMP_System_UnBind(&imp_src, &imp_dst);
}

/* ================================================================
 * CAPABILITY QUERY
 * ================================================================ */

/*
 * hal_get_caps -- return the per-SoC capability struct.
 *
 * The caps are populated at create time from g_hal_caps (defined
 * in hal_caps.c, compiled per-SoC).
 */
static const rss_hal_caps_t *hal_get_caps(void *ctx)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;

    if (!c)
        return NULL;

    return &c->caps;
}

/* ================================================================
 * FACTORY FUNCTIONS
 * ================================================================ */

/*
 * rss_hal_create -- allocate and initialize a HAL context.
 *
 * Zero-initializes the context, copies the per-SoC caps from
 * g_hal_caps, and wires up the ops vtable pointer.
 *
 * Returns NULL on allocation failure.
 */
rss_hal_ctx_t *rss_hal_create(void)
{
    rss_hal_ctx_t *ctx;

    ctx = (rss_hal_ctx_t *)calloc(1, sizeof(*ctx));
    if (!ctx)
        return NULL;

    ctx->ops = &g_ops;
    memcpy(&ctx->caps, &g_hal_caps, sizeof(ctx->caps));

    return ctx;
}

/*
 * rss_hal_destroy -- free a HAL context and internal resources.
 *
 * Does NOT call deinit() -- the caller must do that first.
 */
void rss_hal_destroy(rss_hal_ctx_t *ctx)
{
    int i;

    if (!ctx)
        return;

    /* Free scratch buffer if allocated */
    if (ctx->scratch_buf) {
        free(ctx->scratch_buf);
        ctx->scratch_buf = NULL;
    }

    /* Free per-channel NAL arrays */
    for (i = 0; i < RSS_MAX_ENC_CHANNELS; i++) {
        if (ctx->nal_arrays[i]) {
            free(ctx->nal_arrays[i]);
            ctx->nal_arrays[i] = NULL;
        }
    }

    free(ctx);
}

/*
 * rss_hal_get_ops -- return the ops vtable from a context.
 *
 * Convenience accessor; equivalent to ctx->ops.
 */
const rss_hal_ops_t *rss_hal_get_ops(rss_hal_ctx_t *ctx)
{
    if (!ctx)
        return NULL;

    return ctx->ops;
}

/* ── System info (no vtable, called directly) ── */

int rss_hal_get_imp_version(char *buf, int size)
{
    IMPVersion ver;
    memset(&ver, 0, sizeof(ver));
    int ret = IMP_System_GetVersion(&ver);
    if (ret != 0)
        return ret;
    snprintf(buf, (size_t)size, "%s", ver.aVersion);
    return 0;
}

int rss_hal_get_sysutils_version(char *buf, int size)
{
    SUVersion ver;
    memset(&ver, 0, sizeof(ver));
    int ret = SU_Base_GetVersion(&ver);
    if (ret != 0)
        return ret;
    snprintf(buf, (size_t)size, "%s", ver.chr);
    return 0;
}

const char *rss_hal_get_cpu_info(void)
{
    return IMP_System_GetCPUInfo();
}
