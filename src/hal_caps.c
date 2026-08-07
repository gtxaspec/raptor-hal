/*
 * hal_caps.c -- Per-SoC capability struct initialization
 *
 * Provides a compile-time-constant rss_hal_caps_t for the target SoC.
 * Exactly one PLATFORM_* macro is defined by the build system; the
 * corresponding capability block is compiled and all others are excluded.
 *
 * Values are derived from the SDK difference analysis across all 10
 * supported Ingenic SoCs (T10, T20, T21, T23, T30, T31, T32, T33, T40, T41),
 * plus the SigmaStar Infinity6 families at the end of the chain.
 */

#include "hal_internal.h"
#include "raptor_hal.h"

/* ═══════════════════════════════════════════════════════════════════════
 * T20
 * ═══════════════════════════════════════════════════════════════════════ */
#if defined(PLATFORM_T20)
const rss_hal_caps_t g_hal_caps = {
    /* Encoder */
    .has_h265 = false,
    .has_rotation = false,
    .has_i2d = false,
    .has_bufshare = false,
    .has_set_default_param = false,
    .has_capped_rc = false,
    .has_smart_rc = true,
    .has_gop_attr = false,
    .has_set_bitrate = false,
    .has_stream_buf_size = false,
    .jpeg_pulse = true,
    .has_encoder_pool = false,
    .has_smartp_gop = false,
    .has_rc_options = false,
    .has_pskip = false,
    .has_srd = false,
    .has_max_pic_size = false,
    .has_super_frame = false,
    .has_color2grey = false,
    .has_roi = false,
    .has_map_roi = false,
    .has_qp_bounds_per_frame = false,
    .has_qpg_mode = false,
    .has_qpg_ai = false,
    .has_mbrc = false,
    .has_enc_denoise = false,
    .has_gdr = false,
    .has_sei_userdata = false,
    .has_h264_vui = false,
    .has_h265_vui = false,
    .has_h264_trans = false,
    .has_h265_trans = false,
    .has_enc_crop = false,
    .has_eval_info = false,
    .has_poll_module = false,
    .has_resize_mode = false,
    .has_jpeg_ql = false,
    .has_jpeg_qp = true,
    /* ISP */
    .has_multi_sensor = false,
    .max_sensors = 1,
    .has_t23_multicam_api = false,
    .has_defog = false,
    .has_dpc = false,
    .has_drc = false, /* T20 has SetRawDRC, not SetDRC_Strength */
    .has_face_ae = false,
    .has_bcsh_hue = false,
    .has_sinter = true,
    .has_temper = true,
    .has_highlight_depress = true,
    .has_backlight_comp = false,
    .has_ae_comp = true,
    .has_max_gain = true,
    .has_switch_bin = false,
    .has_gamma = true,
    .has_gamma_attr = false,
    .has_module_control = false,
    .has_wdr = true,
    /* OSD */
    .has_isp_osd = false,
    .has_osd_mosaic = false,
    .has_osd_group_callback = false,
    .has_osd_region_invert = false,
    .has_extended_osd_types = false,
    /* Audio */
    .has_audio_process_lib = true,
    .has_audio_aec_channel = false,
    .has_alc_gain = false,
    .has_agc_mode = false,
    .has_digital_gain = false,
    .has_howling_suppress = false,
    .has_hpf_cutoff = false,
    /* System */
    .uses_xburst2 = false,
    .uses_new_sdk = false,
    .uses_impvi = false,
    /* Limits */
    .max_enc_channels = 2,
    .max_osd_regions = 8,
    .max_osd_groups = 2,
    .max_isp_osd_regions = 0,
};

/* ═══════════════════════════════════════════════════════════════════════
 * T21
 * ═══════════════════════════════════════════════════════════════════════ */
#elif defined(PLATFORM_T21)
const rss_hal_caps_t g_hal_caps = {
    /* Encoder */
    .has_h265 = false, /* declared but marked "Unsupport" */
    .has_rotation = false,
    .has_i2d = false,
    .has_bufshare = false,
    .has_set_default_param = false,
    .has_capped_rc = false,
    .has_smart_rc = true,
    .has_gop_attr = false,
    .has_set_bitrate = false,
    .has_stream_buf_size = false,
    .jpeg_pulse = true,
    .has_encoder_pool = false,
    .has_smartp_gop = false,
    .has_rc_options = false,
    .has_pskip = false,
    .has_srd = false,
    .has_max_pic_size = false,
    .has_super_frame = true,
    .has_color2grey = true,
    .has_roi = true,
    .has_map_roi = false,
    .has_qp_bounds_per_frame = false,
    .has_qpg_mode = true,
    .has_qpg_ai = false,
    .has_mbrc = true,
    .has_enc_denoise = true,
    .has_gdr = false,
    .has_sei_userdata = true,
    .has_h264_vui = false,
    .has_h265_vui = false,
    .has_h264_trans = true,
    .has_h265_trans = true,
    .has_enc_crop = false,
    .has_eval_info = false,
    .has_poll_module = true,
    .has_resize_mode = false,
    .has_jpeg_ql = true,
    .has_jpeg_qp = false,
    /* ISP */
    .has_multi_sensor = false,
    .max_sensors = 1,
    .has_t23_multicam_api = false,
    .has_defog = false,
    .has_dpc = false,
    .has_drc = true,
    .has_face_ae = false,
    .has_bcsh_hue = false,
    .has_sinter = true,
    .has_temper = true,
    .has_highlight_depress = true,
    .has_backlight_comp = false,
    .has_ae_comp = false, /* SetAeComp absent on T21 */
    .has_max_gain = true,
    .has_switch_bin = false,
    .has_gamma = true,
    .has_gamma_attr = false,
    .has_module_control = true,
    .has_wdr = false,
    /* OSD */
    .has_isp_osd = false,
    .has_osd_mosaic = false,
    .has_osd_group_callback = false,
    .has_osd_region_invert = false,
    .has_extended_osd_types = false,
    /* Audio */
    .has_audio_process_lib = true,
    .has_audio_aec_channel = false,
    .has_alc_gain = true,
    .has_agc_mode = false,
    .has_digital_gain = false,
    .has_howling_suppress = false,
    .has_hpf_cutoff = false,
    /* System */
    .uses_xburst2 = false,
    .uses_new_sdk = false,
    .uses_impvi = false,
    /* Limits */
    .max_enc_channels = 2,
    .max_osd_regions = 8,
    .max_osd_groups = 2,
    .max_isp_osd_regions = 0,
};

/* ═══════════════════════════════════════════════════════════════════════
 * T23
 * ═══════════════════════════════════════════════════════════════════════ */
#elif defined(PLATFORM_T23)
const rss_hal_caps_t g_hal_caps = {
    /* Encoder */
    .has_h265 = false, /* declared but marked "Unsupport" */
    .has_rotation = false,
    .has_i2d = false,
    .has_bufshare = false,
    .has_set_default_param = false,
    .has_capped_rc = false,
    .has_smart_rc = true,
    .has_gop_attr = false,
    .has_set_bitrate = false,
    .has_stream_buf_size = false,
    .has_encoder_pool = true,
    .has_smartp_gop = false,
    .has_rc_options = false,
    .has_pskip = false,
    .has_srd = false,
    .has_max_pic_size = false,
    .has_super_frame = false,
    .has_color2grey = false,
    .has_roi = false,
    .has_map_roi = false,
    .has_qp_bounds_per_frame = false,
    .has_qpg_mode = false,
    .has_qpg_ai = false,
    .has_mbrc = false,
    .has_enc_denoise = false,
    .has_gdr = false,
    .has_sei_userdata = false,
    .has_h264_vui = false,
    .has_h265_vui = false,
    .has_h264_trans = false,
    .has_h265_trans = false,
    .has_enc_crop = false,
    .has_eval_info = false,
    .has_poll_module = false,
    .has_resize_mode = false,
    .has_jpeg_ql = false,
    .has_jpeg_qp = false,
    /* ISP */
    .has_multi_sensor = true, /* SDK 1.3.0 MultiCamera API */
    .max_sensors = 3,
    .has_t23_multicam_api = true,
    .has_defog = true,
    .has_dpc = true,
    .has_drc = true,
    .has_face_ae = false,
    .has_bcsh_hue = true,
    .has_sinter = true,
    .has_temper = true,
    .has_highlight_depress = true,
    .has_backlight_comp = true,
    .has_ae_comp = true,
    .has_max_gain = true,
    .has_switch_bin = false,
    .has_gamma = true,
    .has_gamma_attr = false,
    .has_module_control = true,
    .has_wdr = false,
    /* OSD */
    .has_isp_osd = true,
    .has_osd_mosaic = true,
    .has_osd_group_callback = false,
    .has_osd_region_invert = true,
    .has_extended_osd_types = true,
    /* Audio */
    .has_audio_process_lib = true,
    .has_audio_aec_channel = true,
    .has_alc_gain = false,
    .has_agc_mode = false,
    .has_digital_gain = false,
    .has_howling_suppress = true,
    .has_hpf_cutoff = true,
    /* System */
    .uses_xburst2 = false,
    .uses_new_sdk = false,
    .uses_impvi = false,
    /* Limits */
    .max_enc_channels = 6, /* vendor dual-sensor sample uses 6 */
    .max_osd_regions = 16,
    .max_osd_groups = 2,
    .max_isp_osd_regions = 8,
};

/* ═══════════════════════════════════════════════════════════════════════
 * T30
 * ═══════════════════════════════════════════════════════════════════════ */
#elif defined(PLATFORM_T30)
const rss_hal_caps_t g_hal_caps = {
    /* Encoder */
    .has_h265 = true,
    .has_rotation = false,
    .has_i2d = false,
    .has_bufshare = false,
    .has_set_default_param = false,
    .has_capped_rc = false,
    .has_smart_rc = true,
    .has_gop_attr = false,
    .has_set_bitrate = false,
    .has_stream_buf_size = false,
    .jpeg_pulse = true,
    .has_encoder_pool = false,
    .has_smartp_gop = false,
    .has_rc_options = false,
    .has_pskip = false,
    .has_srd = false,
    .has_max_pic_size = false,
    .has_super_frame = false,
    .has_color2grey = false,
    .has_roi = false,
    .has_map_roi = false,
    .has_qp_bounds_per_frame = false,
    .has_qpg_mode = false,
    .has_qpg_ai = false,
    .has_mbrc = false,
    .has_enc_denoise = false,
    .has_gdr = false,
    .has_sei_userdata = false,
    .has_h264_vui = false,
    .has_h265_vui = false,
    .has_h264_trans = false,
    .has_h265_trans = false,
    .has_enc_crop = false,
    .has_eval_info = false,
    .has_poll_module = false,
    .has_resize_mode = false,
    .has_jpeg_ql = false,
    .has_jpeg_qp = false,
    /* ISP */
    .has_multi_sensor = false,
    .max_sensors = 1,
    .has_t23_multicam_api = false,
    .has_defog = false,
    .has_dpc = false,
    .has_drc = false, /* T30 has SetRawDRC, not SetDRC_Strength */
    .has_face_ae = false,
    .has_bcsh_hue = false,
    .has_sinter = true,
    .has_temper = true,
    .has_highlight_depress = true,
    .has_backlight_comp = false,
    .has_ae_comp = true,
    .has_max_gain = true,
    .has_switch_bin = false,
    .has_gamma = true,
    .has_gamma_attr = false,
    .has_module_control = false,
    .has_wdr = true,
    /* OSD */
    .has_isp_osd = false,
    .has_osd_mosaic = false,
    .has_osd_group_callback = false,
    .has_osd_region_invert = false,
    .has_extended_osd_types = false,
    /* Audio */
    .has_audio_process_lib = true,
    .has_audio_aec_channel = false,
    .has_alc_gain = false,
    .has_agc_mode = false,
    .has_digital_gain = false,
    .has_howling_suppress = false,
    .has_hpf_cutoff = false,
    /* System */
    .uses_xburst2 = false,
    .uses_new_sdk = false,
    .uses_impvi = false,
    /* Limits */
    .max_enc_channels = 2,
    .max_osd_regions = 8,
    .max_osd_groups = 2,
    .max_isp_osd_regions = 0,
};

/* ═══════════════════════════════════════════════════════════════════════
 * T31
 * ═══════════════════════════════════════════════════════════════════════ */
#elif defined(PLATFORM_T31)
const rss_hal_caps_t g_hal_caps = {
    /* Encoder */
    .has_h265 = true,
    .has_rotation = true,
    .has_i2d = false,
    .has_bufshare = true,
    .has_set_default_param = true,
    .has_capped_rc = true,
    .has_smart_rc = false,
    .has_gop_attr = true,
    .has_set_bitrate = true,
    .has_stream_buf_size = true,
    .has_encoder_pool = true,
    .has_smartp_gop = true,
    .has_rc_options = true,
    .has_pskip = false,
    .has_srd = false,
    .has_max_pic_size = false,
    .has_super_frame = false,
    .has_color2grey = false,
    .has_roi = false,
    .has_map_roi = false,
    .has_qp_bounds_per_frame = false,
    .has_qpg_mode = false,
    .has_qpg_ai = false,
    .has_mbrc = false,
    .has_enc_denoise = false,
    .has_gdr = false,
    .has_sei_userdata = false,
    .has_h264_vui = false,
    .has_h265_vui = false,
    .has_h264_trans = false,
    .has_h265_trans = false,
    .has_enc_crop = false,
    .has_eval_info = true,
    .has_poll_module = true,
    .has_resize_mode = true,
    .has_jpeg_ql = false,
    .has_jpeg_qp = false,
    /* ISP */
    .has_multi_sensor = false,
    .max_sensors = 1,
    .has_t23_multicam_api = false,
    .has_defog = true,
    .has_dpc = true,
    .has_drc = true,
    .has_face_ae = false,
    .has_bcsh_hue = true,
    .has_sinter = true,
    .has_temper = true,
    .has_highlight_depress = true,
    .has_backlight_comp = true,
    .has_ae_comp = true,
    .has_max_gain = true,
    .has_switch_bin = false,
    .has_gamma = true,
    .has_gamma_attr = false,
    .has_module_control = true,
    .has_wdr = true,
    /* OSD */
    .has_isp_osd = false,
    .has_osd_mosaic = false,
    .has_osd_group_callback = false,
    .has_osd_region_invert = true,
    .has_extended_osd_types = false,
    /* Audio */
    .has_audio_process_lib = true,
    .has_audio_aec_channel = false,
    .has_alc_gain = true,
    .has_agc_mode = true,
    .has_digital_gain = false,
    .has_howling_suppress = false,
    .has_hpf_cutoff = true,
    /* System */
    .uses_xburst2 = false,
    .uses_new_sdk = true,
    .uses_impvi = false,
    /* Limits */
    .max_enc_channels = 3,
    .max_osd_regions = 8,
    .max_osd_groups = 2,
    .max_isp_osd_regions = 0,
};

/* ═══════════════════════════════════════════════════════════════════════
 * T32
 * ═══════════════════════════════════════════════════════════════════════ */
#elif defined(PLATFORM_T32)
const rss_hal_caps_t g_hal_caps = {
    /* Encoder */
    .has_h265 = true,
    .has_rotation = false,
    .has_i2d = true,
    .has_bufshare = false, /* SetbufshareChn not present on T32 */
    .has_set_default_param = true,
    .has_capped_rc = true,
    .has_smart_rc = true,  /* T32 has ENC_RC_MODE_SMART (mode 3) */
    .has_gop_attr = false, /* T32 sets GOP via SetDefaultParam arg */
    .has_set_bitrate = true,
    .has_stream_buf_size = false,
    .has_encoder_pool = true,
    .has_smartp_gop = false,
    .has_rc_options = false,
    .has_pskip = true,
    .has_srd = true,
    .has_max_pic_size = true,
    .has_super_frame = true,
    .has_color2grey = false,
    .has_roi = true,
    .has_map_roi = true,
    .has_qp_bounds_per_frame = true,
    .has_qpg_mode = true,
    .has_qpg_ai = true,
    .has_mbrc = false,
    .has_enc_denoise = false,
    .has_gdr = true,
    .has_sei_userdata = true,
    .has_h264_vui = true,
    .has_h265_vui = true,
    .has_h264_trans = true,
    .has_h265_trans = true,
    .has_enc_crop = true,
    .has_eval_info = false,
    .has_poll_module = false,
    .has_resize_mode = false,
    .has_jpeg_ql = false,
    .has_jpeg_qp = true,
    /* ISP */
    .has_multi_sensor = true, /* IMPVI_NUM, up to 4 sensors */
    .max_sensors = 3,
    .has_t23_multicam_api = false,
    .has_defog = false,
    .has_dpc = false,
    .has_drc = false, /* via SetModuleControl only */
    .has_face_ae = true,
    .has_bcsh_hue = true,
    .has_sinter = false, /* via SetModuleControl only */
    .has_temper = false, /* via SetModuleControl only */
    .has_highlight_depress = false,
    .has_backlight_comp = false,
    .has_ae_comp = false,
    .has_max_gain = false,
    .has_switch_bin = true,
    .has_gamma = false,
    .has_gamma_attr = true,
    .has_module_control = true,
    .has_wdr = true,
    /* OSD */
    .has_isp_osd = true,
    .has_osd_mosaic = true,
    .has_osd_group_callback = false,
    .has_osd_region_invert = true,
    .has_extended_osd_types = true,
    /* Audio */
    .has_audio_process_lib = true,
    .has_audio_aec_channel = true,
    .has_alc_gain = false,
    .has_agc_mode = false,
    .has_digital_gain = true,
    .has_howling_suppress = true,
    .has_hpf_cutoff = true,
    /* System */
    .uses_xburst2 = false,
    .uses_new_sdk = true,
    .uses_impvi = true,
    /* Limits */
    .max_enc_channels = 3,
    .max_osd_regions = 16,
    .max_osd_groups = 2,
    .max_isp_osd_regions = 8,
};

/* ═══════════════════════════════════════════════════════════════════════
 * T33 (T32-compatible, no DMIC/DVP/WDR)
 * ═══════════════════════════════════════════════════════════════════════ */
#elif defined(PLATFORM_T33)
const rss_hal_caps_t g_hal_caps = {
    /* Encoder */
    .has_h265 = true,
    .has_rotation = false,
    .has_i2d = true,
    .has_bufshare = false,
    .has_set_default_param = true,
    .has_capped_rc = true,
    .has_smart_rc = true,
    .has_gop_attr = false,
    .has_set_bitrate = true,
    .has_stream_buf_size = false,
    .has_encoder_pool = true,
    .has_smartp_gop = false,
    .has_rc_options = false,
    .has_pskip = true,
    .has_srd = true,
    .has_max_pic_size = true,
    .has_super_frame = true,
    .has_color2grey = false,
    .has_roi = true,
    .has_map_roi = true,
    .has_qp_bounds_per_frame = true,
    .has_qpg_mode = true,
    .has_qpg_ai = true,
    .has_mbrc = false,
    .has_enc_denoise = false,
    .has_gdr = true,
    .has_sei_userdata = true,
    .has_h264_vui = true,
    .has_h265_vui = true,
    .has_h264_trans = true,
    .has_h265_trans = true,
    .has_enc_crop = true,
    .has_eval_info = false,
    .has_poll_module = false,
    .has_resize_mode = false,
    .has_jpeg_ql = false,
    .has_jpeg_qp = true,
    /* ISP */
    .has_multi_sensor = true,
    .max_sensors = 3,
    .has_t23_multicam_api = false,
    .has_defog = false,
    .has_dpc = false,
    .has_drc = false,
    .has_face_ae = true,
    .has_bcsh_hue = true,
    .has_sinter = false,
    .has_temper = false,
    .has_highlight_depress = false,
    .has_backlight_comp = false,
    .has_ae_comp = false,
    .has_max_gain = false,
    .has_switch_bin = true,
    .has_gamma = false,
    .has_gamma_attr = true,
    .has_module_control = true,
    .has_wdr = false,
    /* OSD */
    .has_isp_osd = true,
    .has_osd_mosaic = true,
    .has_osd_group_callback = false,
    .has_osd_region_invert = true,
    .has_extended_osd_types = true,
    /* Audio */
    .has_audio_process_lib = true,
    .has_audio_aec_channel = true,
    .has_alc_gain = false,
    .has_agc_mode = false,
    .has_digital_gain = true,
    .has_howling_suppress = true,
    .has_hpf_cutoff = true,
    /* System */
    .uses_xburst2 = false,
    .uses_new_sdk = true,
    .uses_impvi = true,
    /* Limits */
    .max_enc_channels = 3,
    .max_osd_regions = 16,
    .max_osd_groups = 2,
    .max_isp_osd_regions = 8,
};

/* ═══════════════════════════════════════════════════════════════════════
 * T40
 * ═══════════════════════════════════════════════════════════════════════ */
#elif defined(PLATFORM_T40)
const rss_hal_caps_t g_hal_caps = {
    /* Encoder */
    .has_h265 = true,
    .has_rotation = false,
    .has_i2d = true,
    .has_bufshare = true,
    .has_set_default_param = true,
    .has_capped_rc = true,
    .has_smart_rc = false,
    .has_gop_attr = true,
    .has_set_bitrate = true,
    .has_stream_buf_size = true,
    .has_encoder_pool = true,
    .has_smartp_gop = true,
    .has_rc_options = true,
    .has_pskip = false,
    .has_srd = false,
    .has_max_pic_size = false,
    .has_super_frame = false,
    .has_color2grey = false,
    .has_roi = false,
    .has_map_roi = false,
    .has_qp_bounds_per_frame = false,
    .has_qpg_mode = false,
    .has_qpg_ai = false,
    .has_mbrc = false,
    .has_enc_denoise = false,
    .has_gdr = false,
    .has_sei_userdata = false,
    .has_h264_vui = false,
    .has_h265_vui = false,
    .has_h264_trans = false,
    .has_h265_trans = false,
    .has_enc_crop = false,
    .has_eval_info = false,
    .has_poll_module = false,
    .has_resize_mode = false,
    .has_jpeg_ql = false,
    .has_jpeg_qp = false,
    /* ISP */
    .has_multi_sensor = true,
    .max_sensors = 3,
    .has_t23_multicam_api = false,
    .has_defog = false,
    .has_dpc = false,
    .has_drc = false, /* via SetModuleControl only */
    .has_face_ae = true,
    .has_bcsh_hue = true,
    .has_sinter = false, /* via SetModuleControl only */
    .has_temper = false, /* via SetModuleControl only */
    .has_highlight_depress = false,
    .has_backlight_comp = false,
    .has_ae_comp = false,
    .has_max_gain = false,
    .has_switch_bin = true,
    .has_gamma = false,
    .has_gamma_attr = true,
    .has_module_control = true,
    .has_wdr = true,
    /* OSD */
    .has_isp_osd = true,
    .has_osd_mosaic = true,
    .has_osd_group_callback = true,
    .has_osd_region_invert = true,
    .has_extended_osd_types = true,
    /* Audio */
    .has_audio_process_lib = true,
    .has_audio_aec_channel = true,
    .has_alc_gain = false,
    .has_agc_mode = false,
    .has_digital_gain = true,
    .has_howling_suppress = true,
    .has_hpf_cutoff = true,
    /* System */
    .uses_xburst2 = true,
    .uses_new_sdk = true,
    .uses_impvi = true,
    /* Limits */
    .max_enc_channels = 4,
    .max_osd_regions = 16,
    .max_osd_groups = 4,
    .max_isp_osd_regions = 8,
};

/* ═══════════════════════════════════════════════════════════════════════
 * T41
 * ═══════════════════════════════════════════════════════════════════════ */
#elif defined(PLATFORM_T41)
const rss_hal_caps_t g_hal_caps = {
    /* Encoder */
    .has_h265 = true,
    .has_rotation = false,
    .has_i2d = true,
    .has_bufshare = true,
    .has_set_default_param = true,
    .has_capped_rc = true,
    .has_smart_rc = false,
    .has_gop_attr = true,
    .has_set_bitrate = true,
    .has_stream_buf_size = true,
    .has_encoder_pool = true,
    .has_smartp_gop = true,
    .has_rc_options = true,
    .has_pskip = false,
    .has_srd = false,
    .has_max_pic_size = true,
    .has_super_frame = false,
    .has_color2grey = false,
    .has_roi = false,
    .has_map_roi = false,
    .has_qp_bounds_per_frame = true,
    .has_qpg_mode = false,
    .has_qpg_ai = false,
    .has_mbrc = false,
    .has_enc_denoise = false,
    .has_gdr = false,
    .has_sei_userdata = false,
    .has_h264_vui = false,
    .has_h265_vui = false,
    .has_h264_trans = false,
    .has_h265_trans = false,
    .has_enc_crop = false,
    .has_eval_info = false,
    .has_poll_module = true,
    .has_resize_mode = true,
    .has_jpeg_ql = true,
    .has_jpeg_qp = false,
    /* ISP */
    .has_multi_sensor = true, /* IMPVI_NUM defined but SEC/THR unsupported */
    .max_sensors = 3,         /* experimental — vendor docs say SEC/THR not yet functional */
    .has_t23_multicam_api = false,
    .has_defog = false,
    .has_dpc = false,
    .has_drc = false, /* via SetModuleControl only */
    .has_face_ae = true,
    .has_bcsh_hue = true,
    .has_sinter = false, /* via SetModuleControl only */
    .has_temper = false, /* via SetModuleControl only */
    .has_highlight_depress = false,
    .has_backlight_comp = false,
    .has_ae_comp = false,
    .has_max_gain = false,
    .has_switch_bin = true,
    .has_gamma = false,
    .has_gamma_attr = true,
    .has_module_control = true,
    .has_wdr = true,
    /* OSD */
    .has_isp_osd = true,
    .has_osd_mosaic = true,
    .has_osd_group_callback = true,
    .has_osd_region_invert = true,
    .has_extended_osd_types = true,
    /* Audio */
    .has_audio_process_lib = true,
    .has_audio_aec_channel = true,
    .has_alc_gain = false,
    .has_agc_mode = false,
    .has_digital_gain = true,
    .has_howling_suppress = true,
    .has_hpf_cutoff = true,
    /* System */
    .uses_xburst2 = true,
    .uses_new_sdk = true,
    .uses_impvi = true,
    /* Limits */
    .max_enc_channels = 4,
    .max_osd_regions = 16,
    .max_osd_groups = 4,
    .max_isp_osd_regions = 8,
};

/* ═══════════════════════════════════════════════════════════════════════
 * INFINITY6E (SigmaStar SSC30KQ / SSC338Q)
 *
 * Only fields with a positive value or a permanent-false reason are listed;
 * the rest default to false/0. A false means either that MI has no
 * equivalent, noted individually, or that no hal_* op is wired to it --
 * consumers check these flags before calling optional ops, so false is what
 * keeps rvd/rsd off a NULL vtable entry.
 * ═══════════════════════════════════════════════════════════════════════ */
#elif defined(PLATFORM_INFINITY6E)
const rss_hal_caps_t g_hal_caps = {
    .soc_name = HAL_PLATFORM_NAME,
    /* Compile-time fallback; MI_SYS_GetVersion() supplies the real one. */
    .sdk_version = "MI",

    /* H.264, H.265 and MJPEG are all in hardware (i6_venc_codec). */
    .has_h265 = true,
    /* MI offers CBR/VBR/ABR/FIXQP/AVBR and nothing equivalent to Ingenic's
     * SMART mode. */
    .has_smart_rc = false,

    /* MI supports multi-sensor on some parts, but raptor's multi-sensor path
     * is Ingenic IMPVI-specific. */
    .has_multi_sensor = false,
    .max_sensors = 1,
    /* T23-specific IMP_ISP_MultiCamera_* API. */
    .has_t23_multicam_api = false,

    /*
     * True only where MI has a control that matches the raptor knob: defog
     * (MI_ISP_IQ_SetDefog, a toggle), sinter and temper (NRLuma and NR3D,
     * both 0..255), ae_comp (MI_ISP_AE_SetEVComp) and max_gain (MI's AE
     * exposure-limit struct). The rest stay false for the reasons in
     * hal_isp.c's OP COVERAGE comment.
     *
     * These five are the least portable entries in the table. Each knob is
     * reached by poking a field at a hardcoded (payload size, offset) pair
     * inside an opaque MI IQ struct, and those numbers describe one
     * INFINITY6E libmi_isp.so build rather than the MI API, so they do not
     * transfer to another Infinity6 family unmeasured. A wrong offset does
     * not fail: access is read-modify-write, so it lands a plausible value
     * in the wrong field and the image quietly degrades.
     */
    .has_defog = true,
    .has_sinter = true,
    .has_temper = true,
    .has_ae_comp = true,
    .has_max_gain = true,

    /*
     * Every audio capability stays false. The VQE flags
     * (has_audio_process_lib, has_agc_mode, has_hpf_cutoff,
     * has_howling_suppress, has_audio_aec_channel) are not merely
     * unimplemented -- enabling one crashes. MI's Iaa* algorithm packs are
     * absent from every library shipped for this family, while the
     * MI_AI_*Vqe* wrappers are present, so nothing fails at load; the call
     * sites then reach the missing symbols through the PLT with no null
     * guard, and an unresolved weak symbol's GOT slot is 0. MI's own
     * reference drops the algorithm functions from version 2.19.
     *
     * has_alc_gain and has_digital_gain describe two gain stages. MI has one
     * input gain control and audio_set_volume owns it.
     */

    /* Ingenic internals -- xburst2 core, IMP SDK generation, IMPVI calling
     * convention -- permanently false for any other vendor. */
    .uses_xburst2 = false,
    .uses_new_sdk = false,
    .uses_impvi = false,

    /*
     * The family exposes 9 addressable VENC channels (I6_VENC_CHN_NUM),
     * capped here at RSS_MAX_ENC_CHANNELS (8) because that is raptor's array
     * bound and so the most this field can honestly advertise.
     *
     * max_fs_channels is 4 because a raptor framesource channel is a VPE
     * output port and a VPE channel has four; all four were measured
     * accepting MI_VPE_SetPortMode at 640x360 NV12.
     *
     * max_osd_groups is 4 because a group here is an encoder channel with a
     * bound VPE port. The has_osd_* flags stay false: MI has only OSD and
     * COVER region types, no group callback, and rss_osd_region_t has no
     * field to drive its invert attribute.
     */
    .max_enc_channels = 8,
    .max_fs_channels = 4,
    /* The backend's own tracking bound, not an MI limit -- MI publishes
     * none. Keep in step with STAR_OSD_REGION_MAX; not referenced
     * symbolically because this file compiles for every platform and must
     * not pull in the MI headers. */
    .max_osd_regions = 16,
    .max_osd_groups = 4,
};

#else
#error "No PLATFORM_* defined. Set one of: PLATFORM_T10 T20 T21 T23 T30 T31 T32 T33 T40 T41 INFINITY6E"
#endif
