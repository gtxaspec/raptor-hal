/*
 * hal_caps.c -- Per-SoC capability struct initialization
 *
 * Provides a compile-time-constant rss_hal_caps_t for the target SoC.
 * Exactly one PLATFORM_* macro is defined by the build system; the
 * corresponding capability block is compiled and all others are excluded.
 *
 * Values are derived from the SDK difference analysis across all 10
 * supported Ingenic SoCs (T10, T20, T21, T23, T30, T31, T32, T33, T40, T41).
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
 * Two kinds of false appear below, and the distinction matters:
 *
 *   1. Hardware/SDK facts — the MI SDK genuinely has no equivalent.
 *      These stay false permanently and are commented individually.
 *   2. Not-yet-implemented — the capability may well exist, but the
 *      corresponding hal_* op is not wired up yet. Consumers check these
 *      flags before calling optional ops, so declaring false keeps
 *      rvd/rsd from invoking a NULL vtable entry. Each later phase flips
 *      on the flags it implements (ISP -> phase 3, audio -> 4, OSD -> 5).
 *
 * Since the backend currently populates no video/audio ops at all,
 * everything in category 2 is false. Only fields with a positive value
 * or a permanent-false explanation are listed; the rest default to
 * false/0 via designated initialization.
 * ═══════════════════════════════════════════════════════════════════════ */
#elif defined(PLATFORM_INFINITY6E)
const rss_hal_caps_t g_hal_caps = {
    /* System info */
    .soc_name = "INFINITY6E",
    /* Real value comes from MI_SYS_GetVersion() at runtime (phase 2);
     * this is only the compile-time fallback string. */
    .sdk_version = "MI",

    /* Encoder — H.264/H.265/MJPEG are all present in hardware
     * (i6_venc_codec: I6_VENC_CODEC_H264/H265/MJPG). */
    .has_h265 = true,
    /* Permanent false: MI rate control offers CBR/VBR/ABR/FIXQP/AVBR
     * (i6_venc_ratemode) with no equivalent of Ingenic's SMART mode. */
    .has_smart_rc = false,

    /* ISP — single sensor for now; MI supports multi-sensor on some parts
     * but raptor's multi-sensor path is Ingenic IMPVI-specific. */
    .has_multi_sensor = false,
    .max_sensors = 1,
    /* Permanent false: T23-specific IMP_ISP_MultiCamera_* API. */
    .has_t23_multicam_api = false,

    /* System — all three describe Ingenic internals (xburst2 core, IMP SDK
     * generation, IMPVI multi-sensor calling convention) and are
     * permanently false for any non-Ingenic vendor. */
    .uses_xburst2 = false,
    .uses_new_sdk = false,
    .uses_impvi = false,

    /* Limits — Infinity6E exposes 9 addressable VENC channels
     * (I6_VENC_CHN_NUM in both divinus's i6_venc.h and waybeam's
     * sigmastar_types.h; divinus allocates channel state for all 9 and
     * iterates them). An earlier 3 here came from misreading
     * MI_VENC_MAX_CHN_NUM_PER_DC, which is the per-device-group limit and
     * not the total. Capped at RSS_MAX_ENC_CHANNELS (8) since that is
     * raptor's array bound, so 8 is the most this field can honestly
     * advertise.
     * OSD limits stay 0 until the region backend lands in phase 5.
     *
     * max_fs_channels is 4 because a raptor framesource channel is a VPE
     * output port (src/star/hal_framesource.c) and a VPE channel has four
     * of them: divinus's teardown disables ports 0..3 (i6_hal.c:365) and
     * waybeam uses 0 and 1. Measured on an SSC30KQ: star_probe -v had all
     * four ports accept MI_VPE_SetPortMode at 640x360 NV12. */
    .max_enc_channels = 8,
    .max_fs_channels = 4,
};

#else
#error "No PLATFORM_* defined. Set one of: PLATFORM_T10 T20 T21 T23 T30 T31 T32 T33 T40 T41 INFINITY6E"
#endif
