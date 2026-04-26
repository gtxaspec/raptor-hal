/*
 * raptor_hal.h -- Raptor Streaming System Hardware Abstraction Layer
 *
 * This is the ONLY header that RSS daemons include for hardware access.
 * All Ingenic IMP SDK types are abstracted behind RSS types.
 *
 * Consumers:
 *   RVD  -- video daemon (ISP, framesource, encoder, OSD, frame output)
 *   RAD  -- audio daemon (audio input, encoding, frame output)
 *   RIC  -- IR control daemon (exposure queries)
 */

#ifndef RAPTOR_HAL_H
#define RAPTOR_HAL_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <errno.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 * Error codes
 * ================================================================ */

#define RSS_OK 0
#define RSS_ERR (-1)
#define RSS_ERR_NOTSUP (-ENOTSUP)
#define RSS_ERR_TIMEOUT (-ETIMEDOUT)
#define RSS_ERR_INVAL (-EINVAL)
#define RSS_ERR_NOMEM (-ENOMEM)
#define RSS_ERR_IO (-EIO)
#define RSS_ERR_BUSY (-EBUSY)
#define RSS_ERR_NOENT (-ENOENT)

/* ================================================================
 * HAL Types
 * ================================================================ */

/* Video codec selection */
typedef enum {
    RSS_CODEC_H264 = 0,
    RSS_CODEC_H265 = 1,
    RSS_CODEC_JPEG = 2,
    RSS_CODEC_MJPEG = 3,
} rss_codec_t;

/* Rate control mode */
typedef enum {
    RSS_RC_FIXQP = 0,
    RSS_RC_CBR = 1,
    RSS_RC_VBR = 2,
    RSS_RC_SMART = 3,          /* old SDK; mapped to CAPPED_VBR on new */
    RSS_RC_CAPPED_VBR = 4,     /* new SDK; mapped to VBR on old */
    RSS_RC_CAPPED_QUALITY = 5, /* new SDK; mapped to VBR on old */
} rss_rc_mode_t;

/* GOP control mode (T31/T40/T41 only) */
typedef enum {
    RSS_GOP_DEFAULT = 0,
    RSS_GOP_PYRAMIDAL = 1,
    RSS_GOP_SMARTP = 2,
} rss_gop_mode_t;

/* RC options bitmask (T31/T40/T41 only; combine with |) */
#define RSS_RC_OPT_NONE 0x00
#define RSS_RC_OPT_SCN_CHG_RES 0x01
#define RSS_RC_OPT_DELAYED 0x02
#define RSS_RC_OPT_STATIC_SCENE 0x04
#define RSS_RC_OPT_ENABLE_SKIP 0x08
#define RSS_RC_OPT_SC_PREVENTION 0x10

/* Super frame mode */
typedef enum {
    RSS_SUPERFRM_NONE = 0,
    RSS_SUPERFRM_DISCARD = 1,
    RSS_SUPERFRM_REENCODE = 2,
} rss_super_frame_mode_t;

/* RC priority for super frame handling */
typedef enum {
    RSS_RC_PRIO_RDO = 0,
    RSS_RC_PRIO_BITRATE = 1,
    RSS_RC_PRIO_FRAMEBITS = 2,
} rss_rc_priority_t;

/* Pixel format */
typedef enum {
    RSS_PIXFMT_NV12 = 0,
    RSS_PIXFMT_NV21 = 1,
    RSS_PIXFMT_YUYV422 = 2,
    RSS_PIXFMT_UYVY422 = 3,
    RSS_PIXFMT_YUV420P = 4,
    RSS_PIXFMT_RGB24 = 5,
    RSS_PIXFMT_BGR24 = 6,
    RSS_PIXFMT_BGRA = 7,
    RSS_PIXFMT_ARGB = 8,
    RSS_PIXFMT_RGB565LE = 9,
    RSS_PIXFMT_GRAY8 = 10,
    RSS_PIXFMT_RAW = 11,
    RSS_PIXFMT_RAW8 = 12,  /* T32/T40/T41 only */
    RSS_PIXFMT_RAW16 = 13, /* T32/T40/T41 only */
} rss_pixfmt_t;

/* Abstracted NAL unit type */
typedef enum {
    /* H.264 */
    RSS_NAL_H264_SPS = 0x10,
    RSS_NAL_H264_PPS = 0x11,
    RSS_NAL_H264_SEI = 0x12,
    RSS_NAL_H264_IDR = 0x13,
    RSS_NAL_H264_SLICE = 0x14,

    /* H.265 */
    RSS_NAL_H265_VPS = 0x20,
    RSS_NAL_H265_SPS = 0x21,
    RSS_NAL_H265_PPS = 0x22,
    RSS_NAL_H265_SEI = 0x23,
    RSS_NAL_H265_IDR = 0x24,
    RSS_NAL_H265_SLICE = 0x25,

    /* JPEG */
    RSS_NAL_JPEG_FRAME = 0x30,

    RSS_NAL_UNKNOWN = 0xFF,
} rss_nal_type_t;

/* Single NAL unit within an encoded frame */
typedef struct {
    const uint8_t *data; /* contiguous NAL payload */
    uint32_t length;     /* payload length in bytes */
    rss_nal_type_t type;
    bool frame_end; /* last NAL in the frame */
} rss_nal_unit_t;

/* Encoded video frame (returned by enc_get_frame) */
typedef struct {
    rss_nal_unit_t *nals;
    uint32_t nal_count;
    rss_codec_t codec;
    int64_t timestamp; /* capture timestamp in microseconds */
    uint32_t seq;
    bool is_key;

    /* HAL-internal: do not touch */
    void *_priv;
} rss_frame_t;

/* Audio frame (returned by audio_read_frame) */
typedef struct {
    const int16_t *data;
    uint32_t length; /* data length in bytes */
    int64_t timestamp;
    uint32_t seq;

    /* HAL-internal */
    void *_priv;
} rss_audio_frame_t;

/* Encoder channel configuration */
typedef struct {
    rss_codec_t codec;
    uint16_t width;
    uint16_t height;
    int profile; /* H264: 0=base,1=main,2=high; H265: ignored */

    /* Rate control */
    rss_rc_mode_t rc_mode;
    uint32_t bitrate; /* target bitrate in bps */
    uint32_t max_bitrate;
    int16_t init_qp;   /* -1 for SDK default */
    int16_t min_qp;    /* [0..51] */
    int16_t max_qp;    /* [0..51] */
    int16_t ip_delta;  /* QP delta I vs P frame; -1 = SDK default (T31+) */
    int16_t pb_delta;  /* QP delta P vs B frame; -1 = SDK default (T31+) */
    uint16_t max_psnr; /* PSNR quality cap for capped_vbr/capped_quality; 0 = SDK default (T31+) */

    /* Frame rate */
    uint32_t fps_num;
    uint32_t fps_den;

    /* GOP */
    uint32_t gop_length;
    rss_gop_mode_t gop_mode;     /* T31/T40/T41; 0 = default */
    uint32_t max_same_scene_cnt; /* T31/T40/T41; 0 = SDK default (2) */

    /* RC options bitmask (T31/T40/T41; RSS_RC_OPT_*) */
    uint32_t rc_options;

    /* Buffer size hint; 0 = SDK default */
    uint32_t buf_size;

    /* Stream buffer tuning (T31+, before CreateChn) */
    uint8_t max_stream_cnt;   /* 0 = SDK default (2); encoder output buffer count */
    uint32_t stream_buf_size; /* 0 = SDK default; per-buffer size in bytes */

    /* IVDC (ISP-VPU Direct Connect) — T23+ only, main channel only */
    bool ivdc;
} rss_video_config_t;

/* Super frame configuration (T21/T32) */
typedef struct {
    rss_super_frame_mode_t mode;
    uint32_t i_bits_thr; /* I frame size threshold (bits) */
    uint32_t p_bits_thr; /* P frame size threshold (bits) */
    rss_rc_priority_t priority;
    uint8_t max_reencode; /* max re-encode attempts [1,3]; T32 only */
} rss_super_frame_cfg_t;

/* P-skip configuration (T32 only) */
typedef struct {
    bool enable;
    int max_frames; /* max consecutive P frames in skip mode */
    int threshold;  /* bitrate exceed factor * 100 */
} rss_pskip_cfg_t;

/* SRD configuration (T32 only, H265) */
typedef struct {
    bool enable;
    uint8_t level; /* [0..3], higher = more aggressive static scene optimization */
} rss_srd_cfg_t;

/* ROI region configuration (T21: 8 regions, T32: 16 regions) */
typedef struct {
    uint32_t index; /* region index (0-7 on T21, 0-15 on T32) */
    bool enable;
    bool relative_qp; /* false=absolute QP, true=relative to frame QP */
    int qp;           /* QP value (absolute) or delta (relative) */
    int x, y, w, h;   /* region rectangle in pixels */
} rss_enc_roi_t;

/* Encoder denoise configuration (T21 only) */
typedef struct {
    bool enable;
    int dn_type; /* 0=off, 1=I+P frame denoise, 2=I-only denoise */
    int dn_i_qp; /* denoise I frame QP */
    int dn_p_qp; /* denoise P frame QP */
} rss_enc_denoise_cfg_t;

/* GDR (Gradual Decoder Refresh) configuration (T32 only) */
typedef struct {
    bool enable;
    int gdr_cycle;  /* interval between GDR P frames [3..65535] */
    int gdr_frames; /* P frames split from I frame [2..10] */
} rss_enc_gdr_cfg_t;

/* H.264 transform configuration */
typedef struct {
    int chroma_qp_index_offset; /* [-12..12] */
} rss_enc_h264_trans_t;

/* H.265 transform configuration */
typedef struct {
    int chroma_cr_qp_offset; /* [-12..12] */
    int chroma_cb_qp_offset; /* [-12..12] */
} rss_enc_h265_trans_t;

/* Encoder crop configuration (T32 only) */
typedef struct {
    bool enable;
    uint32_t x, y, w, h;
} rss_enc_crop_cfg_t;

/* JPEG custom quantization table */
typedef struct {
    bool user_table_en;      /* false=SDK default, true=use custom table */
    uint8_t qmem_table[128]; /* custom quantization table */
} rss_enc_jpeg_ql_t;

/* Raw frame info (returned by fs_get_frame / fs_snap_frame) */
typedef struct {
    uint16_t width;
    uint16_t height;
    rss_pixfmt_t pixfmt;
    int64_t timestamp;
    uint32_t phys_addr;
    void *virt_addr;
    uint32_t size;
} rss_frame_info_t;

/* IVS motion detection parameters (RSS wrappers — HAL translates to SDK types) */
#define RSS_IVS_MAX_ROI 52

typedef struct {
    int p0_x, p0_y; /* top-left */
    int p1_x, p1_y; /* bottom-right */
} rss_rect_t;

typedef struct {
    int sense[RSS_IVS_MAX_ROI]; /* per-ROI sensitivity: 0-4 (normal), 0-8 (panoramic) */
    int skip_frame_count;
    int width;
    int height;
    rss_rect_t roi[RSS_IVS_MAX_ROI];
    int roi_count;
} rss_ivs_move_param_t;

typedef struct {
    int skip_frame_count;
    int sense; /* global sensitivity: 0-3 */
    int width;
    int height;
} rss_ivs_base_move_param_t;

typedef struct {
    int ret_roi[RSS_IVS_MAX_ROI]; /* 0 = no motion, 1 = motion detected */
} rss_ivs_move_result_t;

/* IVS person/object detection parameters and results */
#define RSS_IVS_MAX_DETECTIONS 20

typedef struct {
    int skip_frame_count;
    int width;
    int height;
    int sensitivity;     /* 0-5 (default 4) */
    int det_distance;    /* 0-4: 6m/8m/10m/11m/13m (default 2) */
    bool motion_trigger; /* enable motion-triggered detection */
} rss_ivs_persondet_param_t;

typedef struct {
    char model_path[256];
    int width;
    int height;
    int num_classes;
    float conf_threshold;
    float nms_threshold;
} rss_ivs_jzdl_param_t;

typedef struct {
    rss_rect_t box;   /* bounding box */
    float confidence; /* 0.0 - 1.0 */
    int class_id;     /* 0 = person, -1 = unclassified */
} rss_ivs_detection_t;

typedef struct {
    int count;
    rss_ivs_detection_t detections[RSS_IVS_MAX_DETECTIONS];
    int64_t timestamp;
} rss_ivs_detect_result_t;

/* Framesource channel configuration */
typedef struct {
    uint16_t width;
    uint16_t height;
    rss_pixfmt_t pixfmt;

    uint32_t fps_num;
    uint32_t fps_den;

    /* ISP-level crop (before scaling); zero to disable */
    struct {
        bool enable;
        int x, y;
        int w, h;
    } crop;

    /* Frame-level crop (after scaling); T23+ only */
    struct {
        bool enable;
        int x, y;
        int w, h;
    } fcrop;

    /* Scaler (for sub-streams at lower resolution than sensor) */
    struct {
        bool enable;
        int out_width;
        int out_height;
    } scaler;

    int nr_vbs;   /* video buffer blocks; 0 = SDK default */
    int chn_type; /* 0 = physical, 1 = extension */
} rss_fs_config_t;

/* Audio sample rate */
typedef enum {
    RSS_AUDIO_RATE_8000 = 8000,
    RSS_AUDIO_RATE_16000 = 16000,
    RSS_AUDIO_RATE_24000 = 24000,
    RSS_AUDIO_RATE_32000 = 32000,
    RSS_AUDIO_RATE_44100 = 44100,
    RSS_AUDIO_RATE_48000 = 48000,
} rss_audio_rate_t;

/* Audio input type */
typedef enum {
    RSS_AUDIO_INPUT_AMIC = 0, /* Analog microphone (IMP_AI_*) */
    RSS_AUDIO_INPUT_DMIC = 1, /* Digital microphone array (IMP_DMIC_*) */
} rss_audio_input_t;

/* Audio device and channel configuration */
typedef struct {
    rss_audio_rate_t sample_rate;
    int samples_per_frame;
    int chn_count;                /* 1=mono, 2=stereo */
    int frame_depth;              /* usrFrmDepth [2..50] */
    int ai_vol;                   /* [-30..120], 60=unity */
    int ai_gain;                  /* [0..31] */
    rss_audio_input_t input_type; /* AMIC or DMIC */
    int dmic_count;               /* DMIC: number of mics (1/2/4), 0=auto */
    int dmic_aec_id;              /* DMIC: which mic for AEC processing (0-3) */
} rss_audio_config_t;

/* ISP image tuning values; 128 = neutral */
typedef struct {
    uint8_t brightness;
    uint8_t contrast;
    uint8_t saturation;
    uint8_t sharpness;
    uint8_t hue; /* requires has_bcsh_hue */
} rss_image_attr_t;

/* OSD region type */
typedef enum {
    RSS_OSD_PIC = 0,      /* RGBA picture */
    RSS_OSD_COVER = 1,    /* solid color rectangle */
    RSS_OSD_PIC_RMEM = 2, /* picture from reserved memory (T31+) */
} rss_osd_type_t;

/* OSD region definition */
typedef struct {
    rss_osd_type_t type;

    int x;
    int y;
    int width;
    int height;

    /* PIC / PIC_RMEM: bitmap data */
    const uint8_t *bitmap_data; /* BGRA, width*height*4 bytes */
    rss_pixfmt_t bitmap_fmt;    /* must be RSS_PIXFMT_BGRA */

    /* COVER: fill color (BGRA) */
    uint32_t cover_color;

    /* Alpha blending */
    bool global_alpha_en;
    uint8_t fg_alpha;
    uint8_t bg_alpha;

    int layer; /* z-order */
} rss_osd_region_t;

/* Sensor VIN type */
typedef enum {
    RSS_SENSOR_VIN_MIPI_CSI0 = 0,
    RSS_SENSOR_VIN_MIPI_CSI1 = 1,
    RSS_SENSOR_VIN_DVP = 2,
} rss_sensor_vin_t;

/* Sensor MCLK source */
typedef enum {
    RSS_SENSOR_MCLK0 = 0,
    RSS_SENSOR_MCLK1 = 1,
    RSS_SENSOR_MCLK2 = 2,
} rss_sensor_mclk_t;

/* Sensor configuration */
typedef struct {
    char name[20];     /* sensor driver name (matches IMPI2CInfo.type) */
    uint16_t i2c_addr; /* 7-bit I2C address */
    int i2c_adapter;
    uint16_t sensor_id; /* 0 if unused */

    /* T40/T32/T41 only */
    rss_sensor_vin_t vin_type;
    rss_sensor_mclk_t mclk;
    int default_boot;

    /* GPIO pins; -1 = unused */
    int rst_gpio;
    int pwdn_gpio;
    int power_gpio;
} rss_sensor_config_t;

/* Maximum number of sensors supported (IMPVI_MAIN, SEC, THR) */
#define RSS_MAX_SENSORS 3

/* Multi-sensor configuration (passed to hal_init) */
typedef struct {
    int sensor_count; /* 1, 2, or 3 */
    rss_sensor_config_t sensors[RSS_MAX_SENSORS];

    /* T23 MIPI switch GPIO config (ignored on T32/T40/T41) */
    struct {
        bool enable;
        uint16_t switch_gpio;  /* GPIO controlling MIPI switch chip */
        uint16_t main_gstate;  /* GPIO state to select main sensor */
        uint16_t sec_gstate;   /* GPIO state to select secondary sensor */
        uint16_t switch_gpio2; /* second GPIO (triple camera only) */
        uint16_t thr_gstate[2];
    } mipi_switch;

    /* Image stitching mode (T23 only, 0 = disabled) */
    int stitch_mode;
} rss_multi_sensor_config_t;

/* Exposure info (for IR-cut control) */
typedef struct {
    uint32_t total_gain;
    uint32_t exposure_time; /* microseconds */
    uint32_t ae_luma;
    uint32_t ev;       /* EV from GetEVAttr (T20-T31) */
    uint16_t wb_rgain; /* AWB red gain from GetWB_Statis (T20-T31) */
    uint16_t wb_bgain; /* AWB blue gain from GetWB_Statis (T20-T31) */
} rss_exposure_t;

/* White balance mode (matches ISP_CORE_WB_MODE_* from libimp) */
typedef enum {
    RSS_WB_AUTO = 0,
    RSS_WB_MANUAL = 1,
    RSS_WB_DAYLIGHT = 2,
    RSS_WB_CLOUDY = 3,
    RSS_WB_INCANDESCENT = 4,
    RSS_WB_FLUORESCENT = 5,
    RSS_WB_TWILIGHT = 6,
    RSS_WB_SHADE = 7,
    RSS_WB_WARM_FLUORESCENT = 8,
    RSS_WB_CUSTOM = 9,
} rss_wb_mode_t;

/* White balance configuration */
typedef struct {
    rss_wb_mode_t mode;
    uint16_t r_gain;
    uint16_t g_gain;
    uint16_t b_gain;
} rss_wb_config_t;

/* Audio AGC parameters */
typedef struct {
    int target_level_dbfs;   /* [0..31] */
    int compression_gain_db; /* [0..90] */
} rss_agc_config_t;

/* Custom audio encoder callbacks */
typedef struct {
    char name[16];
    int max_frame_len;

    int (*open)(void *attr, void *encoder);
    int (*encode)(void *encoder, const int16_t *pcm, int pcm_len, uint8_t *out, int *out_len);
    int (*close)(void *encoder);
} rss_audio_encoder_t;

/* Noise suppression level */
typedef enum {
    RSS_NS_LOW = 0,
    RSS_NS_MODERATE = 1,
    RSS_NS_HIGH = 2,
    RSS_NS_VERYHIGH = 3,
} rss_ns_level_t;

/* Pipeline device ID */
typedef enum {
    RSS_DEV_FS = 0,
    RSS_DEV_ENC = 1,
    RSS_DEV_DEC = 2,
    RSS_DEV_IVS = 3,
    RSS_DEV_OSD = 4,
} rss_dev_id_t;

/* Pipeline endpoint for bind/unbind */
typedef struct {
    rss_dev_id_t device;
    int group;
    int output;
} rss_cell_t;

/* ISP running mode */
typedef enum {
    RSS_ISP_DAY = 0,
    RSS_ISP_NIGHT = 1,
} rss_isp_mode_t;

/* Anti-flicker mode */
typedef enum {
    RSS_ANTIFLICKER_OFF = 0,
    RSS_ANTIFLICKER_50HZ = 1,
    RSS_ANTIFLICKER_60HZ = 2,
} rss_antiflicker_t;

/* ================================================================
 * Capability Struct
 * ================================================================ */

typedef struct {
    /* System info */
    const char *soc_name;
    const char *sdk_version;
    int max_fs_channels;
    int max_enc_channels;
    int max_osd_groups;
    int max_osd_regions;

    /* Encoder capabilities */
    bool has_h265;
    bool has_rotation;
    bool has_i2d;
    bool has_bufshare;
    bool has_set_default_param;
    bool has_capped_rc;
    bool has_smart_rc;
    bool has_gop_attr;
    bool has_set_bitrate;
    bool has_stream_buf_size;
    bool has_encoder_pool;
    bool has_smartp_gop;
    bool has_rc_options;
    bool has_pskip;
    bool has_srd;
    bool has_max_pic_size;
    bool has_super_frame;
    bool has_color2grey;
    bool has_roi;
    bool has_map_roi;
    bool has_qp_bounds_per_frame;
    bool has_qpg_mode;
    bool has_qpg_ai;
    bool has_mbrc;
    bool has_enc_denoise;
    bool has_gdr;
    bool has_sei_userdata;
    bool has_h264_vui;
    bool has_h265_vui;
    bool has_h264_trans;
    bool has_h265_trans;
    bool has_enc_crop;
    bool has_eval_info;
    bool has_poll_module;
    bool has_resize_mode;
    bool has_jpeg_ql;
    bool has_jpeg_qp;

    /* ISP capabilities */
    bool has_multi_sensor;
    int max_sensors;           /* 1 for T20-T31, 3 for T23-1.3.0/T32/T40/T41 */
    bool has_t23_multicam_api; /* T23 1.3.0 IMP_ISP_MultiCamera_* functions */
    bool has_defog;
    bool has_dpc;
    bool has_drc;
    bool has_face_ae;
    bool has_bcsh_hue;
    bool has_sinter;
    bool has_temper;
    bool has_highlight_depress;
    bool has_backlight_comp;
    bool has_ae_comp;
    bool has_max_gain;
    bool has_switch_bin;
    bool has_gamma;
    bool has_gamma_attr;
    bool has_module_control;
    bool has_wdr;

    /* OSD capabilities */
    bool has_isp_osd;
    bool has_osd_mosaic;
    bool has_osd_group_callback;
    bool has_osd_region_invert;
    bool has_extended_osd_types;

    /* Audio capabilities */
    bool has_audio_process_lib;
    bool has_audio_aec_channel;
    bool has_alc_gain;
    bool has_agc_mode;
    bool has_digital_gain;
    bool has_howling_suppress;
    bool has_hpf_cutoff;

    /* System capabilities */
    bool uses_xburst2;
    bool uses_new_sdk;
    bool uses_impvi;
    int max_isp_osd_regions;
} rss_hal_caps_t;

/* ================================================================
 * Opaque Context
 * ================================================================ */

typedef struct rss_hal_ctx rss_hal_ctx_t;

/* ================================================================
 * Operations Vtable
 * ================================================================ */

typedef struct rss_hal_ops {

    /* --- System lifecycle --- */

    int (*init)(void *ctx, const rss_multi_sensor_config_t *multi_cfg);
    int (*deinit)(void *ctx);
    const rss_hal_caps_t *(*get_caps)(void *ctx);
    int (*bind)(void *ctx, const rss_cell_t *src, const rss_cell_t *dst);
    int (*unbind)(void *ctx, const rss_cell_t *src, const rss_cell_t *dst);

    /* --- System utilities --- */

    int (*sys_get_version)(void *ctx, char *buf, int len);
    int (*sys_get_cpu_info)(void *ctx, char *buf, int len);
    int (*sys_get_timestamp)(void *ctx, int64_t *ts);
    int (*sys_rebase_timestamp)(void *ctx, int64_t base);
    int (*sys_read_reg32)(void *ctx, uint32_t addr, uint32_t *val);
    int (*sys_write_reg32)(void *ctx, uint32_t addr, uint32_t val);
    int (*sys_get_bind_by_dest)(void *ctx, rss_cell_t *dst, rss_cell_t *src);

    /* --- Framesource --- */

    int (*fs_create_channel)(void *ctx, int chn, const rss_fs_config_t *cfg);
    int (*fs_set_channel_attr)(void *ctx, int chn, const rss_fs_config_t *cfg);
    int (*fs_destroy_channel)(void *ctx, int chn);
    int (*fs_enable_channel)(void *ctx, int chn);
    int (*fs_disable_channel)(void *ctx, int chn);
    int (*fs_set_rotation)(void *ctx, int chn, int degrees);
    int (*fs_set_fifo)(void *ctx, int chn, int depth);
    int (*fs_get_frame)(void *ctx, int chn, void **frame_data, rss_frame_info_t *info);
    int (*fs_release_frame)(void *ctx, int chn, void *frame_data);
    int (*fs_snap_frame)(void *ctx, int chn, void **frame_data, rss_frame_info_t *info);
    int (*fs_set_frame_depth)(void *ctx, int chn, int depth);
    int (*fs_get_frame_depth)(void *ctx, int chn, int *depth);
    int (*fs_get_fifo)(void *ctx, int chn, int *depth);
    int (*fs_set_delay)(void *ctx, int chn, int delay_ms);
    int (*fs_get_delay)(void *ctx, int chn, int *delay_ms);
    int (*fs_set_max_delay)(void *ctx, int chn, int max_delay_ms);
    int (*fs_get_max_delay)(void *ctx, int chn, int *max_delay_ms);
    int (*fs_set_pool)(void *ctx, int chn, int pool_id);
    int (*fs_get_pool)(void *ctx, int chn, int *pool_id);
    int (*fs_get_timed_frame)(void *ctx, int chn, void *framets, int block, void *framedata,
                              void *frame);
    int (*fs_set_frame_offset)(void *ctx, int chn, int offset);
    int (*fs_chn_stat_query)(void *ctx, int chn, void *stat);
    int (*fs_enable_chn_undistort)(void *ctx, int chn);
    int (*fs_disable_chn_undistort)(void *ctx, int chn);

    /* --- Encoder --- */

    int (*enc_create_group)(void *ctx, int grp);
    int (*enc_destroy_group)(void *ctx, int grp);
    int (*enc_create_channel)(void *ctx, int chn, const rss_video_config_t *cfg);
    int (*enc_destroy_channel)(void *ctx, int chn);
    int (*enc_register_channel)(void *ctx, int grp, int chn);
    int (*enc_unregister_channel)(void *ctx, int chn);
    int (*enc_start)(void *ctx, int chn);
    int (*enc_stop)(void *ctx, int chn);
    int (*enc_poll)(void *ctx, int chn, uint32_t timeout_ms);
    int (*enc_get_frame)(void *ctx, int chn, rss_frame_t *frame);
    int (*enc_release_frame)(void *ctx, int chn, rss_frame_t *frame);
    int (*enc_request_idr)(void *ctx, int chn);
    int (*enc_set_rc_mode)(void *ctx, int chn, rss_rc_mode_t mode, uint32_t bitrate);
    int (*enc_set_bitrate)(void *ctx, int chn, uint32_t bitrate);
    int (*enc_set_gop)(void *ctx, int chn, uint32_t gop_length);
    int (*enc_set_fps)(void *ctx, int chn, uint32_t fps_num, uint32_t fps_den);
    int (*enc_set_bufshare)(void *ctx, int src_chn, int dst_chn);
    int (*enc_get_channel_attr)(void *ctx, int chn, rss_video_config_t *cfg);
    int (*enc_get_fps)(void *ctx, int chn, uint32_t *fps_num, uint32_t *fps_den);
    int (*enc_get_gop_attr)(void *ctx, int chn, uint32_t *gop_length);
    int (*enc_set_gop_attr)(void *ctx, int chn, uint32_t gop_length);
    int (*enc_get_avg_bitrate)(void *ctx, int chn, uint32_t *bitrate);
    int (*enc_flush_stream)(void *ctx, int chn);
    int (*enc_query)(void *ctx, int chn, bool *busy);
    int (*enc_get_fd)(void *ctx, int chn);
    int (*enc_set_qp)(void *ctx, int chn, int qp);
    int (*enc_set_qp_bounds)(void *ctx, int chn, int min_qp, int max_qp);
    int (*enc_set_qp_ip_delta)(void *ctx, int chn, int delta);
    int (*enc_set_qp_pb_delta)(void *ctx, int chn, int delta);
    int (*enc_set_max_psnr)(void *ctx, int chn, int psnr);
    int (*enc_set_stream_buf_size)(void *ctx, int chn, uint32_t size);
    int (*enc_get_stream_buf_size)(void *ctx, int chn, uint32_t *size);
    int (*enc_get_chn_gop_attr)(void *ctx, int chn, void *gop_attr);
    int (*enc_set_chn_gop_attr)(void *ctx, int chn, const void *gop_attr);
    int (*enc_get_chn_enc_type)(void *ctx, int chn, void *enc_type);
    int (*enc_get_chn_ave_bitrate)(void *ctx, int chn, void *stream, int frames, double *br);
    int (*enc_set_chn_entropy_mode)(void *ctx, int chn, int mode);
    int (*enc_get_max_stream_cnt)(void *ctx, int chn, int *cnt);
    int (*enc_set_max_stream_cnt)(void *ctx, int chn, int cnt);
    int (*enc_set_pool)(void *ctx, int chn, int pool_id);
    int (*enc_get_pool)(void *ctx, int chn);
    int (*enc_get_rmem_info)(void *ctx, uintptr_t *virt_base, uint32_t *size,
                             uint32_t *mmap_offset);
    int (*enc_inject_stream_shm)(void *ctx, int chn, void *shm_addr, uint32_t shm_size);

    /* Encoder: Phase 1 — Bandwidth reduction */
    int (*enc_set_gop_mode)(void *ctx, int chn, rss_gop_mode_t mode);
    int (*enc_get_gop_mode)(void *ctx, int chn, rss_gop_mode_t *mode);
    int (*enc_set_rc_options)(void *ctx, int chn, uint32_t options);
    int (*enc_get_rc_options)(void *ctx, int chn, uint32_t *options);
    int (*enc_set_max_same_scene_cnt)(void *ctx, int chn, uint32_t count);
    int (*enc_get_max_same_scene_cnt)(void *ctx, int chn, uint32_t *count);
    int (*enc_set_pskip)(void *ctx, int chn, const rss_pskip_cfg_t *cfg);
    int (*enc_get_pskip)(void *ctx, int chn, rss_pskip_cfg_t *cfg);
    int (*enc_request_pskip)(void *ctx, int chn);
    int (*enc_set_srd)(void *ctx, int chn, const rss_srd_cfg_t *cfg);
    int (*enc_get_srd)(void *ctx, int chn, rss_srd_cfg_t *cfg);
    int (*enc_set_max_pic_size)(void *ctx, int chn, uint32_t max_i_kbits, uint32_t max_p_kbits);
    int (*enc_set_super_frame)(void *ctx, int chn, const rss_super_frame_cfg_t *cfg);
    int (*enc_get_super_frame)(void *ctx, int chn, rss_super_frame_cfg_t *cfg);
    int (*enc_set_color2grey)(void *ctx, int chn, bool enable);
    int (*enc_get_color2grey)(void *ctx, int chn, bool *enable);

    /* Encoder: Phase 2 — Quality improvement */
    int (*enc_set_roi)(void *ctx, int chn, const rss_enc_roi_t *roi);
    int (*enc_get_roi)(void *ctx, int chn, uint32_t index, rss_enc_roi_t *roi);
    int (*enc_set_map_roi)(void *ctx, int chn, const uint8_t *map, uint32_t map_size, int type);
    int (*enc_set_qp_bounds_per_frame)(void *ctx, int chn, int min_i, int max_i, int min_p,
                                       int max_p);
    int (*enc_set_qpg_mode)(void *ctx, int chn, int mode);
    int (*enc_get_qpg_mode)(void *ctx, int chn, int *mode);
    int (*enc_set_qpg_ai)(void *ctx, int chn, const uint8_t *map, uint32_t w, uint32_t h, int mode,
                          int mark_level);
    int (*enc_set_mbrc)(void *ctx, int chn, bool enable);
    int (*enc_get_mbrc)(void *ctx, int chn, bool *enable);
    int (*enc_set_denoise)(void *ctx, int chn, const rss_enc_denoise_cfg_t *cfg);
    int (*enc_get_denoise)(void *ctx, int chn, rss_enc_denoise_cfg_t *cfg);

    /* Encoder: Phase 3 — Error recovery */
    int (*enc_set_gdr)(void *ctx, int chn, const rss_enc_gdr_cfg_t *cfg);
    int (*enc_get_gdr)(void *ctx, int chn, rss_enc_gdr_cfg_t *cfg);
    int (*enc_request_gdr)(void *ctx, int chn, int gdr_frames);
    int (*enc_insert_userdata)(void *ctx, int chn, const void *data, uint32_t len);

    /* Encoder: Phase 4 — Codec compliance */
    int (*enc_set_h264_vui)(void *ctx, int chn, const void *vui);
    int (*enc_get_h264_vui)(void *ctx, int chn, void *vui);
    int (*enc_set_h265_vui)(void *ctx, int chn, const void *vui);
    int (*enc_get_h265_vui)(void *ctx, int chn, void *vui);
    int (*enc_set_h264_trans)(void *ctx, int chn, const rss_enc_h264_trans_t *cfg);
    int (*enc_get_h264_trans)(void *ctx, int chn, rss_enc_h264_trans_t *cfg);
    int (*enc_set_h265_trans)(void *ctx, int chn, const rss_enc_h265_trans_t *cfg);
    int (*enc_get_h265_trans)(void *ctx, int chn, rss_enc_h265_trans_t *cfg);

    /* Encoder: Phase 5 — Operational */
    int (*enc_set_crop)(void *ctx, int chn, const rss_enc_crop_cfg_t *cfg);
    int (*enc_get_crop)(void *ctx, int chn, rss_enc_crop_cfg_t *cfg);
    int (*enc_get_eval_info)(void *ctx, int chn, void *info);
    int (*enc_poll_module_stream)(void *ctx, uint32_t *chn_bitmap, uint32_t timeout_ms);
    int (*enc_set_resize_mode)(void *ctx, int chn, int enable);

    /* Encoder: Phase 6 — JPEG */
    int (*enc_set_jpeg_ql)(void *ctx, int chn, const rss_enc_jpeg_ql_t *ql);
    int (*enc_get_jpeg_ql)(void *ctx, int chn, rss_enc_jpeg_ql_t *ql);
    int (*enc_set_jpeg_qp)(void *ctx, int chn, int qp);
    int (*enc_get_jpeg_qp)(void *ctx, int chn, int *qp);

    /* --- ISP tuning --- */

    int (*isp_set_brightness)(void *ctx, int val);
    int (*isp_set_contrast)(void *ctx, int val);
    int (*isp_set_saturation)(void *ctx, int val);
    int (*isp_set_sharpness)(void *ctx, int val);
    int (*isp_set_hue)(void *ctx, int val);
    int (*isp_set_hflip)(void *ctx, int enable);
    int (*isp_set_vflip)(void *ctx, int enable);
    int (*isp_set_running_mode)(void *ctx, rss_isp_mode_t mode);
    int (*isp_set_sensor_fps)(void *ctx, uint32_t fps_num, uint32_t fps_den);
    int (*isp_set_antiflicker)(void *ctx, rss_antiflicker_t mode);
    int (*isp_set_wb)(void *ctx, const rss_wb_config_t *wb_cfg);
    int (*isp_get_exposure)(void *ctx, rss_exposure_t *exposure);

    /* Optional ISP tuning (return -ENOTSUP if unavailable) */
    int (*isp_set_sinter_strength)(void *ctx, int val);
    int (*isp_set_temper_strength)(void *ctx, int val);
    int (*isp_set_defog)(void *ctx, int enable);
    int (*isp_set_dpc_strength)(void *ctx, int val);
    int (*isp_set_drc_strength)(void *ctx, int val);
    int (*isp_set_ae_comp)(void *ctx, int val);
    int (*isp_set_max_again)(void *ctx, int gain);
    int (*isp_set_max_dgain)(void *ctx, int gain);
    int (*isp_set_highlight_depress)(void *ctx, int val);

    /* ISP getters (mirrors of existing setters) */
    int (*isp_get_brightness)(void *ctx, uint8_t *val);
    int (*isp_get_contrast)(void *ctx, uint8_t *val);
    int (*isp_get_saturation)(void *ctx, uint8_t *val);
    int (*isp_get_sharpness)(void *ctx, uint8_t *val);
    int (*isp_get_hue)(void *ctx, uint8_t *val);
    int (*isp_get_hvflip)(void *ctx, int *hflip, int *vflip);
    int (*isp_get_running_mode)(void *ctx, rss_isp_mode_t *mode);
    int (*isp_get_sensor_fps)(void *ctx, uint32_t *fps_num, uint32_t *fps_den);
    int (*isp_get_antiflicker)(void *ctx, rss_antiflicker_t *mode);
    int (*isp_get_wb)(void *ctx, rss_wb_config_t *wb_cfg);
    int (*isp_get_max_again)(void *ctx, uint32_t *gain);
    int (*isp_get_max_dgain)(void *ctx, uint32_t *gain);
    int (*isp_get_sensor_attr)(void *ctx, uint32_t *width, uint32_t *height);
    int (*isp_get_ae_comp)(void *ctx, int *val);
    int (*isp_get_module_control)(void *ctx, uint32_t *modules);
    int (*isp_get_sinter_strength)(void *ctx, uint8_t *val);
    int (*isp_get_temper_strength)(void *ctx, uint8_t *val);
    int (*isp_get_defog_strength)(void *ctx, uint8_t *val);
    int (*isp_get_dpc_strength)(void *ctx, uint8_t *val);
    int (*isp_get_drc_strength)(void *ctx, uint8_t *val);
    int (*isp_get_highlight_depress)(void *ctx, uint8_t *val);
    int (*isp_get_backlight_comp)(void *ctx, uint8_t *val);

    /* ISP AE advanced */
    int (*isp_set_ae_weight)(void *ctx, const uint8_t weight[15][15]);
    int (*isp_get_ae_weight)(void *ctx, uint8_t weight[15][15]);
    int (*isp_get_ae_zone)(void *ctx, uint32_t zone[15][15]);
    int (*isp_set_ae_roi)(void *ctx, const uint8_t roi[15][15]);
    int (*isp_get_ae_roi)(void *ctx, uint8_t roi[15][15]);
    int (*isp_set_ae_hist)(void *ctx, const uint8_t thresholds[4]);
    int (*isp_get_ae_hist)(void *ctx, uint8_t thresholds[4], uint16_t bins[5]);
    int (*isp_get_ae_hist_origin)(void *ctx, uint32_t bins[256]);
    int (*isp_set_ae_it_max)(void *ctx, uint32_t it_max);
    int (*isp_get_ae_it_max)(void *ctx, uint32_t *it_max);
    int (*isp_set_ae_min)(void *ctx, int min_it, int min_again);
    int (*isp_get_ae_min)(void *ctx, int *min_it, int *min_again);

    /* ISP AWB advanced */
    int (*isp_set_awb_weight)(void *ctx, const uint8_t weight[15][15]);
    int (*isp_get_awb_weight)(void *ctx, uint8_t weight[15][15]);
    int (*isp_get_awb_zone)(void *ctx, uint8_t zone_r[225], uint8_t zone_g[225],
                            uint8_t zone_b[225]);
    int (*isp_get_awb_ct)(void *ctx, uint32_t *color_temp);
    int (*isp_get_awb_rgb_coefft)(void *ctx, uint16_t *rgain, uint16_t *ggain, uint16_t *bgain);
    int (*isp_get_awb_hist)(void *ctx, void *hist_data);

    /* ISP gamma / CCM / WDR */
    int (*isp_set_gamma)(void *ctx, const uint16_t gamma[129]);
    int (*isp_get_gamma)(void *ctx, uint16_t gamma[129]);
    int (*isp_set_ccm)(void *ctx, const void *ccm_attr);
    int (*isp_get_ccm)(void *ctx, void *ccm_attr);
    int (*isp_set_wdr_mode)(void *ctx, int mode);
    int (*isp_get_wdr_mode)(void *ctx, int *mode);
    int (*isp_wdr_enable)(void *ctx, int enable);
    int (*isp_wdr_get_enable)(void *ctx, int *enabled);
    int (*isp_set_bypass)(void *ctx, int enable);
    int (*isp_set_module_control)(void *ctx, uint32_t modules);

    /* ISP misc */
    int (*isp_set_default_bin_path)(void *ctx, const char *path);
    int (*isp_get_default_bin_path)(void *ctx, char *path, int path_len);
    int (*isp_set_frame_drop)(void *ctx, int drop);
    int (*isp_get_frame_drop)(void *ctx, int *drop);
    int (*isp_set_sensor_register)(void *ctx, uint32_t reg, uint32_t val);
    int (*isp_get_sensor_register)(void *ctx, uint32_t reg, uint32_t *val);
    int (*isp_set_auto_zoom)(void *ctx, const void *zoom_attr);
    int (*isp_set_video_drop)(void *ctx, void (*callback)(void));
    int (*isp_set_mask)(void *ctx, const void *mask_attr);
    int (*isp_get_mask)(void *ctx, void *mask_attr);

    /* ISP advanced AE/AWB/misc (T23+T31 and select SoCs) */
    int (*isp_set_expr)(void *ctx, const void *expr_attr);
    int (*isp_get_ae_attr)(void *ctx, void *ae_attr);
    int (*isp_set_ae_attr)(void *ctx, const void *ae_attr);
    int (*isp_get_ae_state)(void *ctx, void *ae_state);
    int (*isp_get_ae_target_list)(void *ctx, void *target_list);
    int (*isp_set_ae_target_list)(void *ctx, const void *target_list);
    int (*isp_set_ae_freeze)(void *ctx, int enable);
    int (*isp_get_af_zone)(void *ctx, void *af_zone);
    int (*isp_get_awb_clust)(void *ctx, void *clust);
    int (*isp_set_awb_clust)(void *ctx, const void *clust);
    int (*isp_get_awb_ct_attr)(void *ctx, void *ct_attr);
    int (*isp_set_awb_ct_attr)(void *ctx, const void *ct_attr);
    int (*isp_get_awb_ct_trend)(void *ctx, void *trend);
    int (*isp_set_awb_ct_trend)(void *ctx, const void *trend);
    int (*isp_set_backlight_comp)(void *ctx, int strength);
    int (*isp_get_defog_strength_adv)(void *ctx, void *defog_attr);
    int (*isp_set_defog_strength_adv)(void *ctx, const void *defog_attr);
    int (*isp_get_front_crop)(void *ctx, void *crop_attr);
    int (*isp_set_front_crop)(void *ctx, const void *crop_attr);
    int (*isp_get_blc_attr)(void *ctx, void *blc_attr);
    int (*isp_get_csc_attr)(void *ctx, void *csc_attr);
    int (*isp_set_csc_attr)(void *ctx, const void *csc_attr);
    int (*isp_set_custom_mode)(void *ctx, int mode);
    int (*isp_get_custom_mode)(void *ctx, int *mode);
    int (*isp_enable_drc)(void *ctx, int enable);

    /* ISP AF / move-state (T20-T31) */
    int (*isp_get_af_hist)(void *ctx, void *af_hist);
    int (*isp_set_af_hist)(void *ctx, const void *af_hist);
    int (*isp_get_af_metrics)(void *ctx, void *metrics);
    int (*isp_enable_movestate)(void *ctx);
    int (*isp_disable_movestate)(void *ctx);
    int (*isp_set_shading)(void *ctx, const void *shading_attr);
    int (*isp_wait_frame)(void *ctx, int timeout_ms);

    /* ISP AF weight (T21+T23+T31+T32+T40+T41) */
    int (*isp_get_af_weight)(void *ctx, void *af_weight);
    int (*isp_set_af_weight)(void *ctx, const void *af_weight);

    /* ISP WB statistics (T20-T31) */
    int (*isp_get_wb_statis)(void *ctx, void *wb_statis);
    int (*isp_set_awb_hist_adv)(void *ctx, const void *awb_hist);

    /* ISP WB GOL statistics (T21+T23+T31) */
    int (*isp_get_wb_gol_statis)(void *ctx, void *gol_statis);

    /* ISP WDR output mode (T31 only) */
    int (*isp_set_wdr_output_mode)(void *ctx, int mode);
    int (*isp_get_wdr_output_mode)(void *ctx, int *mode);

    /* ISP scaler level (T23+T31+T32+T41) */
    int (*isp_set_scaler_lv)(void *ctx, int chn, int level);

    /* --- Multi-sensor ISP tuning (sensor_idx: 0=main, 1=sec, 2=thr) --- */

    int (*isp_set_brightness_n)(void *ctx, int sensor_idx, int val);
    int (*isp_set_contrast_n)(void *ctx, int sensor_idx, int val);
    int (*isp_set_saturation_n)(void *ctx, int sensor_idx, int val);
    int (*isp_set_sharpness_n)(void *ctx, int sensor_idx, int val);
    int (*isp_set_hue_n)(void *ctx, int sensor_idx, int val);
    int (*isp_set_hflip_n)(void *ctx, int sensor_idx, int enable);
    int (*isp_set_vflip_n)(void *ctx, int sensor_idx, int enable);
    int (*isp_set_running_mode_n)(void *ctx, int sensor_idx, rss_isp_mode_t mode);
    int (*isp_set_sensor_fps_n)(void *ctx, int sensor_idx, uint32_t fps_num, uint32_t fps_den);
    int (*isp_set_antiflicker_n)(void *ctx, int sensor_idx, rss_antiflicker_t mode);
    int (*isp_set_sinter_strength_n)(void *ctx, int sensor_idx, int val);
    int (*isp_set_temper_strength_n)(void *ctx, int sensor_idx, int val);
    int (*isp_set_ae_comp_n)(void *ctx, int sensor_idx, int val);
    int (*isp_set_max_again_n)(void *ctx, int sensor_idx, int gain);
    int (*isp_set_max_dgain_n)(void *ctx, int sensor_idx, int gain);
    int (*isp_get_exposure_n)(void *ctx, int sensor_idx, rss_exposure_t *exposure);
    int (*isp_set_custom_mode_n)(void *ctx, int sensor_idx, int mode);
    int (*isp_set_ae_freeze_n)(void *ctx, int sensor_idx, int enable);

    /* --- Audio --- */

    int (*audio_init)(void *ctx, const rss_audio_config_t *cfg);
    int (*audio_deinit)(void *ctx);
    int (*audio_set_volume)(void *ctx, int dev, int chn, int vol);
    int (*audio_set_gain)(void *ctx, int dev, int chn, int gain);
    int (*audio_enable_ns)(void *ctx, rss_ns_level_t level);
    int (*audio_disable_ns)(void *ctx);
    int (*audio_enable_hpf)(void *ctx);
    int (*audio_disable_hpf)(void *ctx);
    int (*audio_enable_agc)(void *ctx, const rss_agc_config_t *cfg);
    int (*audio_disable_agc)(void *ctx);
    int (*audio_read_frame)(void *ctx, int dev, int chn, rss_audio_frame_t *frame, bool block);
    int (*audio_release_frame)(void *ctx, int dev, int chn, rss_audio_frame_t *frame);
    int (*audio_register_encoder)(void *ctx, const rss_audio_encoder_t *enc, int *handle);
    int (*audio_unregister_encoder)(void *ctx, int handle);

    /* AI additional */
    int (*audio_set_aec_profile_path)(void *ctx, const char *dir);
    int (*audio_enable_aec)(void *ctx, int ai_dev, int ai_chn, int ao_dev, int ao_chn);
    int (*audio_disable_aec)(void *ctx);
    int (*audio_get_volume)(void *ctx, int dev, int chn, int *vol);
    int (*audio_get_gain)(void *ctx, int dev, int chn, int *gain);
    int (*audio_set_mute)(void *ctx, int dev, int chn, int mute);
    int (*audio_set_alc_gain)(void *ctx, int dev, int chn, int gain);
    int (*audio_get_alc_gain)(void *ctx, int dev, int chn, int *gain);
    int (*audio_set_agc_mode)(void *ctx, int mode);
    int (*audio_set_hpf_co_freq)(void *ctx, int freq);
    int (*audio_enable_aec_ref_frame)(void *ctx, int ai_dev, int ai_chn, int ao_dev, int ao_chn);
    int (*audio_disable_aec_ref_frame)(void *ctx, int ai_dev, int ai_chn);
    int (*audio_get_chn_param)(void *ctx, int dev, int chn, void *param);
    int (*audio_get_frame_and_ref)(void *ctx, int dev, int chn, void *frame, void *ref, int block);

    /* Audio encoding pipeline (AENC) */
    int (*aenc_create_channel)(void *ctx, int chn, int codec_type);
    int (*aenc_destroy_channel)(void *ctx, int chn);
    int (*aenc_send_frame)(void *ctx, int chn, rss_audio_frame_t *frame);
    int (*aenc_poll_stream)(void *ctx, int chn, uint32_t timeout_ms);
    int (*aenc_get_stream)(void *ctx, int chn, rss_audio_frame_t *stream);
    int (*aenc_release_stream)(void *ctx, int chn, rss_audio_frame_t *stream);

    /* Audio decoding pipeline (ADEC) — for backchannel / two-way audio */
    int (*adec_register_decoder)(void *ctx, int *handle, void *decoder);
    int (*adec_unregister_decoder)(void *ctx, int handle);
    int (*adec_create_channel)(void *ctx, int chn, int codec_type);
    int (*adec_destroy_channel)(void *ctx, int chn);
    int (*adec_send_stream)(void *ctx, int chn, const uint8_t *data, uint32_t len,
                            int64_t timestamp);
    int (*adec_clear_buf)(void *ctx, int chn);
    int (*adec_poll_stream)(void *ctx, int chn, uint32_t timeout_ms);
    int (*adec_get_stream)(void *ctx, int chn, rss_audio_frame_t *stream);
    int (*adec_release_stream)(void *ctx, int chn, rss_audio_frame_t *stream);

    /* Audio output (AO) — speaker playback for backchannel */
    int (*ao_init)(void *ctx, const rss_audio_config_t *cfg);
    int (*ao_deinit)(void *ctx);
    int (*ao_set_volume)(void *ctx, int vol);
    int (*ao_set_gain)(void *ctx, int gain);
    int (*ao_send_frame)(void *ctx, const int16_t *data, uint32_t len, bool block);
    int (*ao_pause)(void *ctx);
    int (*ao_resume)(void *ctx);
    int (*ao_clear_buf)(void *ctx);
    int (*ao_flush_buf)(void *ctx);
    int (*ao_get_volume)(void *ctx, int *vol);
    int (*ao_get_gain)(void *ctx, int *gain);
    int (*ao_set_mute)(void *ctx, int mute);
    int (*ao_enable_hpf)(void *ctx);
    int (*ao_disable_hpf)(void *ctx);
    int (*ao_enable_agc)(void *ctx);
    int (*ao_disable_agc)(void *ctx);
    int (*ao_set_hpf_co_freq)(void *ctx, int freq);
    int (*ao_query_chn_stat)(void *ctx, int dev, int chn, void *stat);
    int (*ao_soft_mute)(void *ctx, int dev, int chn);
    int (*ao_soft_unmute)(void *ctx, int dev, int chn);
    int (*ao_cache_switch)(void *ctx, int dev, int chn, int enable);

    /* --- OSD --- */

    int (*osd_set_pool_size)(void *ctx, uint32_t bytes);
    int (*osd_create_group)(void *ctx, int grp);
    int (*osd_destroy_group)(void *ctx, int grp);
    int (*osd_create_region)(void *ctx, int *handle, const rss_osd_region_t *attr);
    int (*osd_destroy_region)(void *ctx, int handle);
    int (*osd_register_region)(void *ctx, int handle, int grp);
    int (*osd_unregister_region)(void *ctx, int handle, int grp);
    int (*osd_set_region_attr)(void *ctx, int handle, const rss_osd_region_t *attr);
    int (*osd_update_region_data)(void *ctx, int handle, const uint8_t *data);
    int (*osd_show_region)(void *ctx, int handle, int grp, int show, int layer);
    int (*osd_get_region_attr)(void *ctx, int handle, rss_osd_region_t *attr);
    int (*osd_get_group_region_attr)(void *ctx, int handle, int grp, rss_osd_region_t *attr);
    int (*osd_show)(void *ctx, int handle, int grp, bool show);
    int (*osd_start)(void *ctx, int grp);
    int (*osd_stop)(void *ctx, int grp);
    int (*osd_set_region_attr_with_timestamp)(void *ctx, int handle, const rss_osd_region_t *attr,
                                              uint64_t timestamp);
    int (*osd_attach_to_group)(void *ctx, int handle, int grp);

    /* --- GPIO / IR-cut --- */

    int (*gpio_set)(void *ctx, int pin, int value);
    int (*gpio_get)(void *ctx, int pin, int *value);
    int (*ircut_set)(void *ctx, int state);

    /* --- IVS (Intelligent Video Surveillance / Motion Detection) --- */

    int (*ivs_create_group)(void *ctx, int grp);
    int (*ivs_destroy_group)(void *ctx, int grp);
    int (*ivs_create_channel)(void *ctx, int chn, void *algo_handle);
    int (*ivs_destroy_channel)(void *ctx, int chn);
    int (*ivs_register_channel)(void *ctx, int grp, int chn);
    int (*ivs_unregister_channel)(void *ctx, int chn);
    int (*ivs_start)(void *ctx, int chn);
    int (*ivs_stop)(void *ctx, int chn);
    int (*ivs_poll_result)(void *ctx, int chn, uint32_t timeout_ms);
    int (*ivs_get_result)(void *ctx, int chn, void **result);
    int (*ivs_release_result)(void *ctx, int chn, void *result);
    int (*ivs_get_param)(void *ctx, int chn, void *param);
    int (*ivs_set_param)(void *ctx, int chn, void *param);
    int (*ivs_release_data)(void *ctx, int chn, void *data);
    void *(*ivs_create_move_interface)(void *ctx, void *param);
    int (*ivs_destroy_move_interface)(void *ctx, void *handle);
    void *(*ivs_create_base_move_interface)(void *ctx, void *param);
    int (*ivs_destroy_base_move_interface)(void *ctx, void *handle);
    void *(*ivs_create_persondet_interface)(void *ctx, void *param);
    int (*ivs_destroy_persondet_interface)(void *ctx, void *handle);
    void *(*ivs_create_jzdl_interface)(void *ctx, void *param); /* unused — kept for ABI */
    int (*ivs_destroy_jzdl_interface)(void *ctx, void *handle); /* unused — kept for ABI */

    /* --- DMIC (Digital Microphone — T30/T31/T32/T40/T41) --- */

    int (*dmic_init)(void *ctx, const rss_audio_config_t *cfg);
    int (*dmic_deinit)(void *ctx);
    int (*dmic_set_volume)(void *ctx, int vol);
    int (*dmic_get_volume)(void *ctx, int *vol);
    int (*dmic_set_gain)(void *ctx, int gain);
    int (*dmic_get_gain)(void *ctx, int *gain);
    int (*dmic_set_chn_param)(void *ctx, int chn, int frames_per_buf);
    int (*dmic_get_chn_param)(void *ctx, int chn, int *frames_per_buf);
    int (*dmic_read_frame)(void *ctx, rss_audio_frame_t *frame, bool block);
    int (*dmic_release_frame)(void *ctx, rss_audio_frame_t *frame);
    int (*dmic_poll_frame)(void *ctx, uint32_t timeout_ms);
    int (*dmic_enable_aec)(void *ctx, int dev, int chn, int ao_dev, int ao_chn);
    int (*dmic_disable_aec)(void *ctx, int dev, int chn);
    int (*dmic_enable_aec_ref_frame)(void *ctx, int dev, int chn, int ao_dev, int ao_chn);
    int (*dmic_disable_aec_ref_frame)(void *ctx, int dev, int chn, int ao_dev, int ao_chn);
    int (*dmic_get_pub_attr)(void *ctx, int dev, void *attr);
    int (*dmic_get_frame_and_ref)(void *ctx, int dev, int chn, void *frame, void *ref, int block);

    /* --- ISP OSD (T23/T32/T40/T41 — hardware overlay in ISP pipeline) ---
     *
     * Uses IMP_ISP_Tuning_*OsdRgn* API. Not in the bind chain — overlay
     * happens inside ISP hardware before framesource. Works with IVDC.
     * sensornum = sensor index (0, 1, 2). */

    int (*isp_osd_set_pool_size)(void *ctx, int size);
    int (*isp_osd_create_region)(void *ctx, int sensornum, int *handle_out);
    int (*isp_osd_destroy_region)(void *ctx, int sensornum, int handle);
    int (*isp_osd_set_region_attr)(void *ctx, int sensornum, int handle, int chx,
                                   const rss_osd_region_t *attr);
    int (*isp_osd_show_region)(void *ctx, int sensornum, int handle, int show);
    int (*isp_osd_set_mask)(void *ctx, int sensornum, int chx, int pinum, int enable, int x, int y,
                            int w, int h, uint32_t color);

    /* --- Memory Management --- */

    void *(*mem_alloc)(void *ctx, uint32_t size, const char *name);
    void (*mem_free)(void *ctx, void *ptr);
    int (*mem_flush_cache)(void *ctx, void *ptr, uint32_t size);
    void *(*mem_phys_to_virt)(void *ctx, uint32_t phys_addr);
    uint32_t (*mem_virt_to_phys)(void *ctx, void *virt_addr);
    void *(*mem_pool_alloc)(void *ctx, uint32_t pool_id, uint32_t size);
    void (*mem_pool_free)(void *ctx, void *ptr);
    int (*mem_pool_flush_cache)(void *ctx, void *ptr, uint32_t size);
    void *(*mem_pool_phys_to_virt)(void *ctx, uint32_t phys_addr);
    uint32_t (*mem_pool_virt_to_phys)(void *ctx, void *virt_addr);

} rss_hal_ops_t;

/* ================================================================
 * Factory Functions
 * ================================================================ */

rss_hal_ctx_t *rss_hal_create(void);
void rss_hal_destroy(rss_hal_ctx_t *ctx);
const rss_hal_ops_t *rss_hal_get_ops(rss_hal_ctx_t *ctx);

/* Single-sensor init convenience wrapper (backward compat) */
static inline int rss_hal_init_single(const rss_hal_ops_t *ops, void *ctx,
                                      const rss_sensor_config_t *sensor)
{
    rss_multi_sensor_config_t m = {0};
    m.sensor_count = 1;
    m.sensors[0] = *sensor;
    return ops->init ? ops->init(ctx, &m) : RSS_ERR_NOTSUP;
}

/* ================================================================
 * Convenience Macro
 * ================================================================ */

/*
 * RSS_HAL_CALL -- invoke a vtable function with NULL guard.
 *
 * Returns RSS_ERR_NOTSUP if the function pointer is NULL.
 */
#define RSS_HAL_CALL(ops, fn, ctx, ...)                                                            \
    ((ops)->fn ? (ops)->fn((ctx), ##__VA_ARGS__) : RSS_ERR_NOTSUP)

/* System info (called once at startup, no vtable needed) */
int rss_hal_get_imp_version(char *buf, int size);
int rss_hal_get_sysutils_version(char *buf, int size);
const char *rss_hal_get_cpu_info(void);
const char *rss_hal_get_platform_name(void);
void rss_hal_check_platform(const char *name);

/* ================================================================
 * HAL Logging Callback
 *
 * By default, HAL logs to stderr via fprintf. Daemons can redirect
 * HAL log output to their own logger (e.g., syslog) by setting a
 * callback at init time. The level values match RSS log levels:
 *   0=fatal, 1=error, 2=warn, 3=info, 4=debug
 * ================================================================ */

typedef void (*rss_hal_log_func_t)(int level, const char *file, int line, const char *fmt, ...)
    __attribute__((format(printf, 4, 5)));

void rss_hal_set_log_func(rss_hal_log_func_t func);

/* ================================================================
 * Standalone JZDL inference (bypasses IVS pipeline)
 * ================================================================ */

void *hal_jzdl_create(const rss_ivs_jzdl_param_t *param);
int hal_jzdl_detect(void *handle, const uint8_t *nv12_data, rss_ivs_detect_result_t *result);
void hal_jzdl_destroy(void *handle);

#ifdef __cplusplus
}
#endif

#endif /* RAPTOR_HAL_H */
