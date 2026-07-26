/*
 * star/star_state.h -- shared backend state for the SigmaStar MI HAL
 *
 * Exists because the MI backend spans more than one translation unit:
 * hal_common.c owns the pipeline lifecycle (MI_SYS / MI_SNR / MI_VIF /
 * the VPE channel) and hal_framesource.c owns the VPE output ports, and
 * both need the same loaded-library handles and sensor descriptors.
 *
 * It cannot live in hal_internal.h: that header is included *by* the
 * i6_*.h ABI headers (for HAL_LOG_ERR and RSS_ERR_*), so it must not
 * include them back. rss_hal_ctx_t->platform points at the
 * star_state_t below for exactly this reason -- see the MI BACKEND
 * STATE comment in hal_common.c.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef STAR_STATE_H
#define STAR_STATE_H

#include "hal_internal.h"

#include "i6_isp.h"
#include "i6_snr.h"
#include "i6_sys.h"
#include "i6_venc.h"
#include "i6_vif.h"
#include "i6_vpe.h"

/* ================================================================
 * FIXED TOPOLOGY
 *
 * One sensor on VIF device 0 / channel 0 / port 0, feeding VPE
 * device 0 / channel 0. MI's device and channel numbering only
 * becomes interesting with several sensors, and this target has one.
 * Task 2e generalizes sensor *selection*; the topology stays fixed.
 * ================================================================ */

#define STAR_SNR_INDEX 0
#define STAR_VIF_DEV 0
#define STAR_VIF_CHN 0
#define STAR_VIF_PORT 0
#define STAR_VPE_DEV 0
#define STAR_VPE_CHN 0

/*
 * ISP channel. Every MI_ISP call takes a channel, and it is the *VPE*
 * channel index -- MI's ISP is not a separate device but the front half
 * of the VPE channel, which is why enabling VPE auto-starts CUS3A and
 * why the IQ binary is loaded per VPE channel. Both references pass a
 * bare 0 here (divinus's _i6_isp_chn, waybeam's literal); naming it
 * keeps the coupling to STAR_VPE_CHN visible.
 */
#define STAR_ISP_CHN STAR_VPE_CHN

/*
 * The key MI_ISP_API_CmdLoadBinFile wants alongside the path. Not a
 * checksum of anything -- both references pass this same literal
 * (divinus i6_hal.c:215, waybeam star6e_pipeline.c:295), and the
 * wrapper reads the file itself with fopen, so the value is a
 * protocol constant rather than a property of the binary.
 */
#define STAR_IQ_LOAD_KEY 1234u

/*
 * VPE output ports per channel. divinus's teardown disables ports 0..3
 * (i6_hal.c:365) and waybeam only ever uses 0 and 1, so 4 is the
 * documented-by-use bound. star_probe -v reports which ports actually
 * accept MI_VPE_SetPortMode on this silicon; hal_caps.c's
 * max_fs_channels quotes that result.
 */
#define STAR_VPE_PORT_NUM 4

/*
 * Default output-port buffer-queue depth, and how long a blocking
 * frame fetch waits.
 *
 * 3 is the vendor's own number: the MI_SYS_ChnOutputPortGetBuf sample
 * in SigmaStar's MI_SYS reference (ref/sigmastar-docs, MI SYS API 2.25)
 * runs SetChnOutputPortDepth(&port, 2, 3), then releases with
 * (&port, 0, 3) -- user depth to zero, queue depth left alone. That
 * release pattern is exactly what rvd's fs_set_frame_depth(chn, 0)
 * means, so the two models line up without inventing anything.
 */
#define STAR_VPE_QUEUE_DEPTH 3
#define STAR_FRAME_TIMEOUT_MS 2000

/*
 * VENC channels have one input port, always 0 -- divinus keeps it in a
 * variable (_i6_venc_port) only because it shares this file across four
 * SoC families.
 */
#define STAR_VENC_PORT 0

/*
 * How many i6_venc_pack the encoder may return for one frame without
 * heap traffic in the streaming path.
 *
 * The vendor sample sizes this array from MI_VENC_Query's u32CurPacks
 * and mallocs it per frame; divinus keeps a stack array of 8 and only
 * mallocs above that (i6_hal.c:854). H.264/H.265 hand back one pack per
 * frame here -- multiple NAL units ride *inside* a pack, described by
 * its packetInfo -- so this is generous in practice.
 *
 * 16 rather than divinus's 8 because that is rvd's own ceiling: its
 * encoder thread copies at most 16 NALs into the ring and warns when it
 * truncates (rvd_frame_loop.c:256). Matching it means the HAL never
 * becomes the tighter limit.
 *
 * MI_VENC_GetStream is handed a pack array the caller sizes, and
 * whether it respects that size or writes u32CurPacks entries regardless
 * is not documented -- divinus mallocs precisely to avoid finding out.
 * So does star_enc_packs: above this bound the array is heap-allocated
 * to the full count, and only the NAL *reporting* is capped.
 */
#define STAR_VENC_MAX_PACKS 16

/*
 * Per-VPE-port state. One of these is a raptor "framesource channel":
 * raptor fs channel N maps to VPE port N with no indirection, which is
 * also how divinus does it (i6_channel_create(index) configures port
 * `index`, and i6_channel_bind binds that same index to VENC channel
 * `index`). Keeping the identity mapping means 2d's VENC bind needs no
 * lookup table.
 */
typedef struct {
    bool configured; /* MI_VPE_SetPortMode has succeeded */
    bool enabled;    /* MI_VPE_EnablePort has succeeded */

    unsigned short width;
    unsigned short height;
    i6_common_pixfmt pixFmt;

    /*
     * Frame rate is not a VPE port attribute -- MI applies rate control
     * when binding (MI_SYS_BindChnPort2 takes srcFps/dstFps). Kept here
     * so 2d's VPE->VENC bind can use the rate the caller asked for.
     */
    unsigned int fps_num;
    unsigned int fps_den;

    /* Both arguments of MI_SYS_SetChnOutputPortDepth, tracked because MI
     * offers no getter for either. */
    unsigned int user_depth;
    unsigned int queue_depth;

    /* MI_SYS_GetFd wakeup descriptor, opened on the first frame fetch
     * and closed when the port is disabled. -1 when not open. */
    int fd;

    /* At most one outstanding frame per port -- see hal_fs_get_frame. */
    bool frame_held;
    int frame_handle;
    i6_sys_bufinfo frame;
} star_vpe_port_t;

/*
 * Per-VENC-channel state.
 *
 * raptor encoder channel N is MI VENC channel N. rvd binds framesource
 * channel N to encoder channel N by default but does not require it, so
 * `src_port` records which VPE port was actually bound rather than
 * assuming the identity -- unbind has to name the same pair.
 */
typedef struct {
    bool created;   /* MI_VENC_CreateChn has succeeded */
    bool receiving; /* MI_VENC_StartRecvPic has succeeded */
    bool bound;     /* a VPE port is bound to this channel */

    /*
     * MI_VENC_GetChnDevid's answer, cached at create time. The bind
     * needs it and it cannot be read back once the channel is
     * destroyed, which is the order teardown runs in.
     */
    unsigned int device;
    int src_port; /* bound VPE port, -1 when unbound */

    rss_codec_t codec;
    unsigned short width;
    unsigned short height;
    unsigned int fps_num;
    unsigned int fps_den;
    unsigned int gop;
    rss_rc_mode_t rc_mode;
    unsigned int bitrate;     /* bps, as the caller expressed it */
    unsigned int max_bitrate; /* bps */

    /*
     * QP bounds as the caller gave them, -1 meaning "SDK default".
     * Kept because MI has no per-knob setter: changing the bitrate
     * rewrites the whole rate struct, and without these a set_bitrate
     * would quietly reset the QP bounds the channel was created with.
     */
    int16_t init_qp;
    int16_t min_qp;
    int16_t max_qp;

    /* MI_VENC_GetFd descriptor, opened on first poll. -1 when closed. */
    int fd;

    /*
     * One outstanding stream per channel, mirroring the framesource
     * ports. The pack array backs strm.packet, and the NAL array backs
     * the rss_frame_t handed out by enc_get_frame -- both must outlive
     * the call, and neither may be reused until enc_release_frame.
     */
    bool frame_held;
    i6_venc_strm strm;
    i6_venc_pack packs[STAR_VENC_MAX_PACKS];
    rss_nal_unit_t nals[STAR_VENC_MAX_PACKS];

    /* Oversized pack array for the rare frame above STAR_VENC_MAX_PACKS.
     * Allocated on demand, grown never shrunk, freed at destroy. */
    i6_venc_pack *heap_packs;
    unsigned int heap_count;
} star_venc_chn_t;

typedef struct {
    i6_sys_impl sys;
    i6_snr_impl snr;
    i6_vif_impl vif;
    i6_vpe_impl vpe;
    i6_venc_impl venc;
    i6_isp_impl isp;

    /* Sensor descriptors, read back after MI_SNR_Enable (see hal_init) */
    i6_snr_pad pad;
    i6_snr_plane plane;

    /* Selected sensor mode */
    i6_snr_res res;
    unsigned char res_index;

    /* Sensor frame rate, as programmed. Used for the VIF->VPE bind and
     * as the source rate for 2d's VPE->VENC bind. */
    unsigned int fps;

    /*
     * ISP state.
     *
     * iq_file is the tuning binary actually loaded, empty when none was.
     * Kept because MI offers no way to ask what is loaded, and because a
     * *reload* is not free: the vendor AE reinitialises the sensor
     * shutter register from the binary's own defaults, which on a
     * running pipeline shows up as a framerate change. waybeam skips
     * redundant loads for exactly that reason
     * (star6e_pipeline.c:2073-2077).
     *
     * gray and the flip pair exist because MI has no getter for either:
     * MI_ISP_IQ_GetColorToGray reads the IQ struct rather than a
     * day/night mode, and MI_SNR_SetOrien has no counterpart at all.
     */
    char iq_file[128];
    bool isp_loaded;
    bool gray;
    bool hflip;
    bool vflip;

    star_vpe_port_t port[STAR_VPE_PORT_NUM];
    star_venc_chn_t enc[I6_VENC_CHN_NUM];

    /* Unwind flags -- each set only once its step has succeeded, so
     * teardown undoes exactly what was done and no more. */
    bool sys_inited;
    bool snr_enabled;
    bool vif_dev_enabled;
    bool vif_port_enabled;
    bool vpe_chn_created;
    bool vpe_chn_started;
    bool vif_vpe_bound;
} star_state_t;

static inline star_state_t *star_state(void *ctx)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;

    return c ? (star_state_t *)c->platform : NULL;
}

/*
 * star_vpe_pixfmt -- the pixel format the VPE *channel* consumes.
 *
 * Shared by hal_common.c (channel attributes) and star_probe. Bayer
 * sensors need this derived rather than read from the plane's own
 * pixFmt field; see the long comment on star_vif_pixfmt in
 * hal_common.c for the hardware evidence.
 */
i6_common_pixfmt star_vif_pixfmt(const i6_snr_plane *plane);

/* Framesource ops -- src/star/hal_framesource.c */
int hal_fs_create_channel(void *ctx, int chn, const rss_fs_config_t *cfg);
int hal_fs_set_channel_attr(void *ctx, int chn, const rss_fs_config_t *cfg);
int hal_fs_destroy_channel(void *ctx, int chn);
int hal_fs_enable_channel(void *ctx, int chn);
int hal_fs_disable_channel(void *ctx, int chn);
int hal_fs_set_fifo(void *ctx, int chn, int depth);
int hal_fs_get_fifo(void *ctx, int chn, int *depth);
int hal_fs_set_frame_depth(void *ctx, int chn, int depth);
int hal_fs_get_frame_depth(void *ctx, int chn, int *depth);
int hal_fs_get_frame(void *ctx, int chn, void **frame_data, rss_frame_info_t *info);
int hal_fs_release_frame(void *ctx, int chn, void *frame_data);

/* Called from star_teardown so a port's frame, fd and enable state do
 * not outlive the VPE channel. */
void star_fs_release_all(star_state_t *st);

/* Encoder ops -- src/star/hal_encoder.c */
int hal_enc_create_group(void *ctx, int grp);
int hal_enc_destroy_group(void *ctx, int grp);
int hal_enc_create_channel(void *ctx, int chn, const rss_video_config_t *cfg);
int hal_enc_destroy_channel(void *ctx, int chn);
int hal_enc_register_channel(void *ctx, int grp, int chn);
int hal_enc_unregister_channel(void *ctx, int chn);
int hal_enc_start(void *ctx, int chn);
int hal_enc_stop(void *ctx, int chn);
int hal_enc_poll(void *ctx, int chn, uint32_t timeout_ms);
int hal_enc_get_frame(void *ctx, int chn, rss_frame_t *frame);
int hal_enc_release_frame(void *ctx, int chn, rss_frame_t *frame);
int hal_enc_request_idr(void *ctx, int chn);
int hal_enc_set_rc_mode(void *ctx, int chn, rss_rc_mode_t mode, uint32_t bitrate);
int hal_enc_set_bitrate(void *ctx, int chn, uint32_t bitrate);
int hal_enc_set_gop(void *ctx, int chn, uint32_t gop_length);
int hal_enc_set_fps(void *ctx, int chn, uint32_t fps_num, uint32_t fps_den);
int hal_enc_get_channel_attr(void *ctx, int chn, rss_video_config_t *cfg);
int hal_enc_get_fps(void *ctx, int chn, uint32_t *fps_num, uint32_t *fps_den);
int hal_enc_get_gop_attr(void *ctx, int chn, uint32_t *gop_length);
int hal_enc_set_gop_attr(void *ctx, int chn, uint32_t gop_length);
int hal_enc_get_avg_bitrate(void *ctx, int chn, uint32_t *bitrate);
int hal_enc_query(void *ctx, int chn, bool *busy);
int hal_enc_get_fd(void *ctx, int chn);

/*
 * The VPE-port half of a bind, shared with hal_common.c's bind/unbind:
 * MI binds VPE port -> VENC channel, so the encoder side owns the
 * device id and the bound-port bookkeeping.
 */
int star_enc_bind_port(star_state_t *st, int port, int chn);
int star_enc_unbind_port(star_state_t *st, int port, int chn);

/* Called from star_teardown, before the VPE channel goes away. */
void star_enc_release_all(star_state_t *st);

/* ================================================================
 * ISP -- src/star/hal_isp.c
 * ================================================================ */

/*
 * Bring the ISP up: bind libmi_isp, wait for the IQ parameter store to
 * initialise, load the sensor's tuning binary, then start CUS3A.
 *
 * Called from hal_init *after* the VPE channel is created and bound,
 * because none of it is legal before then -- the ISP channel is the
 * front half of the VPE channel (see STAR_ISP_CHN). Failure is
 * deliberately non-fatal: a camera with an untuned image is worth far
 * more than a camera that refuses to stream, which is also why phase
 * 2's colour cast was a defect and not an outage.
 *
 * cfg supplies the optional iq_file override and the sensor name used
 * to derive the default path.
 */
void star_isp_bringup(star_state_t *st, const rss_sensor_config_t *cfg);

/* Release the ISP libraries. Called from star_teardown. */
void star_isp_teardown(star_state_t *st);

/*
 * Clamp the AE's maximum shutter to one frame period.
 *
 * Called from star_isp_bringup after the tuning binary is loaded, since
 * the binary carries its own AE limits and they are not required to
 * suit the framerate this pipeline asked for. An uncapped AE converges
 * on an exposure longer than the frame period in dim light, and the
 * sensor answers by dropping its own rate -- a 30 fps request silently
 * delivering 12 fps, with nothing in any log to say why.
 */
int star_isp_cap_exposure(star_state_t *st, unsigned int fps);

/* ISP ops. Only those MI can honour are defined; see the OP COVERAGE
 * comment in hal_isp.c for what is deliberately absent and why. */
int hal_isp_set_brightness(void *ctx, int val);
int hal_isp_set_contrast(void *ctx, int val);
int hal_isp_set_saturation(void *ctx, int val);
int hal_isp_set_sharpness(void *ctx, int val);
int hal_isp_set_sinter_strength(void *ctx, int val);
int hal_isp_set_temper_strength(void *ctx, int val);
int hal_isp_set_ae_comp(void *ctx, int val);
int hal_isp_set_defog(void *ctx, int enable);
int hal_isp_set_antiflicker(void *ctx, rss_antiflicker_t mode);
int hal_isp_set_max_again(void *ctx, int gain);
int hal_isp_set_max_dgain(void *ctx, int gain);
int hal_isp_set_running_mode(void *ctx, rss_isp_mode_t mode);
int hal_isp_set_hflip(void *ctx, int enable);
int hal_isp_set_vflip(void *ctx, int enable);

int hal_isp_get_brightness(void *ctx, uint8_t *val);
int hal_isp_get_contrast(void *ctx, uint8_t *val);
int hal_isp_get_saturation(void *ctx, uint8_t *val);
int hal_isp_get_sharpness(void *ctx, uint8_t *val);
int hal_isp_get_sinter_strength(void *ctx, uint8_t *val);
int hal_isp_get_temper_strength(void *ctx, uint8_t *val);
int hal_isp_get_ae_comp(void *ctx, int *val);
int hal_isp_get_antiflicker(void *ctx, rss_antiflicker_t *mode);
int hal_isp_get_max_again(void *ctx, uint32_t *gain);
int hal_isp_get_max_dgain(void *ctx, uint32_t *gain);
int hal_isp_get_running_mode(void *ctx, rss_isp_mode_t *mode);
int hal_isp_get_hvflip(void *ctx, int *hflip, int *vflip);

#endif /* STAR_STATE_H */
