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

#include "i6_aud.h"
#include "i6_isp.h"
#include "i6_rgn.h"
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
 * OSD regions the backend will track at once.
 *
 * MI publishes no limit on region handles, and neither reference probes
 * for one: divinus numbers handles from its own overlay slots and
 * waybeam uses a single fixed handle. 16 is chosen to match rvd's
 * largest per-platform region budget in hal_caps.c, so a config that
 * works on Ingenic is not silently truncated here. A region costs a
 * handle and its bitmap, nothing per-slot, so the array is cheap.
 */
#define STAR_OSD_REGION_MAX 16

/*
 * Per-region bookkeeping.
 *
 * Everything here is tracked rather than read back from MI, for two
 * reasons. MI_RGN_GetAttr and MI_RGN_GetDisplayAttr exist, but the
 * geometry raptor cares about (x/y/layer/alpha) is spread across the
 * region attr and the *per-channel* display attr, so a read-back needs
 * both plus a live attach; and rvd sets attributes before the region is
 * attached to anything, when there is no display attr to read.
 *
 * `attached` is the distinction that matters: registered means rvd asked
 * for the region to appear on a group, attached means MI has actually
 * been told, which cannot happen until the VPE port exists. See
 * star_osd_flush_pending.
 */
typedef struct {
    bool used;
    rss_osd_type_t type;

    int x;
    int y;
    int width;
    int height;
    int layer;

    bool global_alpha_en;
    unsigned char fg_alpha;
    unsigned char bg_alpha;
    unsigned int cover_color;

    /* Registered group, or -1. */
    int grp;
    bool attached;
    bool show;

    /* Converted bitmap handed to MI_RGN_SetBitMap. Kept per region so a
     * per-frame update does not allocate, and resized only when the
     * region's geometry changes. */
    void *bmp;
    size_t bmp_size;

    /*
     * Set once the first bitmap has been accepted by MI_RGN_SetBitMap, so
     * that fact can be logged exactly once per region. Without it the
     * only two failure modes an invisible overlay can have -- "attached
     * but never fed" and "fed but not composited" -- look identical from
     * the log, which is precisely the hole the const-alpha bug hid in.
     */
    bool bmp_logged;
} star_osd_region_t;

/*
 * Audio input device.
 *
 * MI's AI "device" is not a numbering convenience like it is elsewhere --
 * the MI_AI reference is explicit that it selects the *physical* input
 * ("Amic/Dmic/I2S RX/Line in"), so the index is a board property. 0 is
 * the onboard analog path, which is what both references use and all the
 * vendor examples pass.
 *
 * rad's `[audio] device` defaults to 1 (an Ingenic index), so hal_audio.c
 * configures this device regardless and warns once if it was asked for a
 * different one; raptor-ssc30kq.conf sets the key to 0 to keep the log
 * quiet.
 */
#define STAR_AUD_DEV 0

/*
 * Tracks this backend supports. MI allows up to I6_AUD_CHN_NUM per
 * device, but raptor's audio config only distinguishes mono from stereo
 * (rss_audio_config_t.chn_count is 1 or 2) and rad only ever reads
 * channel 0, so anything beyond 2 would be state nothing can reach.
 */
#define STAR_AUD_CHN_MAX 2

/*
 * How long a blocking MI_AI_GetFrame waits, in ms.
 *
 * 128 is divinus's value (waybeam uses 50). At the 20 ms capture period
 * this backend configures, either is several periods of slack, and the
 * timeout only decides how promptly a stopped device is noticed -- rad
 * treats a timeout as "try again", so a longer wait costs nothing but
 * latency in the failure case.
 */
#define STAR_AUD_GET_TIMEOUT_MS 128

/*
 * MI_AI_SetVqeVolume's argument is an index into a per-device analog-gain
 * table, not a decibel value -- the MI_AI reference spells the table out
 * ("the corresponding gain (DB) of s32volumedb under each device"). The
 * columns are arithmetically self-consistent, which is what makes them
 * safe to quote: Amic runs 0..21 for -6..+57 dB in 3 dB steps, Line in
 * 0..7 for -6..+15 dB, Dmic 0..4 for 0..+24 dB in 6 dB steps.
 *
 * Both references pass dB-shaped numbers straight through -- divinus
 * validates its `[audio] gain` to [-60,30] and waybeam maps 0..100 onto
 * -60..+30 -- so on this API most of their range is out of table. Their
 * *defaults* happen to be valid indices, which is presumably why it never
 * surfaced.
 */
#define STAR_AUD_VOL_MAX_AMIC 21
#define STAR_AUD_VOL_MAX_DMIC 4

/*
 * MI_AI_ERR_BUF_EMPTY -- "audio input buffer is empty", from the MI_AI
 * reference's error-code table. The one MI_AI_GetFrame failure that means
 * "nothing captured yet" rather than "something is wrong", so it is the
 * one hal_audio.c reports as a timeout instead of an error.
 *
 * Quoted from the SSD20X documentation, so treat a *different* code
 * arriving every period on this chip as this constant being wrong rather
 * than as a real fault -- hal_audio.c logs the code it saw, which is what
 * makes that distinguishable.
 */
#define STAR_AUD_ERR_BUF_EMPTY 0xA004200Eu

/*
 * MI_SYS output-port queue for each AI channel, set right after
 * MI_AI_EnableChn. Without it MI_AI_GetFrame returns MI_AI_ERR_NOBUF
 * (0xA004200D) on every call while the device looks healthy -- see the
 * long comment at the call site in hal_audio.c for why these particular
 * numbers and what the three reference sources use.
 */
#define STAR_AUD_PORT_USR_DEPTH 1
#define STAR_AUD_PORT_BUF_DEPTH 16

/*
 * What to fall back to if MI refuses BUF_DEPTH above. The references all use
 * small values (vendor 8, divinus 4, waybeam 2) so a deeper queue is the one
 * thing here with no third-party precedent, and a refused depth fails
 * hal_audio_init outright -- i.e. no audio at all, which is far worse than a
 * shallow queue. 4 is the value this board ran with for the whole bring-up,
 * so it is known to be accepted.
 */
#define STAR_AUD_PORT_BUF_DEPTH_FALLBACK 4

/*
 * "Insufficient audio input buffer" -- the port has no user-side queue. One
 * digit from BUF_EMPTY above and nothing like it in meaning, which is why
 * hal_audio.c names codes in its log instead of printing bare hex.
 */
#define STAR_AUD_ERR_NOBUF 0xA004200Du

/*
 * How many times read_frame will re-establish a channel's output port queue in
 * response to NOBUF before it gives up and just reports the error. Small on
 * purpose: this recovers a port that was lost during bring-up, and must not
 * become an unbounded retry loop running at the capture period.
 */
#define STAR_AUD_NOBUF_RECOVER_MAX 3

/*
 * Consecutive good frames before the NOBUF recovery budget is refilled --
 * 5 seconds at a 20ms period. Long enough that "the port is working again"
 * is a real observation rather than one lucky read between failures.
 */
#define STAR_AUD_NOBUF_RECOVER_REARM_FRAMES 250

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

    /*
     * Set once the ISP has answered and the tuning binary has had its
     * one attempt. Before that the ISP refuses every query -- it is
     * served by the VPE channel, which does not run until an output port
     * is enabled -- so the control ops queue their values instead of
     * failing, and this is the flag that says which of the two applies.
     * See the comment above star_isp_bringup.
     */
    bool isp_tuned;

    /* Gain ceilings requested before the ISP would accept them; -1 for
     * "nothing asked". Not in the IQ table because MI keeps both in the
     * AE exposure-limit struct rather than in a per-module payload. */
    int pend_max_again;
    int pend_max_dgain;

    /* Requested AE max integration time in microseconds, -1 for "leave
     * the tuning's own ceiling alone". Queued like the gain ceilings and
     * for the same reason. */
    int pend_ae_it_max;

    /*
     * The AE's own gain ceilings, snapshotted from whichever tuning is in
     * effect before any config knob overwrites them. These are the
     * calibrated limits and MI treats them as authoritative: a ceiling
     * above them does not stick. waybeam found the same thing the hard way
     * -- "isp.gainMax above the bin ceiling never stuck (found with
     * gainMax=32000 vs bin 8192)", maruko_cus3a.c -- so they are the range
     * a requested ceiling has to be judged against rather than a hint.
     * Zero for "the AE never published them".
     */
    unsigned int bin_min_sensor_gain;
    unsigned int bin_max_sensor_gain;
    unsigned int bin_min_isp_gain;
    unsigned int bin_max_isp_gain;

    /* How many times the tuning binary has been reloaded after finding the
     * ISP back on its defaults. Bounded: a reload that does not stick must
     * not become a loop. */
    int iq_reloads;

    bool gray;
    bool hflip;
    bool vflip;

    star_vpe_port_t port[STAR_VPE_PORT_NUM];
    star_venc_chn_t enc[I6_VENC_CHN_NUM];

    /*
     * OSD state -- src/star/hal_osd.c.
     *
     * MI_RGN has no notion of a region *group*. raptor's group is the
     * encoder channel whose picture the region should appear on, so a
     * group here is just a flag plus the set of regions registered to
     * it, and the real MI operation is attaching the region to the VPE
     * output port feeding that encoder.
     *
     * osd_src_port exists because rvd's bind chain is
     * FS -> OSD -> ENC while MI's data path is VPE port -> VENC with no
     * stage in between. hal_bind records the FS port when it sees
     * FS -> OSD and performs the real bind when it sees OSD -> ENC, so
     * the OSD cell collapses instead of being rejected.
     */
    i6_rgn_impl rgn;
    bool rgn_loaded;
    bool rgn_inited;

    /* Which pixel format MI_RGN_Create actually accepted -- see
     * star_osd_probe_pixfmt. rgn_fmt_known distinguishes "not probed
     * yet" from a successfully probed format. */
    i6_rgn_pixfmt rgn_fmt;
    bool rgn_fmt_known;

    bool osd_grp[I6_VENC_CHN_NUM];
    int osd_src_port[I6_VENC_CHN_NUM];
    star_osd_region_t osd[STAR_OSD_REGION_MAX];

    /*
     * Audio capture state -- src/star/hal_audio.c.
     *
     * Lives in the same struct as the video state even though the two
     * never coexist in one process, because star_state_t is what
     * rss_hal_ctx_t->platform points at and both archives compile
     * hal_common.c. The audio archive simply leaves the video half zero.
     *
     * aud_owns_sys records that audio_init brought MI_SYS up itself.
     * rad calls rss_hal_create and then audio_init directly -- it never
     * calls the init op -- so on this backend audio_init has to do the
     * MI_SYS work that hal_init would otherwise have done, and teardown
     * has to know whether it is entitled to undo it.
     *
     * The volume is tracked rather than read back because MI's getter
     * (MI_AI_GetVqeVolume) reports the *VQE* volume, and the VQE
     * algorithm libraries are absent on this platform.
     */
    i6_aud_impl aud;
    bool aud_loaded;
    bool aud_owns_sys;
    bool aud_dev_enabled;
    bool aud_chn_enabled[STAR_AUD_CHN_MAX];
    int aud_dev;
    unsigned int aud_chn_count;
    int aud_rate;
    int aud_volume;
    rss_audio_input_t aud_input;

    /* One outstanding frame per channel. MI hands out a descriptor that
     * must come back to MI_AI_ReleaseFrame unchanged, and
     * rss_audio_frame_t has nowhere to store 200-odd bytes, so the
     * descriptor stays here and _priv points at it. */
    i6_aud_frm aud_frame[STAR_AUD_CHN_MAX];
    bool aud_frame_held[STAR_AUD_CHN_MAX];

    /* Last MI_AI_GetFrame failure already reported, so a persistent fault
     * is named once instead of every capture period. */
    int aud_last_err;
    bool aud_dev_warned;

    /* NOBUF recovery attempts spent on this channel. Bounds the re-apply in
     * hal_audio_read_frame, which is destructive -- it flushes the port queue.
     * Refilled only after a sustained run of good frames (aud_ok_run), because
     * refilling on a single frame let a fault that alternates with successful
     * reads re-apply at the capture period rate. */
    int aud_nobuf_recover[STAR_AUD_CHN_MAX];
    int aud_ok_run[STAR_AUD_CHN_MAX];

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
 * Bind libmi_isp and work out which tuning binary to load.
 *
 * Called from hal_init. Touches no MI call at all, because at that point
 * the ISP cannot answer one: it is served by the VPE channel, and the
 * VPE channel does not run until an output port is enabled. The actual
 * load is therefore star_isp_tune_when_ready's job -- the long comment
 * above star_isp_bringup explains what this cost on the first board run.
 *
 * cfg supplies the optional iq_file override and the sensor name used to
 * derive the default path.
 */
void star_isp_bringup(star_state_t *st, const rss_sensor_config_t *cfg);

/*
 * Load the tuning binary and flush any queued control values, once the
 * ISP is answering. Idempotent, and a no-op until then, so the enable
 * and start paths can both call it and the first one to find the ISP up
 * wins. verbose=false for early opportunistic attempts, true for the
 * one whose failure is worth a warning.
 */
void star_isp_tune_when_ready(star_state_t *st, bool verbose);

/*
 * Mark the tuning as lost, so the next star_isp_tune_when_ready re-applies
 * it. Called when the last VPE output port is disabled, because stopping
 * the VPE channel is what throws the tuning away -- see the definition.
 */
void star_isp_untune(star_state_t *st);

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
int hal_isp_set_ae_it_max(void *ctx, uint32_t it_max);
int hal_isp_get_ae_it_max(void *ctx, uint32_t *it_max);
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
int hal_isp_get_exposure(void *ctx, rss_exposure_t *exposure);

/*
 * Audio capture ops -- src/star/hal_audio.c.
 *
 * This is the whole of it. Everything else in the audio half of
 * rss_hal_ops_t stays NULL, which RSS_HAL_CALL already turns into
 * RSS_ERR_NOTSUP; hal_audio.c's OP COVERAGE comment says why for each.
 */
int hal_audio_init(void *ctx, const rss_audio_config_t *cfg);
int hal_audio_deinit(void *ctx);
int hal_audio_read_frame(void *ctx, int dev, int chn, rss_audio_frame_t *frame, bool block);
int hal_audio_release_frame(void *ctx, int dev, int chn, rss_audio_frame_t *frame);
int hal_audio_set_volume(void *ctx, int dev, int chn, int vol);
int hal_audio_get_volume(void *ctx, int dev, int chn, int *vol);
int hal_audio_set_mute(void *ctx, int dev, int chn, int mute);

/*
 * OSD ops -- src/star/hal_osd.c.
 *
 * Twelve ops, which is what rvd calls; hal_osd.c's OP COVERAGE comment
 * names the five it leaves NULL and why.
 */
int hal_osd_set_pool_size(void *ctx, uint32_t bytes);
int hal_osd_create_group(void *ctx, int grp);
int hal_osd_destroy_group(void *ctx, int grp);
int hal_osd_start(void *ctx, int grp);
int hal_osd_stop(void *ctx, int grp);
int hal_osd_create_region(void *ctx, int *handle, const rss_osd_region_t *attr);
int hal_osd_destroy_region(void *ctx, int handle);
int hal_osd_register_region(void *ctx, int handle, int grp);
int hal_osd_unregister_region(void *ctx, int handle, int grp);
int hal_osd_set_region_attr(void *ctx, int handle, const rss_osd_region_t *attr);
int hal_osd_update_region_data(void *ctx, int handle, const uint8_t *data);
int hal_osd_show_region(void *ctx, int handle, int grp, int show, int layer);

/*
 * Attach any region registered to this encoder channel, called once the
 * VPE port -> VENC bind exists. rvd registers regions before it binds,
 * so this is where startup overlays actually reach MI.
 */
void star_osd_flush_pending(star_state_t *st, int chn);
void star_osd_release_all(star_state_t *st);

#endif /* STAR_STATE_H */
