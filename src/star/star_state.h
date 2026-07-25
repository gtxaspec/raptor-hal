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

#include "i6_snr.h"
#include "i6_sys.h"
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

typedef struct {
    i6_sys_impl sys;
    i6_snr_impl snr;
    i6_vif_impl vif;
    i6_vpe_impl vpe;

    /* Sensor descriptors, read back after MI_SNR_Enable (see hal_init) */
    i6_snr_pad pad;
    i6_snr_plane plane;

    /* Selected sensor mode */
    i6_snr_res res;
    unsigned char res_index;

    /* Sensor frame rate, as programmed. Used for the VIF->VPE bind and
     * as the source rate for 2d's VPE->VENC bind. */
    unsigned int fps;

    star_vpe_port_t port[STAR_VPE_PORT_NUM];

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

#endif /* STAR_STATE_H */
