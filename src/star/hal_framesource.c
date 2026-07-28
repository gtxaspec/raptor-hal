/*
 * star/hal_framesource.c -- Raptor HAL framesource, SigmaStar MI backend
 *
 * Counterpart to src/hal_framesource.c (Ingenic IMP FrameSource).
 *
 * The mapping, which is the whole design decision in this file:
 *
 *   raptor framesource channel N  ==  VPE channel 0, output port N
 *
 * MI's pipeline is VIF -> VPE -> VENC, where the VPE *channel* is the
 * ISP instance for one sensor and its output *ports* are the scaled
 * taps that encoders bind to. So a raptor "framesource channel" -- one
 * scaled YUV stream off the sensor -- is a VPE port, not a VPE channel.
 * The channel itself is created once during hal_init, because it needs
 * the sensor geometry and has to be bound to VIF before any pixels
 * move; see star_vpe_bringup in hal_common.c.
 *
 * divinus maps it the same way (i6_channel_create(index) configures
 * port `index`; i6_channel_bind binds port `index` to VENC channel
 * `index`), so the identity mapping also means 2d's VENC bind needs no
 * lookup table.
 *
 * Ops MI has no equivalent for are simply absent from the vtable in
 * hal_common.c rather than stubbed here:
 *
 *   fs_set_rotation      i6e_vpe_chn has a rotateOn flag, but it is a
 *                        channel attribute -- changing it means
 *                        recreating the channel and rebinding VIF, not
 *                        a per-port call.
 *   fs_snap_frame        no MI one-shot capture. rvd's snapshot path
 *                        (rvd_ctrl.c:1795) already works the other way,
 *                        via fs_set_frame_depth(1) + fs_get_frame,
 *                        which is implemented here.
 *   fs_set/get_delay,    IMP-specific FrameSource notions with nothing
 *   max_delay, pool,     behind them in MI.
 *   timed_frame,
 *   frame_offset,
 *   chn_stat_query,
 *   undistort
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "star_state.h"

#include <sys/select.h>

/* ================================================================
 * PIXEL FORMAT TRANSLATION
 * ================================================================ */

/*
 * star_fs_pixfmt -- rss_pixfmt_t to the MI format a VPE port can emit.
 *
 * Returns I6_PIXFMT_END for formats VPE cannot produce, which callers
 * turn into RSS_ERR_NOTSUP. RSS_PIXFMT_RAW is deliberately in that set:
 * the bayer domain ends at VIF on this hardware, and VPE ports emit YUV
 * only. Quietly substituting NV12 would write YUV into a file the
 * caller asked to be raw bayer, which is worse than failing.
 */
static i6_common_pixfmt star_fs_pixfmt(rss_pixfmt_t fmt)
{
    switch (fmt) {
    case RSS_PIXFMT_NV12:
        return I6_PIXFMT_YUV420SP;
    case RSS_PIXFMT_NV21:
        return I6_PIXFMT_YUV420SP_NV21;
    case RSS_PIXFMT_YUYV422:
        return I6_PIXFMT_YUV422_YUYV;
    case RSS_PIXFMT_UYVY422:
        return I6_PIXFMT_YUV422_UYVY;
    case RSS_PIXFMT_GRAY8:
        return I6_PIXFMT_GRAY8;
    default:
        return I6_PIXFMT_END;
    }
}

static rss_pixfmt_t star_fs_pixfmt_rev(i6_common_pixfmt fmt)
{
    switch (fmt) {
    case I6_PIXFMT_YUV420SP:
        return RSS_PIXFMT_NV12;
    case I6_PIXFMT_YUV420SP_NV21:
        return RSS_PIXFMT_NV21;
    case I6_PIXFMT_YUV422_YUYV:
        return RSS_PIXFMT_YUYV422;
    case I6_PIXFMT_YUV422_UYVY:
        return RSS_PIXFMT_UYVY422;
    case I6_PIXFMT_GRAY8:
        return RSS_PIXFMT_GRAY8;
    default:
        return RSS_PIXFMT_NV12;
    }
}

/* ================================================================
 * PORT LOOKUP
 * ================================================================ */

static star_vpe_port_t *star_fs_port(void *ctx, int chn)
{
    star_state_t *st = star_state(ctx);

    if (!st || chn < 0 || chn >= STAR_VPE_PORT_NUM)
        return NULL;

    return &st->port[chn];
}

/*
 * Every op needs the same three things: the state, the port, and a live
 * VPE channel. Fold the checks so each op reads as its MI call.
 */
#define STAR_FS_ENTER(ctx, chn, st_var, port_var)                                                  \
    star_state_t *st_var = star_state(ctx);                                                        \
    star_vpe_port_t *port_var = star_fs_port(ctx, chn);                                            \
    if (!st_var || !port_var)                                                                      \
        return RSS_ERR_INVAL;                                                                      \
    if (!st_var->vpe_chn_started)                                                                  \
        return RSS_ERR_NOENT

/*
 * star_fs_apply_depth -- push both depths to MI in one call.
 *
 * MI_SYS_SetChnOutputPortDepth(port, userFrameDepth, bufQueueDepth)
 * sets "the maximum number of buf that the output user can get" and
 * "the maximum number of buf for this output system" respectively
 * (SigmaStar MI_SYS reference, 2.4.13). raptor splits those across two
 * ops -- fs_set_frame_depth is the user depth, fs_set_fifo the queue
 * depth -- so both funnel through here.
 *
 * Deferred until the port is enabled. rvd configures a channel before
 * starting it (rvd_pipeline.c:873 calls fs_set_fifo and
 * fs_set_frame_depth straight after fs_create_channel), and setting a
 * depth on a port MI has not enabled yet is untested on this silicon;
 * caching costs nothing and keeps the call order the same as the
 * references'.
 */
static int star_fs_apply_depth(star_state_t *st, int chn, star_vpe_port_t *port)
{
    i6_sys_bind bind;
    int ret;

    if (!port->enabled)
        return RSS_OK;

    if (!st->sys.fnSetOutputDepth) {
        HAL_LOG_ERR("MI_SYS_SetChnOutputPortDepth unavailable");
        return RSS_ERR_NOTSUP;
    }

    memset(&bind, 0, sizeof(bind));
    bind.module = I6_SYS_MOD_VPE;
    bind.device = STAR_VPE_DEV;
    bind.channel = STAR_VPE_CHN;
    bind.port = chn;

    ret = st->sys.fnSetOutputDepth(&bind, port->user_depth, port->queue_depth);
    if (ret) {
        HAL_LOG_ERR("MI_SYS_SetChnOutputPortDepth(vpe port %d, %u, %u) failed: %d", chn,
                    port->user_depth, port->queue_depth, ret);
        return RSS_ERR_IO;
    }

    return RSS_OK;
}

/*
 * star_fs_drop_frame -- put back a held frame and forget it.
 *
 * Called on disable and teardown. A frame still checked out when the
 * port goes down would leak an MI buffer handle for the life of the
 * process, and MI's queue depth is 3.
 */
static void star_fs_drop_frame(star_state_t *st, star_vpe_port_t *port)
{
    if (!port->frame_held)
        return;

    if (st->sys.fnPutOutputBuf)
        (void)st->sys.fnPutOutputBuf(port->frame_handle);
    port->frame_held = false;
    port->frame_handle = 0;
}

static void star_fs_close_fd(star_state_t *st, star_vpe_port_t *port)
{
    if (port->fd < 0)
        return;

    if (st->sys.fnCloseFd)
        (void)st->sys.fnCloseFd(port->fd);
    port->fd = -1;
}

void star_fs_release_all(star_state_t *st)
{
    int i;

    if (!st)
        return;

    for (i = 0; i < STAR_VPE_PORT_NUM; i++) {
        star_fs_drop_frame(st, &st->port[i]);
        star_fs_close_fd(st, &st->port[i]);
    }
}

/* ================================================================
 * CHANNEL (VPE PORT) LIFECYCLE
 * ================================================================ */

/*
 * star_fs_configure -- MI_VPE_SetPortMode from an rss_fs_config_t.
 *
 * cfg->width/height is already the *output* size: rvd sets
 * scaler.out_width/out_height to the same values whenever they differ
 * from the sensor (rvd_pipeline.c:838), so the scaler block is a
 * statement of intent rather than a separate geometry. Honour it when
 * present and fall back to width/height, which keeps a caller that
 * fills in only one of the two working either way.
 *
 * Two config fields have no VPE port equivalent and are reported rather
 * than ignored:
 *
 *   cfg->crop   MI does have MI_VPE_SetPortCrop, but its rect is in the
 *               VPE *input* domain and it exists for digital zoom
 *               (waybeam drives pan/zoom with it). rvd only sets crop
 *               for multi-sensor, which this backend declares
 *               unsupported (max_sensors = 1), so wiring it now would
 *               be untestable code. Phase 3.
 *   cfg->fcrop  post-scaler crop, T23-only in IMP. No MI counterpart.
 */
static int star_fs_configure(star_state_t *st, int chn, star_vpe_port_t *port,
                             const rss_fs_config_t *cfg)
{
    i6_vpe_port attr;
    i6_common_pixfmt pixFmt;
    unsigned short width, height;
    int ret;

    pixFmt = star_fs_pixfmt(cfg->pixfmt);
    if (pixFmt == I6_PIXFMT_END) {
        HAL_LOG_ERR("fs chn %d: pixfmt %d cannot be produced by a VPE port "
                    "(raw bayer ends at VIF; VPE emits YUV)",
                    chn, cfg->pixfmt);
        return RSS_ERR_NOTSUP;
    }

    width = cfg->width;
    height = cfg->height;
    if (cfg->scaler.enable && cfg->scaler.out_width > 0 && cfg->scaler.out_height > 0) {
        width = (unsigned short)cfg->scaler.out_width;
        height = (unsigned short)cfg->scaler.out_height;
    }

    if (!width || !height) {
        HAL_LOG_ERR("fs chn %d: zero output geometry (%ux%u)", chn, width, height);
        return RSS_ERR_INVAL;
    }

    if (cfg->crop.enable)
        HAL_LOG_WARN("fs chn %d: crop %dx%d+%d+%d ignored -- MI_VPE_SetPortCrop is phase 3", chn,
                     cfg->crop.w, cfg->crop.h, cfg->crop.x, cfg->crop.y);
    if (cfg->fcrop.enable)
        HAL_LOG_WARN("fs chn %d: post-scaler crop has no MI equivalent, ignored", chn);

    memset(&attr, 0, sizeof(attr));
    attr.output.width = width;
    attr.output.height = height;
    /*
     * Digital mirror/flip stay off here. Orientation is applied at the
     * sensor (MI_SNR_SetOrien), which is what both references do --
     * waybeam explicitly keeps the VPE mirror/flip fields at 0 because
     * they are "unreliable on several sensor combos"
     * (star6e_pipeline.c:547).
     */
    attr.mirror = 0;
    attr.flip = 0;
    attr.pixFmt = pixFmt;
    attr.compress = I6_COMPR_NONE;

    ret = st->vpe.fnSetPortConfig(STAR_VPE_CHN, chn, &attr);
    if (ret) {
        HAL_LOG_ERR("MI_VPE_SetPortMode(%d, %d) %ux%u pixFmt %d failed: %d", STAR_VPE_CHN, chn,
                    width, height, pixFmt, ret);
        return RSS_ERR_IO;
    }

    port->configured = true;
    port->width = width;
    port->height = height;
    port->pixFmt = pixFmt;
    port->fps_num = cfg->fps_num;
    port->fps_den = cfg->fps_den;
    /* nr_vbs is IMP's per-channel buffer count; MI's nearest equivalent
     * is the output port's queue depth. */
    port->queue_depth = cfg->nr_vbs > 0 ? (unsigned int)cfg->nr_vbs : STAR_VPE_QUEUE_DEPTH;

    HAL_LOG_INFO("fs chn %d: VPE port %ux%u pixFmt %d, queue depth %u", chn, width, height, pixFmt,
                 port->queue_depth);

    return RSS_OK;
}

int hal_fs_create_channel(void *ctx, int chn, const rss_fs_config_t *cfg)
{
    STAR_FS_ENTER(ctx, chn, st, port);

    if (!cfg)
        return RSS_ERR_INVAL;

    if (port->configured) {
        HAL_LOG_ERR("fs chn %d: already created", chn);
        return RSS_ERR_BUSY;
    }

    return star_fs_configure(st, chn, port, cfg);
}

/*
 * MI_VPE_SetPortMode is idempotent and takes effect on a live port, so
 * reconfiguring is the same call. Unlike IMP there is no separate
 * "set attr on an existing channel" entry point to get wrong.
 */
int hal_fs_set_channel_attr(void *ctx, int chn, const rss_fs_config_t *cfg)
{
    STAR_FS_ENTER(ctx, chn, st, port);

    if (!cfg)
        return RSS_ERR_INVAL;

    return star_fs_configure(st, chn, port, cfg);
}

int hal_fs_enable_channel(void *ctx, int chn)
{
    int ret;

    STAR_FS_ENTER(ctx, chn, st, port);

    if (!port->configured) {
        HAL_LOG_ERR("fs chn %d: enable before create", chn);
        return RSS_ERR_INVAL;
    }
    if (port->enabled)
        return RSS_OK;

    ret = st->vpe.fnEnablePort(STAR_VPE_CHN, chn);
    if (ret) {
        HAL_LOG_ERR("MI_VPE_EnablePort(%d, %d) failed: %d", STAR_VPE_CHN, chn, ret);
        return RSS_ERR_IO;
    }
    port->enabled = true;

    /*
     * Enabling a port is what finally makes the VPE channel run, and the
     * ISP is served by that channel -- so this is the earliest moment the
     * ISP can answer anything, and the earliest the sensor's tuning
     * binary can be loaded. Opportunistic and quiet: frames need a moment
     * to start, and hal_enc_start tries again straight afterwards.
     */
    star_isp_tune_when_ready(st, false);

    /* Flush whatever depths were requested while the port was down. */
    return star_fs_apply_depth(st, chn, port);
}

/* True while any output port is still up, and so while the VPE channel --
 * and with it the ISP -- is still running. */
static bool star_fs_any_port_enabled(const star_state_t *st)
{
    int i;

    for (i = 0; i < STAR_VPE_PORT_NUM; i++)
        if (st->port[i].enabled)
            return true;

    return false;
}

int hal_fs_disable_channel(void *ctx, int chn)
{
    int ret;

    STAR_FS_ENTER(ctx, chn, st, port);

    if (!port->enabled)
        return RSS_OK;

    star_fs_drop_frame(st, port);
    star_fs_close_fd(st, port);

    ret = st->vpe.fnDisablePort(STAR_VPE_CHN, chn);
    port->enabled = false;

    /*
     * The last port down stops the VPE channel, and stopping it discards
     * the ISP tuning -- CUS3A restarts on the generic binary when the
     * channel comes back. Say so now so the next enable reloads the
     * sensor's binary instead of trusting a latch that outlived what it
     * described. Done even when DisablePort reported an error: the port is
     * marked down either way, so the tuning must not stay marked live.
     */
    if (!star_fs_any_port_enabled(st))
        star_isp_untune(st);

    if (ret) {
        HAL_LOG_ERR("MI_VPE_DisablePort(%d, %d) failed: %d", STAR_VPE_CHN, chn, ret);
        return RSS_ERR_IO;
    }

    return RSS_OK;
}

/*
 * MI has no per-port destroy -- ports belong to the VPE channel and go
 * away with it (MI_VPE_DestroyChannel, in star_teardown). Disabling and
 * forgetting the configuration is the whole of it, which also makes
 * destroy/create cycles work: rvd does exactly that when a stream's
 * geometry changes.
 */
int hal_fs_destroy_channel(void *ctx, int chn)
{
    int ret;

    STAR_FS_ENTER(ctx, chn, st, port);

    ret = hal_fs_disable_channel(ctx, chn);

    port->configured = false;
    port->width = 0;
    port->height = 0;
    port->user_depth = 0;
    port->queue_depth = 0;

    return ret;
}

/*
 * star_fs_clone_port -- give a second VPE output port the same picture.
 *
 * A JPEG snapshot channel needs its own VPE port, and rvd will not
 * configure one: it points the JPEG stream's fs_chn at the *video*
 * stream's channel and never calls fs_create_channel for it, because on
 * Ingenic a JPEG channel rides its paired video stream's encoder group
 * instead of having a source of its own. hal_enc_register_channel is
 * where that difference is absorbed, and this is the half of it that
 * belongs to the framesource: build the port MI needs from the geometry
 * of the one already running.
 *
 * Cloning rather than deriving from the stream config is deliberate. The
 * snapshot must show what the stream shows, and the video port has
 * already had rvd's scaler decision applied to it (star_fs_configure
 * resolves cfg->scaler before storing width/height), so the port that
 * exists is a better statement of the intended picture than the config
 * that produced it.
 *
 * The port is left enabled: unlike rvd's ports, whose enable is a
 * separate fs_enable_channel call at stream start, nothing downstream
 * will ever call one for this port. Enabling here also gets the depth
 * applied, since star_fs_apply_depth defers while a port is down.
 */
int star_fs_clone_port(star_state_t *st, int src, int dst)
{
    i6_vpe_port attr;
    star_vpe_port_t *s;
    star_vpe_port_t *d;
    int ret;

    if (!st || src < 0 || src >= STAR_VPE_PORT_NUM || dst < 0 || dst >= STAR_VPE_PORT_NUM)
        return RSS_ERR_INVAL;
    if (src == dst)
        return RSS_ERR_INVAL;
    if (!st->vpe_chn_started)
        return RSS_ERR_NOENT;

    s = &st->port[src];
    d = &st->port[dst];

    if (!s->configured) {
        HAL_LOG_ERR("vpe port %d: cannot clone from port %d, which is not configured", dst, src);
        return RSS_ERR_NOENT;
    }
    if (d->configured) {
        HAL_LOG_ERR("vpe port %d: already configured, refusing to clone port %d over it", dst, src);
        return RSS_ERR_BUSY;
    }

    memset(&attr, 0, sizeof(attr));
    attr.output.width = s->width;
    attr.output.height = s->height;
    /* Orientation is applied at the sensor, so both ports already see a
     * correctly oriented picture -- see star_fs_configure. */
    attr.mirror = 0;
    attr.flip = 0;
    attr.pixFmt = s->pixFmt;
    attr.compress = I6_COMPR_NONE;

    ret = st->vpe.fnSetPortConfig(STAR_VPE_CHN, dst, &attr);
    if (ret) {
        HAL_LOG_ERR("MI_VPE_SetPortMode(%d, %d) %ux%u pixFmt %d failed: %d", STAR_VPE_CHN, dst,
                    s->width, s->height, s->pixFmt, ret);
        return RSS_ERR_IO;
    }

    d->configured = true;
    d->width = s->width;
    d->height = s->height;
    d->pixFmt = s->pixFmt;
    /*
     * The *source* rate, not the snapshot rate. This is what the VPE
     * channel actually produces, and star_enc_bind_port reads it as the
     * bind's srcFps; the reduction to the JPEG stream's own rate is the
     * bind's dstFps, which is where MI does the dropping.
     */
    d->fps_num = s->fps_num;
    d->fps_den = s->fps_den;
    /* Nothing calls fs_get_frame on this port -- it feeds VENC directly --
     * so the user depth stays 0, which is also MI's "no user holds
     * buffers" value. */
    d->user_depth = 0;
    d->queue_depth = STAR_VPE_SNAP_QUEUE_DEPTH;

    ret = st->vpe.fnEnablePort(STAR_VPE_CHN, dst);
    if (ret) {
        HAL_LOG_ERR("MI_VPE_EnablePort(%d, %d) failed: %d", STAR_VPE_CHN, dst, ret);
        d->configured = false;
        return RSS_ERR_IO;
    }
    d->enabled = true;

    HAL_LOG_INFO("vpe port %d: snapshot port cloned from port %d, %ux%u pixFmt %d, queue depth %u",
                 dst, src, d->width, d->height, d->pixFmt, d->queue_depth);

    return star_fs_apply_depth(st, dst, d);
}

/*
 * Undo star_fs_clone_port. Deliberately quiet about a port that is
 * already down: this runs on the failure path of the bind that follows
 * the clone as well as on teardown.
 */
void star_fs_release_port(star_state_t *st, int port)
{
    star_vpe_port_t *p;
    int ret;

    if (!st || port < 0 || port >= STAR_VPE_PORT_NUM)
        return;

    p = &st->port[port];
    if (!p->configured && !p->enabled)
        return;

    star_fs_drop_frame(st, p);
    star_fs_close_fd(st, p);

    if (p->enabled) {
        ret = st->vpe.fnDisablePort(STAR_VPE_CHN, port);
        if (ret)
            HAL_LOG_WARN("MI_VPE_DisablePort(%d, %d) failed: %d", STAR_VPE_CHN, port, ret);
        p->enabled = false;

        /*
         * Same last-port-down rule as hal_fs_disable_channel, and it has
         * to be here too rather than only there: rvd tears a stream down
         * encoder-first, so a snapshot port is released *after* the video
         * ports it was cloned from are already disabled. Without this the
         * final port could go down leaving the tuning latched live while
         * the VPE channel -- and with it CUS3A -- has actually stopped,
         * and the next bring-up would trust a latch that outlived what it
         * described.
         */
        if (!star_fs_any_port_enabled(st))
            star_isp_untune(st);
    }

    p->configured = false;
    p->width = 0;
    p->height = 0;
    p->user_depth = 0;
    p->queue_depth = 0;
}

/* ================================================================
 * DEPTHS
 *
 * No MI getter exists for either depth, so the getters report what was
 * last set. Saying so is better than returning a plausible constant.
 * ================================================================ */

int hal_fs_set_frame_depth(void *ctx, int chn, int depth)
{
    STAR_FS_ENTER(ctx, chn, st, port);

    if (depth < 0)
        return RSS_ERR_INVAL;

    port->user_depth = (unsigned int)depth;
    /*
     * The queue has to be able to hold what the user may hold. The
     * vendor's own sample pairs (2, 3); keep that shape when a caller
     * asks for a user depth the configured queue cannot cover.
     */
    if (port->queue_depth < port->user_depth + 1)
        port->queue_depth = port->user_depth + 1;

    return star_fs_apply_depth(st, chn, port);
}

int hal_fs_get_frame_depth(void *ctx, int chn, int *depth)
{
    STAR_FS_ENTER(ctx, chn, st, port);
    (void)st;

    if (!depth)
        return RSS_ERR_INVAL;

    *depth = (int)port->user_depth;
    return RSS_OK;
}

/*
 * fs_set_fifo is IMP's per-channel FIFO depth. MI's nearest equivalent
 * is the same call's u32BufQueueDepth, so that is what this sets.
 * depth <= 0 means "SDK default" in rvd's usage (rvd_pipeline.c:873
 * passes 0), which here is the vendor sample's 3.
 */
int hal_fs_set_fifo(void *ctx, int chn, int depth)
{
    STAR_FS_ENTER(ctx, chn, st, port);

    port->queue_depth = depth > 0 ? (unsigned int)depth : STAR_VPE_QUEUE_DEPTH;
    if (port->queue_depth < port->user_depth + 1)
        port->queue_depth = port->user_depth + 1;

    return star_fs_apply_depth(st, chn, port);
}

int hal_fs_get_fifo(void *ctx, int chn, int *depth)
{
    STAR_FS_ENTER(ctx, chn, st, port);
    (void)st;

    if (!depth)
        return RSS_ERR_INVAL;

    *depth = (int)port->queue_depth;
    return RSS_OK;
}

/* ================================================================
 * FRAME FETCH
 * ================================================================ */

/*
 * star_fs_frame_bytes -- how much of the buffer holds this frame.
 *
 * The vendor's MI_SYS_ChnOutputPortGetBuf sample writes exactly
 *   height * stride[0] + height * stride[1] / 2
 * for a semi-planar 4:2:0 frame, so that is the formula used here
 * rather than u32BufSize -- the latter is the allocation, which is
 * padded and would put alignment garbage in a snapshot file.
 *
 * bufSize is still the safety bound: a computed size larger than the
 * allocation means a stride assumption is wrong, and the caller writes
 * this many bytes out of the mapping.
 */
static unsigned int star_fs_frame_bytes(const i6_sys_frame *frame)
{
    unsigned int size;

    switch (frame->pixFmt) {
    case I6_PIXFMT_YUV420SP:
    case I6_PIXFMT_YUV420SP_NV21:
        size = frame->height * frame->stride[0] + frame->height * frame->stride[1] / 2;
        break;
    case I6_PIXFMT_YUV422SP:
        size = frame->height * frame->stride[0] + frame->height * frame->stride[1];
        break;
    default:
        /* Packed and single-plane formats: one stride covers the row. */
        size = frame->height * frame->stride[0];
        break;
    }

    if (frame->bufSize && size > frame->bufSize)
        size = frame->bufSize;

    return size;
}

/*
 * hal_fs_get_frame -- block until a frame is available, then check it out.
 *
 * MI_SYS_ChnOutputPortGetBuf does not block, so the wait is a select on
 * the port's wakeup descriptor -- which is what the vendor recommends
 * ("it is recommended to use the fd s select method to extract the
 * data, so that MI_SYS only wake up the thread when the corresponding
 * port has data", MI_SYS reference 2.4.15) and what both references do.
 * The fd is opened once per port and kept, since MI_SYS_GetFd and
 * MI_SYS_CloseFd must be used in pairs and re-opening per frame would
 * be an ioctl per frame for nothing.
 *
 * One outstanding frame per port. rvd's only frame consumer is the
 * snapshot path, which is strictly get-then-release
 * (rvd_ctrl.c:1805-1821), and a single slot lets frame_data be a stable
 * pointer into the port state instead of a per-frame allocation. A
 * second get without a release is a caller bug, and reported as one.
 *
 * No MI_SYS_FlushInvCache before the CPU read: the vendor sample reads
 * pVirAddr straight after GetBuf, and neither reference invalidates on
 * this path (waybeam's only FlushInvCache calls are on IPU output
 * buffers, not VPE port frames). The symbol is bound in i6_sys.h if a
 * coherency problem ever does show up.
 */
int hal_fs_get_frame(void *ctx, int chn, void **frame_data, rss_frame_info_t *info)
{
    i6_sys_bind bind;
    struct timeval timeout;
    fd_set fds;
    int handle = 0;
    int ret;

    STAR_FS_ENTER(ctx, chn, st, port);

    if (!frame_data || !info)
        return RSS_ERR_INVAL;

    if (!st->sys.fnGetFd || !st->sys.fnGetOutputBuf || !st->sys.fnPutOutputBuf) {
        HAL_LOG_ERR("MI_SYS frame path unavailable (GetFd/GetBuf/PutBuf)");
        return RSS_ERR_NOTSUP;
    }

    if (!port->enabled) {
        HAL_LOG_ERR("fs chn %d: get_frame on a disabled port", chn);
        return RSS_ERR_INVAL;
    }

    /*
     * With a user frame depth of 0 MI hands out no buffers at all --
     * that is what the depth means, and the vendor recommends setting
     * it back to 0 when done "so that the speed of the underlying call
     * is not affected". IMP behaves the same way, so callers already
     * call fs_set_frame_depth first; say which call is missing rather
     * than blocking until the timeout.
     */
    if (port->user_depth == 0) {
        HAL_LOG_ERR("fs chn %d: get_frame needs fs_set_frame_depth(chn, >0) first", chn);
        return RSS_ERR_INVAL;
    }

    if (port->frame_held) {
        HAL_LOG_ERR("fs chn %d: a frame is already checked out", chn);
        return RSS_ERR_BUSY;
    }

    memset(&bind, 0, sizeof(bind));
    bind.module = I6_SYS_MOD_VPE;
    bind.device = STAR_VPE_DEV;
    bind.channel = STAR_VPE_CHN;
    bind.port = chn;

    if (port->fd < 0) {
        ret = st->sys.fnGetFd(&bind, &port->fd);
        if (ret || port->fd < 0) {
            HAL_LOG_ERR("MI_SYS_GetFd(vpe port %d) failed: %d", chn, ret);
            port->fd = -1;
            return RSS_ERR_IO;
        }
    }

    FD_ZERO(&fds);
    FD_SET(port->fd, &fds);
    timeout.tv_sec = STAR_FRAME_TIMEOUT_MS / 1000;
    timeout.tv_usec = (STAR_FRAME_TIMEOUT_MS % 1000) * 1000;

    ret = select(port->fd + 1, &fds, NULL, NULL, &timeout);
    if (ret < 0) {
        HAL_LOG_ERR("fs chn %d: select failed: %s", chn, strerror(errno));
        return RSS_ERR_IO;
    }
    if (ret == 0) {
        HAL_LOG_WARN("fs chn %d: no frame within %d ms", chn, STAR_FRAME_TIMEOUT_MS);
        return RSS_ERR_TIMEOUT;
    }

    memset(&port->frame, 0, sizeof(port->frame));
    ret = st->sys.fnGetOutputBuf(&bind, &port->frame, &handle);
    if (ret) {
        HAL_LOG_ERR("MI_SYS_ChnOutputPortGetBuf(vpe port %d) failed: %d", chn, ret);
        return RSS_ERR_IO;
    }

    if (port->frame.bufType != I6_SYS_BUFDATA_FRAME) {
        HAL_LOG_ERR("fs chn %d: buffer type %d is not a frame", chn, port->frame.bufType);
        (void)st->sys.fnPutOutputBuf(handle);
        return RSS_ERR_IO;
    }

    port->frame_held = true;
    port->frame_handle = handle;

    info->width = port->frame.frame.width;
    info->height = port->frame.frame.height;
    info->pixfmt = star_fs_pixfmt_rev(port->frame.frame.pixFmt);
    info->timestamp = (int64_t)port->frame.pts;
    /* rss_frame_info_t's phys_addr is 32-bit; MI reports 64. Infinity6E
     * physical addresses fit, but truncate explicitly so the narrowing
     * is a decision rather than a warning. */
    info->phys_addr = (uint32_t)port->frame.frame.phyAddr[0];
    info->virt_addr = port->frame.frame.virAddr[0];
    info->size = star_fs_frame_bytes(&port->frame.frame);

    *frame_data = &port->frame;
    return RSS_OK;
}

int hal_fs_release_frame(void *ctx, int chn, void *frame_data)
{
    int ret;

    STAR_FS_ENTER(ctx, chn, st, port);

    if (!port->frame_held) {
        HAL_LOG_ERR("fs chn %d: release with no frame checked out", chn);
        return RSS_ERR_INVAL;
    }
    /*
     * frame_data is the pointer get_frame handed out. Checking identity
     * catches a channel mix-up, which would otherwise return one port's
     * buffer against another's handle.
     */
    if (frame_data != &port->frame) {
        HAL_LOG_ERR("fs chn %d: release with a foreign frame pointer", chn);
        return RSS_ERR_INVAL;
    }

    ret = st->sys.fnPutOutputBuf(port->frame_handle);
    port->frame_held = false;
    port->frame_handle = 0;
    if (ret) {
        HAL_LOG_ERR("MI_SYS_ChnOutputPortPutBuf(vpe port %d) failed: %d", chn, ret);
        return RSS_ERR_IO;
    }

    return RSS_OK;
}
