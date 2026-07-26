/*
 * hal_osd.c -- OSD/overlay ops for Infinity6E, over MI_RGN.
 *
 * THE POINT OF THIS FILE
 *
 * Two things, one of which is not obvious. The obvious one is drawing:
 * rvd renders text and graphics into BGRA bitmaps and this file hands
 * them to MI as regions attached to the VPE output port feeding each
 * encoder. The other is that **rvd cannot start at all on this backend
 * without it**: rvd builds its bind chain as FS [-> IVS] [-> OSD] -> ENC
 * on the strength of `[osd] enabled` alone -- not on caps, not on
 * whether the OSD ops exist -- and until now hal_bind rejected anything
 * that was not exactly FS -> ENC, so the rejected bind took the whole
 * pipeline down. That is why `[osd] enabled = false` was pinned in the
 * board config from phase 2 onward. This file removes that requirement:
 * see hal_bind in hal_common.c, which now collapses the OSD stage.
 *
 * HOW raptor's MODEL MAPS ONTO MI's
 *
 * raptor (following Ingenic) has region *groups*: a region is created,
 * registered into a group, and the group is bound into the pipeline as a
 * stage. MI has none of that. A region is created globally, then
 * *attached* to a channel port, and the overlay is composited by that
 * port's hardware. So:
 *
 *   group           -> the encoder channel the region should appear on.
 *                      A flag plus a set of registered regions; no MI
 *                      object exists.
 *   register        -> attach the region to the VPE output port bound to
 *                      that encoder channel.
 *   group bind      -> nothing. The real FS -> ENC bind is unchanged.
 *   osd_start/stop  -> nothing. MI has no group scheduling.
 *
 * WHY ATTACH IS DEFERRED
 *
 * rvd creates and registers every region *before* it binds the chain and
 * before the framesource is enabled (rvd_pipeline.c: osd_create_group ->
 * rvd_osd_init_stream -> osd_start, then the bind loop). At that point
 * the VPE port a region must attach to is not merely disabled, it is not
 * yet known -- rvd may bind any framesource channel to any encoder
 * channel, and the backend records the pair only when the bind happens.
 * So register_region records the intent and star_osd_flush_pending
 * performs the attach from star_enc_bind_port once the port is known and
 * live. This is the same shape as phase 3's deferred ISP tuning, for the
 * same underlying reason: rvd's call order is built around Ingenic's
 * object lifetimes, not MI's.
 *
 * PIXEL FORMAT
 *
 * rvd renders BGRA8888 and says so in rss_osd_region_t. MI's accepted
 * OSD formats are not knowable statically (see i6_rgn.h TRAP 2), so
 * star_osd_probe_pixfmt asks the driver once, at the first region
 * create, preferring the format that needs no conversion:
 *
 *   ARGB8888  a straight copy -- BGRA in memory *is* 0xAARRGGBB as a
 *             little-endian word, which is what MI calls ARGB.
 *   ARGB4444  16 alpha levels, adequate for antialiased text.
 *   ARGB1555  1-bit alpha, what divinus uses. Antialiased edges become
 *             hard, so this is the last choice rather than the first.
 *
 * The chosen format is logged. If a board turns out to reject 8888,
 * that shows up as one INFO line rather than as a mystery.
 *
 * OP COVERAGE
 *
 * Twelve ops, which is exactly what rvd calls. Deliberately absent:
 *
 *   osd_show                   rvd never calls it; osd_show_region
 *                              carries the same information plus the
 *                              layer, and one code path is better than
 *                              two that can disagree.
 *   osd_attach_to_group        an Ingenic re-attach primitive rvd does
 *                              not use. register_region covers it.
 *   osd_get_region_attr        unused by rvd, and misleading to
 *   osd_get_group_region_attr  implement: the values raptor asks for live
 *                              partly in the region attr and partly in a
 *                              per-channel display attr that does not
 *                              exist until the region is attached.
 *   osd_set_region_attr_with_timestamp
 *                              Ingenic's timestamped variant; MI has no
 *                              equivalent and rvd never calls it.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "star_state.h"

#include <stdlib.h>
#include <string.h>

/*
 * No HAL_MODULE_VIDEO guard, deliberately: the Makefile passes that flag
 * only to src/hal_common_{video,audio}.o, so a guard here would compile
 * the file away to nothing and the ops table in hal_common.c would fail
 * to link. Membership in VIDEO_SRCS is what keeps this out of the audio
 * archive.
 */

/* Bytes per pixel of the formats this file will use. */
static unsigned int star_osd_bpp(i6_rgn_pixfmt fmt)
{
    return fmt == I6_RGN_PIXFMT_ARGB888 ? 4 : 2;
}

static const char *star_osd_fmt_name(i6_rgn_pixfmt fmt)
{
    switch (fmt) {
    case I6_RGN_PIXFMT_ARGB888:
        return "ARGB8888";
    case I6_RGN_PIXFMT_ARGB4444:
        return "ARGB4444";
    case I6_RGN_PIXFMT_ARGB1555:
        return "ARGB1555";
    default:
        return "?";
    }
}

static star_osd_region_t *star_osd_region(star_state_t *st, int handle)
{
    if (!st || handle < 0 || handle >= STAR_OSD_REGION_MAX)
        return NULL;
    if (!st->osd[handle].used)
        return NULL;

    return &st->osd[handle];
}

/*
 * The RGN port a region attaches to for a given group.
 *
 * Groups are encoder channels; the port is whichever VPE output port was
 * bound to that encoder. Returns false when the bind has not happened
 * yet, which is the normal case during rvd's OSD setup and the reason
 * attach is deferred rather than failed.
 */
static bool star_osd_port_for_group(star_state_t *st, int grp, i6_sys_bind *port)
{
    if (!st || grp < 0 || grp >= I6_VENC_CHN_NUM)
        return false;
    if (!st->enc[grp].bound || st->enc[grp].src_port < 0)
        return false;

    memset(port, 0, sizeof(*port));
    /*
     * RGN's private module enum, not i6_sys_mod -- see i6_rgn.h TRAP 1.
     * The cast is the point rather than a wart: the struct field is
     * declared i6_sys_mod because every other MI_SYS call means that, and
     * -Werror=enum-conversion is right to notice that this one does not.
     */
    port->module = (i6_sys_mod)I6_RGN_MOD_VPE;
    port->device = STAR_VPE_DEV;
    port->channel = STAR_VPE_CHN;
    port->port = (unsigned int)st->enc[grp].src_port;

    return true;
}

/* Fill the per-channel display attr MI wants from a tracked region. */
static void star_osd_fill_chn(const star_osd_region_t *r, i6_rgn_chn *chn)
{
    memset(chn, 0, sizeof(*chn));
    chn->show = r->show ? 1 : 0;
    chn->point.x = (unsigned int)r->x;
    chn->point.y = (unsigned int)r->y;

    if (r->type == RSS_OSD_COVER) {
        chn->cover.layer = (unsigned int)r->layer;
        chn->cover.size.width = (unsigned int)r->width;
        chn->cover.size.height = (unsigned int)r->height;
        chn->cover.color = r->cover_color;
        return;
    }

    chn->osd.layer = (unsigned int)r->layer;
    /*
     * ALPHA: constAlphaOn STAYS 0. Board-verified 2026-07-26 -- setting
     * it is what made every overlay invisible.
     *
     * bgFgAlpha is {background, foreground}: the alphas the hardware
     * applies to a pixel according to its own alpha channel, so the
     * bitmap's per-pixel transparency survives and antialiased glyph
     * edges blend. constAlphaOn = 1 switches that off and gives the whole
     * *rectangle* one alpha from constAlpha -- and since it shares the
     * union with bgFgAlpha, constAlpha[0] is whatever bg_alpha was. rvd
     * sends bg_alpha = 0 for every text region, so const-alpha mode
     * painted each region with alpha 0: regions created, attached, shown,
     * and perfectly transparent. Nothing in the log, because the driver
     * had been told exactly what to do.
     *
     * So global_alpha_en cannot be mapped onto constAlphaOn even though
     * the names line up. On Ingenic, global alpha *modulates* the
     * per-pixel channel; MI's const alpha *replaces* it. The honest
     * mapping of "modulate" onto MI is the bgFgAlpha path, which is also
     * the only configuration either reference uses (divinus:
     * constAlphaOn = 0, bgFgAlpha = {0, opacity}). A caller that really
     * wants a uniform-alpha rectangle wants a COVER region, which is the
     * branch above.
     */
    chn->osd.constAlphaOn = 0;
    chn->osd.bgFgAlpha[0] = r->bg_alpha;
    chn->osd.bgFgAlpha[1] = r->fg_alpha;
}

/*
 * Ask the driver which pixel format it will accept, once.
 *
 * Done by creating and immediately destroying a 2x2 region rather than
 * by consulting a table, because there is no table to consult: the
 * accepted set is decided in mi_rgn.ko and the two references only
 * demonstrate ARGB1555 and I4. Preference order is
 * cheapest-conversion-first; see the PIXEL FORMAT note above.
 *
 * A probe handle above every real one keeps this from colliding with a
 * region rvd is about to create.
 *
 * RSS_OSD_PIXFMT restricts the probe to one format, for bring-up on a
 * board where "the driver accepts it" and "it composites correctly" have
 * turned out to be different questions. It only narrows the list -- the
 * driver still has to accept the choice -- so it cannot force a format
 * mi_rgn.ko rejects. It exists because ARGB4444 is what this chip
 * accepts while ARGB1555 is the only format either reference has ever
 * been seen to display, and settling that on hardware should not need a
 * rebuild.
 */
static int star_osd_probe_pixfmt(star_state_t *st)
{
    static const i6_rgn_pixfmt tries[] = {I6_RGN_PIXFMT_ARGB888, I6_RGN_PIXFMT_ARGB4444,
                                          I6_RGN_PIXFMT_ARGB1555};
    const unsigned int probe_handle = STAR_OSD_REGION_MAX;
    const char *want = getenv("RSS_OSD_PIXFMT");
    unsigned int i;

    if (st->rgn_fmt_known)
        return RSS_OK;

    for (i = 0; i < sizeof(tries) / sizeof(tries[0]); i++) {
        i6_rgn_cnf cnf;
        int ret;

        if (want && want[0] && strcmp(want, star_osd_fmt_name(tries[i])) != 0)
            continue;

        memset(&cnf, 0, sizeof(cnf));
        cnf.type = I6_RGN_TYPE_OSD;
        cnf.pixFmt = tries[i];
        cnf.size.width = 2;
        cnf.size.height = 2;

        ret = st->rgn.fnCreateRegion(probe_handle, &cnf);
        if (ret) {
            /*
             * INFO, not DBG: a rejected probe makes mi_rgn.ko print
             * "<<<MI_RGN_IMPL_Create[...] Check osd attr error!" to the
             * kernel log at KERN_ERR, in red. That message is expected --
             * asking is the whole point of a probe -- but with this line
             * compiled out (HAL_LOG_DBG needs HAL_DEBUG) the kernel error
             * appeared in logread with nothing in userspace to explain it,
             * which cost real time to chase. One INFO line per rejected
             * format, once per boot, is worth it.
             */
            HAL_LOG_INFO("osd: %s rejected by MI_RGN_Create: %#x "
                         "(the kernel's \"Check osd attr error\" is this probe)",
                         star_osd_fmt_name(tries[i]), (unsigned int)ret);
            continue;
        }

        st->rgn.fnDestroyRegion(probe_handle);
        st->rgn_fmt = tries[i];
        st->rgn_fmt_known = true;
        HAL_LOG_INFO("osd: using %s%s%s", star_osd_fmt_name(tries[i]),
                     tries[i] == I6_RGN_PIXFMT_ARGB888 ? " (no conversion needed)" : "",
                     want && want[0] ? " (RSS_OSD_PIXFMT)" : "");
        return RSS_OK;
    }

    if (want && want[0])
        HAL_LOG_ERR("osd: RSS_OSD_PIXFMT=%s matched no probeable format, or MI_RGN_Create "
                    "rejected it (try ARGB8888, ARGB4444 or ARGB1555)",
                    want);
    else
        HAL_LOG_ERR("osd: MI_RGN_Create rejected every pixel format tried "
                    "(ARGB8888, ARGB4444, ARGB1555)");
    return RSS_ERR_NOTSUP;
}

/* Bring MI_RGN up on first use. */
static int star_osd_ensure_init(star_state_t *st)
{
    int ret;

    if (!st->rgn_loaded) {
        ret = i6_rgn_load(&st->rgn);
        if (ret) {
            /* i6_rgn_load logs which symbol or library was missing. */
            return ret;
        }
        st->rgn_loaded = true;
    }

    if (!st->rgn_inited) {
        /*
         * The palette only matters for the I2/I4/I8 formats, which this
         * backend never selects -- rvd renders true colour. MI still
         * wants the argument, so pass a zeroed one rather than NULL,
         * matching both references.
         */
        i6_rgn_pal pal;

        memset(&pal, 0, sizeof(pal));
        ret = st->rgn.fnInit(&pal);
        if (ret) {
            HAL_LOG_ERR("MI_RGN_Init failed: %#x", (unsigned int)ret);
            return RSS_ERR_IO;
        }
        st->rgn_inited = true;
    }

    return RSS_OK;
}

/* Attach one region to its group's port, if that port is live yet. */
static int star_osd_try_attach(star_state_t *st, int handle, star_osd_region_t *r)
{
    i6_sys_bind port;
    i6_rgn_chn chn;
    int ret;

    if (r->attached || r->grp < 0)
        return RSS_OK;
    if (!star_osd_port_for_group(st, r->grp, &port))
        return RSS_OK; /* Deferred, not failed. */

    star_osd_fill_chn(r, &chn);

    ret = st->rgn.fnAttachChannel((unsigned int)handle, &port, &chn);
    if (ret) {
        HAL_LOG_WARN("MI_RGN_AttachToChn(region %d, VPE port %u) failed: %#x", handle, port.port,
                     (unsigned int)ret);
        return RSS_ERR_IO;
    }

    r->attached = true;
    /*
     * INFO for the same reason as the probe rejection: this is the one
     * line that says an overlay reached the compositor at all, it fires
     * once per region rather than per frame, and its absence from a
     * release build is indistinguishable from a silent deferral in
     * star_osd_port_for_group.
     */
    HAL_LOG_INFO("osd: region %d attached to VPE port %u (group %d), layer %d, alpha bg/fg %u/%u",
                 handle, port.port, r->grp, r->layer, r->bg_alpha, r->fg_alpha);

    return RSS_OK;
}

static void star_osd_detach(star_state_t *st, int handle, star_osd_region_t *r)
{
    i6_sys_bind port;

    if (!r->attached)
        return;

    if (star_osd_port_for_group(st, r->grp, &port))
        st->rgn.fnDetachChannel((unsigned int)handle, &port);

    r->attached = false;
}

/*
 * Push the tracked display attr to MI for an attached region.
 *
 * MI_RGN_SetDisplayAttr covers position, layer, alpha and show, so every
 * runtime change rvd makes except geometry lands here.
 */
static int star_osd_push_chn(star_state_t *st, int handle, star_osd_region_t *r)
{
    i6_sys_bind port;
    i6_rgn_chn chn;
    int ret;

    if (!r->attached)
        return RSS_OK;
    if (!star_osd_port_for_group(st, r->grp, &port))
        return RSS_OK;

    star_osd_fill_chn(r, &chn);

    ret = st->rgn.fnSetChannelConfig((unsigned int)handle, &port, &chn);
    if (ret) {
        HAL_LOG_WARN("MI_RGN_SetDisplayAttr(region %d) failed: %#x", handle, (unsigned int)ret);
        return RSS_ERR_IO;
    }

    return RSS_OK;
}

/* Create the MI region behind a tracked slot. */
static int star_osd_create_mi(star_state_t *st, int handle, star_osd_region_t *r)
{
    i6_rgn_cnf cnf;
    int ret;

    memset(&cnf, 0, sizeof(cnf));
    cnf.type = r->type == RSS_OSD_COVER ? I6_RGN_TYPE_COVER : I6_RGN_TYPE_OSD;
    cnf.pixFmt = st->rgn_fmt;
    cnf.size.width = (unsigned int)r->width;
    cnf.size.height = (unsigned int)r->height;

    ret = st->rgn.fnCreateRegion((unsigned int)handle, &cnf);
    if (ret) {
        HAL_LOG_ERR("MI_RGN_Create(region %d, %dx%d, %s) failed: %#x", handle, r->width, r->height,
                    star_osd_fmt_name(st->rgn_fmt), (unsigned int)ret);
        return RSS_ERR_IO;
    }

    return RSS_OK;
}

/*
 * Attach every region whose group is this encoder channel.
 *
 * Called from star_enc_bind_port once the VPE port -> VENC bind exists,
 * which is the first moment an attach can succeed. Failures are logged
 * by star_osd_try_attach and deliberately not propagated: a region that
 * will not attach costs an overlay, and taking the stream down over it
 * would be worse than the missing text.
 */
void star_osd_flush_pending(star_state_t *st, int chn)
{
    int i;

    if (!st || !st->rgn_inited)
        return;

    for (i = 0; i < STAR_OSD_REGION_MAX; i++) {
        if (st->osd[i].used && st->osd[i].grp == chn)
            star_osd_try_attach(st, i, &st->osd[i]);
    }
}

/*
 * Release everything OSD-side. Called from hal_deinit.
 *
 * Order matters: MI_RGN_DeInit with regions still attached leaves the
 * driver holding references, so detach, destroy, then deinit.
 */
void star_osd_release_all(star_state_t *st)
{
    int i;

    if (!st || !st->rgn_loaded)
        return;

    for (i = 0; i < STAR_OSD_REGION_MAX; i++) {
        star_osd_region_t *r = &st->osd[i];

        if (!r->used)
            continue;

        star_osd_detach(st, i, r);
        st->rgn.fnDestroyRegion((unsigned int)i);
        free(r->bmp);
        memset(r, 0, sizeof(*r));
    }

    if (st->rgn_inited) {
        st->rgn.fnDeinit();
        st->rgn_inited = false;
    }

    i6_rgn_unload(&st->rgn);
    st->rgn_loaded = false;
    st->rgn_fmt_known = false;
}

/* ---- ops ------------------------------------------------------------- */

/*
 * MI allocates a region's memory when the region is created, out of its
 * own pool, and exposes no size control -- there is nothing to set. The
 * request is reported once and accepted rather than refused: rvd asks
 * for a pool because Ingenic needs one, and answering NOTSUP would make
 * rvd's osd-restart path log a failure for a step that was never
 * necessary here.
 */
int hal_osd_set_pool_size(void *ctx, uint32_t bytes)
{
    star_state_t *st = star_state(ctx);

    (void)bytes; /* Only reaches the log line, which DEBUG=0 compiles out. */

    if (!st)
        return RSS_ERR_INVAL;

    HAL_LOG_DBG("osd: pool size %u ignored -- MI allocates per region", bytes);

    return RSS_OK;
}

int hal_osd_create_group(void *ctx, int grp)
{
    star_state_t *st = star_state(ctx);
    int ret;

    if (!st)
        return RSS_ERR_INVAL;
    if (grp < 0 || grp >= I6_VENC_CHN_NUM) {
        HAL_LOG_ERR("osd: group %d out of range (0..%d)", grp, I6_VENC_CHN_NUM - 1);
        return RSS_ERR_INVAL;
    }

    ret = star_osd_ensure_init(st);
    if (ret)
        return ret;

    st->osd_grp[grp] = true;

    return RSS_OK;
}

int hal_osd_destroy_group(void *ctx, int grp)
{
    star_state_t *st = star_state(ctx);
    int i;

    if (!st)
        return RSS_ERR_INVAL;
    if (grp < 0 || grp >= I6_VENC_CHN_NUM)
        return RSS_ERR_INVAL;

    /* Regions outlive the group on this backend -- they are global MI
     * objects -- so destroying a group detaches its members and leaves
     * them created, which is what rvd expects when it tears one stream
     * down and leaves another running. */
    for (i = 0; i < STAR_OSD_REGION_MAX; i++) {
        if (st->osd[i].used && st->osd[i].grp == grp)
            star_osd_detach(st, i, &st->osd[i]);
    }

    st->osd_grp[grp] = false;

    return RSS_OK;
}

/*
 * No MI equivalent: there is no group object to start or stop, and
 * whether an overlay is composited is the per-region `show` flag that
 * osd_show_region already owns. Accepted as a no-op rather than
 * refused, because rvd calls both unconditionally around every stream.
 */
int hal_osd_start(void *ctx, int grp)
{
    (void)grp;

    return star_state(ctx) ? RSS_OK : RSS_ERR_INVAL;
}

int hal_osd_stop(void *ctx, int grp)
{
    (void)grp;

    return star_state(ctx) ? RSS_OK : RSS_ERR_INVAL;
}

int hal_osd_create_region(void *ctx, int *handle, const rss_osd_region_t *attr)
{
    star_state_t *st = star_state(ctx);
    star_osd_region_t *r;
    int slot;
    int ret;

    if (!st || !handle || !attr)
        return RSS_ERR_INVAL;
    if (attr->width <= 0 || attr->height <= 0) {
        HAL_LOG_ERR("osd: region geometry %dx%d is empty", attr->width, attr->height);
        return RSS_ERR_INVAL;
    }

    ret = star_osd_ensure_init(st);
    if (ret)
        return ret;

    ret = star_osd_probe_pixfmt(st);
    if (ret)
        return ret;

    for (slot = 0; slot < STAR_OSD_REGION_MAX; slot++) {
        if (!st->osd[slot].used)
            break;
    }
    if (slot == STAR_OSD_REGION_MAX) {
        HAL_LOG_ERR("osd: all %d region slots in use", STAR_OSD_REGION_MAX);
        return RSS_ERR_NOMEM;
    }

    r = &st->osd[slot];
    memset(r, 0, sizeof(*r));
    r->type = attr->type;
    r->x = attr->x;
    r->y = attr->y;
    r->width = attr->width;
    r->height = attr->height;
    r->layer = attr->layer;
    r->global_alpha_en = attr->global_alpha_en;
    r->fg_alpha = attr->fg_alpha;
    r->bg_alpha = attr->bg_alpha;
    r->cover_color = attr->cover_color;
    r->grp = -1;
    /* rvd creates regions hidden and shows them once there is something
     * to draw; osd_show_region is what flips this. */
    r->show = false;

    ret = star_osd_create_mi(st, slot, r);
    if (ret) {
        memset(r, 0, sizeof(*r));
        return ret;
    }

    r->used = true;
    *handle = slot;

    HAL_LOG_DBG("osd: region %d created, %dx%d at %d,%d layer %d", slot, r->width, r->height, r->x,
                r->y, r->layer);

    return RSS_OK;
}

int hal_osd_destroy_region(void *ctx, int handle)
{
    star_state_t *st = star_state(ctx);
    star_osd_region_t *r = star_osd_region(st, handle);
    int ret;

    if (!st)
        return RSS_ERR_INVAL;
    if (!r)
        return RSS_ERR_NOENT;

    star_osd_detach(st, handle, r);

    ret = st->rgn.fnDestroyRegion((unsigned int)handle);
    if (ret)
        HAL_LOG_WARN("MI_RGN_Destroy(region %d) failed: %#x", handle, (unsigned int)ret);

    free(r->bmp);
    memset(r, 0, sizeof(*r));

    return ret ? RSS_ERR_IO : RSS_OK;
}

int hal_osd_register_region(void *ctx, int handle, int grp)
{
    star_state_t *st = star_state(ctx);
    star_osd_region_t *r = star_osd_region(st, handle);

    if (!st)
        return RSS_ERR_INVAL;
    if (!r)
        return RSS_ERR_NOENT;
    if (grp < 0 || grp >= I6_VENC_CHN_NUM)
        return RSS_ERR_INVAL;

    if (r->grp >= 0 && r->grp != grp) {
        /* One region, one port. MI would accept the same handle on two
         * ports, but raptor's model has a region belonging to a group,
         * and silently leaving it on the old one would be worse than
         * saying so. */
        HAL_LOG_WARN("osd: region %d moving from group %d to %d", handle, r->grp, grp);
        star_osd_detach(st, handle, r);
    }

    r->grp = grp;

    /* Succeeds now if the stream is already running (a runtime region),
     * defers to star_osd_flush_pending if this is startup. */
    return star_osd_try_attach(st, handle, r);
}

int hal_osd_unregister_region(void *ctx, int handle, int grp)
{
    star_state_t *st = star_state(ctx);
    star_osd_region_t *r = star_osd_region(st, handle);

    if (!st)
        return RSS_ERR_INVAL;
    if (!r)
        return RSS_ERR_NOENT;
    if (grp >= 0 && r->grp != grp)
        return RSS_ERR_INVAL;

    star_osd_detach(st, handle, r);
    r->grp = -1;

    return RSS_OK;
}

/*
 * Geometry, position, alpha and layer in one call.
 *
 * A size change cannot be applied in place -- MI fixes a region's
 * dimensions at create time -- so it becomes destroy + create +
 * re-attach, which is what divinus does when it notices the same
 * mismatch. Everything else is a display-attr push.
 */
int hal_osd_set_region_attr(void *ctx, int handle, const rss_osd_region_t *attr)
{
    star_state_t *st = star_state(ctx);
    star_osd_region_t *r = star_osd_region(st, handle);
    bool resized;
    int grp;
    int ret;

    if (!st || !attr)
        return RSS_ERR_INVAL;
    if (!r)
        return RSS_ERR_NOENT;
    if (attr->width <= 0 || attr->height <= 0)
        return RSS_ERR_INVAL;

    resized = attr->width != r->width || attr->height != r->height || attr->type != r->type;

    r->x = attr->x;
    r->y = attr->y;
    r->layer = attr->layer;
    r->global_alpha_en = attr->global_alpha_en;
    r->fg_alpha = attr->fg_alpha;
    r->bg_alpha = attr->bg_alpha;
    r->cover_color = attr->cover_color;

    if (!resized)
        return star_osd_push_chn(st, handle, r);

    grp = r->grp;
    star_osd_detach(st, handle, r);
    st->rgn.fnDestroyRegion((unsigned int)handle);

    r->type = attr->type;
    r->width = attr->width;
    r->height = attr->height;

    /* The old bitmap is the wrong size now; the next update reallocates. */
    free(r->bmp);
    r->bmp = NULL;
    r->bmp_size = 0;
    r->bmp_logged = false;

    ret = star_osd_create_mi(st, handle, r);
    if (ret) {
        /* The slot no longer has an MI region behind it, so stop
         * claiming it does. */
        free(r->bmp);
        memset(r, 0, sizeof(*r));
        return ret;
    }

    r->grp = grp;

    return star_osd_try_attach(st, handle, r);
}

/*
 * BGRA8888 from rvd -> whatever MI accepted.
 *
 * rvd's "BGRA, width*height*4" is byte order in memory, so as a
 * little-endian 32-bit word each pixel is already 0xAARRGGBB -- MI's
 * ARGB8888. That is why the 8888 path is a copy and not a shuffle, and
 * why 8888 is probed first.
 */
static void star_osd_convert(const uint8_t *src, void *dst, int pixels, i6_rgn_pixfmt fmt)
{
    int i;

    if (fmt == I6_RGN_PIXFMT_ARGB888) {
        memcpy(dst, src, (size_t)pixels * 4);
        return;
    }

    if (fmt == I6_RGN_PIXFMT_ARGB4444) {
        uint16_t *out = (uint16_t *)dst;

        for (i = 0; i < pixels; i++) {
            uint8_t b = src[i * 4 + 0];
            uint8_t g = src[i * 4 + 1];
            uint8_t rr = src[i * 4 + 2];
            uint8_t a = src[i * 4 + 3];

            out[i] = (uint16_t)(((a >> 4) << 12) | ((rr >> 4) << 8) | ((g >> 4) << 4) | (b >> 4));
        }
        return;
    }

    /* ARGB1555: alpha collapses to one bit. Anything not fully
     * transparent is drawn, which keeps thin antialiased strokes visible
     * instead of dropping them -- the opposite threshold makes small
     * text disappear. */
    {
        uint16_t *out = (uint16_t *)dst;

        for (i = 0; i < pixels; i++) {
            uint8_t b = src[i * 4 + 0];
            uint8_t g = src[i * 4 + 1];
            uint8_t rr = src[i * 4 + 2];
            uint8_t a = src[i * 4 + 3];

            out[i] = (uint16_t)((a ? 0x8000 : 0) | ((rr >> 3) << 10) | ((g >> 3) << 5) | (b >> 3));
        }
    }
}

int hal_osd_update_region_data(void *ctx, int handle, const uint8_t *data)
{
    star_state_t *st = star_state(ctx);
    star_osd_region_t *r = star_osd_region(st, handle);
    i6_rgn_bmp bmp;
    size_t need;
    int pixels;
    int ret;

    if (!st)
        return RSS_ERR_INVAL;
    if (!r)
        return RSS_ERR_NOENT;
    if (!data) {
        /* rvd's Ingenic sequence sets the attr with a NULL data pointer
         * before it has anything to draw. Nothing to push. */
        return RSS_OK;
    }
    if (r->type == RSS_OSD_COVER) {
        /* A cover is a solid rectangle; its colour is in the display
         * attr, and MI has no bitmap for it. */
        return RSS_ERR_NOTSUP;
    }

    pixels = r->width * r->height;
    need = (size_t)pixels * star_osd_bpp(st->rgn_fmt);

    if (r->bmp_size < need) {
        void *nb = realloc(r->bmp, need);

        if (!nb)
            return RSS_ERR_NOMEM;
        r->bmp = nb;
        r->bmp_size = need;
    }

    star_osd_convert(data, r->bmp, pixels, st->rgn_fmt);

    memset(&bmp, 0, sizeof(bmp));
    bmp.pixFmt = st->rgn_fmt;
    bmp.size.width = (unsigned int)r->width;
    bmp.size.height = (unsigned int)r->height;
    bmp.data = r->bmp;

    ret = st->rgn.fnSetBitmap((unsigned int)handle, &bmp);
    if (ret) {
        HAL_LOG_WARN("MI_RGN_SetBitMap(region %d) failed: %#x", handle, (unsigned int)ret);
        return RSS_ERR_IO;
    }

    if (!r->bmp_logged) {
        r->bmp_logged = true;
        HAL_LOG_INFO("osd: region %d first bitmap accepted, %dx%d %s", handle, r->width, r->height,
                     star_osd_fmt_name(st->rgn_fmt));
    }

    return RSS_OK;
}

/*
 * Show/hide plus z-order.
 *
 * rvd passes the layer here as well as in the region attr, and this is
 * the call it makes per frame when an overlay's visibility changes, so
 * both are recorded and pushed together.
 */
int hal_osd_show_region(void *ctx, int handle, int grp, int show, int layer)
{
    star_state_t *st = star_state(ctx);
    star_osd_region_t *r = star_osd_region(st, handle);

    if (!st)
        return RSS_ERR_INVAL;
    if (!r)
        return RSS_ERR_NOENT;
    if (grp >= 0 && r->grp >= 0 && r->grp != grp)
        return RSS_ERR_INVAL;

    r->show = show ? true : false;
    if (layer >= 0)
        r->layer = layer;

    return star_osd_push_chn(st, handle, r);
}
