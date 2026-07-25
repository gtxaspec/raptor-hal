/*
 * star/hal_common.c -- Raptor HAL common layer, SigmaStar MI backend
 *
 * Counterpart to src/hal_common.c (Ingenic IMP). Provides the factory
 * functions, the ops vtable, and the logging hook for SigmaStar Infinity6E
 * parts.
 *
 * Why a separate translation unit rather than #ifdefs in src/hal_common.c:
 * the existing HAL_OLD_SDK/HAL_NEW_SDK/HAL_IMPVI_SDK conditionals all
 * distinguish *generations of the same vendor SDK*, where the call
 * sequences are near-identical and only struct layouts and enum names
 * differ. MI is a different SDK with a different pipeline model
 * (VIF -> VPE -> VENC channel/port binding rather than
 * FrameSource -> Encoder groups), so sharing a file would mean two
 * disjoint implementations behind mutually exclusive guards rather than
 * one implementation with variations.
 *
 * Current state: skeleton. The vtable deliberately publishes only the ops
 * that are actually implemented. RSS_HAL_CALL() NULL-guards every entry
 * and returns RSS_ERR_NOTSUP for unset ones (see raptor_hal.h), so
 * unimplemented subsystems need no stub functions and no stub files --
 * omitting them from the vtable is the supported way to express
 * "not available on this platform".
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "star_state.h"

#include <stdarg.h>
#include <syslog.h>
#include <unistd.h>

/* ================================================================
 * LOGGING
 *
 * Mirrors src/hal_common.c: log through a function pointer that
 * defaults to stderr, which daemons redirect to syslog at init.
 * ================================================================ */

static const char *hal_level_str[] = {"FTL", "ERR", "WRN", "INF", "DBG"};

static void hal_log_stderr(int level, const char *file, int line, const char *fmt, ...)
{
    if (level < 0)
        level = 0;
    if (level > 4)
        level = 4;
    const char *basename = strrchr(file, '/');
    if (basename)
        file = basename + 1;
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

/* ── Per-SoC capability data (src/hal_caps.c, compiled per platform) ── */

extern const rss_hal_caps_t g_hal_caps;

/* ================================================================
 * MI BACKEND STATE
 *
 * star_state_t, the fixed topology constants and the framesource op
 * declarations live in star/star_state.h, because hal_framesource.c
 * needs the same library handles and sensor descriptors. The state is
 * hung off rss_hal_ctx.platform rather than added to the context
 * struct: hal_internal.h cannot include the i6_*.h headers, because
 * those include hal_internal.h themselves for HAL_LOG_ERR and
 * RSS_ERR_*. `platform` exists for exactly this.
 * ================================================================ */

/*
 * The daemons call rss_hal_get_imp_version() and friends with no
 * context argument -- on Ingenic those map to IMP_* globals. Keep a
 * pointer to the live state so the MI equivalents can answer. Set at
 * init, cleared at deinit; a single HAL context per process is
 * already assumed throughout raptor.
 */
static star_state_t *g_star;

/* ── GPIO / IR-cut (src/hal_gpio.c — plain sysfs, no SDK dependency) ── */

#ifdef HAL_MODULE_VIDEO
int hal_gpio_set(void *ctx, int pin, int value);
int hal_gpio_get(void *ctx, int pin, int *value);
int hal_ircut_set(void *ctx, int state);
#endif

/* ================================================================
 * SYSTEM LIFECYCLE
 * ================================================================ */

/*
 * star_vif_pixfmt -- pixel format for the raw-sensor side of the pipeline.
 *
 * Used for both the VIF port and the VPE channel input, since the same
 * bayer frames cross both and both references derive it identically
 * (divinus i6_hal.c:293 and :306, waybeam star6e_pipeline.c:475 and
 * :523).
 *
 * Both references ignore the plane's own pixFmt for bayer sensors and
 * recompute it as RGB_BAYER + precision * I6_BAYER_END + bayer (divinus
 * i6_hal.c:293, waybeam star6e_pipeline.c:475 -- the same expression in
 * both; waybeam uses it unconditionally, divinus falls back to the reported
 * field only when the plane is not bayer at all). We do the same, because
 * hardware settled which one is right.
 *
 * The GC4653 driver reports pixFmt 41 for a plane it simultaneously describes
 * as bayer GR(1) at 10bpp, where the formula gives 20 + 1*12 + 1 = 33. The
 * tempting reading -- that the vendor's bayer-id stride is 20 rather than the
 * 12 our i6_common_bayer implies, since 41 == 20 + 1*20 + 1 -- is wrong. With
 * 41 programmed, MI's own proc table decoded it back as "I0_10BPP", and I0 is
 * index 9 in this enum: exactly what 41 means under the references' formula
 * (41 - 20 = 21 -> precision 1, bayer 9). So the vendor's stride is 12, our
 * enum's ordering is confirmed correct by MI's own decode, and 41 selects an
 * IR pattern that contradicts the driver's own bayer field.
 *
 * The driver is simply unreliable in this field, which is presumably why
 * neither reference trusts it. Derive whenever the plane describes a real
 * bayer pattern, and fall back to the reported value only when it does not --
 * there the formula is meaningless (a YUV sensor, say) and the reported field
 * is all there is. (divinus tests `bayer > I6_BAYER_END`, which feeds the
 * sentinel itself through the formula; the difference is unreachable for real
 * values, but >= is what the sentence above actually means.)
 */
i6_common_pixfmt star_vif_pixfmt(const i6_snr_plane *plane)
{
    if (plane->bayer >= I6_BAYER_END)
        return plane->pixFmt;

    return (i6_common_pixfmt)(I6_PIXFMT_RGB_BAYER + plane->precision * I6_BAYER_END +
                              plane->bayer);
}

/*
 * star_sensor_bringup -- select a mode, start the sensor, read back what it is.
 *
 * Sequence follows both references: SetPlaneMode -> QueryResCount ->
 * SetRes -> SetFps -> SetOrien -> Enable.
 *
 * Two deliberate choices:
 *
 * Geometry comes from the sensor, not from a constant. hal_init has no
 * geometry in its arguments (rss_sensor_config_t carries I2C and GPIO
 * details only), so the native mode is the only thing it *can* mean, and
 * for GC4653 there is exactly one. That is not the configuration plumbing
 * 2e owns -- choosing *among* modes is -- and it avoids a hardcoded 2560x1440
 * that would silently mislead on any other sensor.
 *
 * The descriptors are read back *after* Enable, unlike divinus. The sensor
 * driver's pCus_sensor_init runs on Enable, and before it does, pad.intfAttr
 * and the plane geometry read back as zero -- confirmed on this board with
 * star_probe, which reported "planes 0, lanes 0" pre-Enable and correct
 * values after. divinus queries beforehand and gets away with it only
 * because the single field it uses from intfAttr, mipi.input, is 0 for this
 * sensor anyway. waybeam queries after Enable (sensor_select.c:485); so do we.
 */
static int star_sensor_bringup(star_state_t *st, int mirror, int flip)
{
    unsigned int count = 0;
    int ret;

    ret = st->snr.fnSetPlaneMode(STAR_SNR_INDEX, 0);
    if (ret) {
        HAL_LOG_ERR("MI_SNR_SetPlaneMode failed: %d", ret);
        return RSS_ERR_IO;
    }

    ret = st->snr.fnGetResolutionCount(STAR_SNR_INDEX, &count);
    if (ret || !count) {
        HAL_LOG_ERR("MI_SNR_QueryResCount failed: %d (count %u) -- is "
                    "sensor_<name>_mipi.ko loaded?",
                    ret, count);
        return RSS_ERR_NOENT;
    }

    /* Native mode. 2e picks among modes; there is only one here. */
    st->res_index = 0;
    ret = st->snr.fnGetResolution(STAR_SNR_INDEX, st->res_index, &st->res);
    if (ret) {
        HAL_LOG_ERR("MI_SNR_GetRes(%u) failed: %d", st->res_index, ret);
        return RSS_ERR_IO;
    }

    ret = st->snr.fnSetResolution(STAR_SNR_INDEX, st->res_index);
    if (ret) {
        HAL_LOG_ERR("MI_SNR_SetRes(%u) failed: %d", st->res_index, ret);
        return RSS_ERR_IO;
    }

    /*
     * Record the rate as well as programming it: MI_SYS_BindChnPort2
     * takes source and destination frame rates, so both the VIF->VPE
     * bind below and 2d's VPE->VENC bind need to know it, and MI offers
     * no way to read back what the sensor is running at.
     */
    if (st->res.maxFps) {
        ret = st->snr.fnSetFramerate(STAR_SNR_INDEX, st->res.maxFps);
        if (ret)
            HAL_LOG_WARN("MI_SNR_SetFps(%u) failed: %d", st->res.maxFps, ret);
        else
            st->fps = st->res.maxFps;
    }

    /* Orientation before Enable, so the driver's init picks it up. */
    ret = st->snr.fnSetOrientation(STAR_SNR_INDEX, mirror ? 1 : 0, flip ? 1 : 0);
    if (ret)
        HAL_LOG_WARN("MI_SNR_SetOrien(%d,%d) failed: %d", mirror, flip, ret);

    ret = st->snr.fnEnable(STAR_SNR_INDEX);
    if (ret) {
        HAL_LOG_ERR("MI_SNR_Enable failed: %d", ret);
        return RSS_ERR_IO;
    }
    st->snr_enabled = true;

    ret = st->snr.fnGetPadInfo(STAR_SNR_INDEX, &st->pad);
    if (ret) {
        HAL_LOG_ERR("MI_SNR_GetPadInfo failed: %d", ret);
        return RSS_ERR_IO;
    }

    /*
     * Plane 0 directly, not a loop over pad.planeCnt: that field reads 0 on
     * this hardware and neither reference ever consults it -- both hardcode
     * index 0. Extra planes exist only for hardware HDR.
     */
    ret = st->snr.fnGetPlaneInfo(STAR_SNR_INDEX, 0, &st->plane);
    if (ret) {
        HAL_LOG_ERR("MI_SNR_GetPlaneInfo(0) failed: %d", ret);
        return RSS_ERR_IO;
    }

    HAL_LOG_INFO("sensor \"%.32s\": %ux%u, %u-%u fps, bayer %d, precision %d, pixFmt %d",
                 st->plane.sensName, st->plane.capt.width, st->plane.capt.height,
                 st->res.minFps, st->res.maxFps, st->plane.bayer, st->plane.precision,
                 st->plane.pixFmt);

    return RSS_OK;
}

/*
 * star_vif_bringup -- configure and enable the VIF device and its port.
 *
 * VIF is the sensor-facing capture block; it has no Ingenic counterpart,
 * since IMP folds this into IMP_ISP_AddSensor. Attributes come from the pad
 * and plane descriptors read back above rather than from constants, so this
 * follows whatever sensor is actually loaded.
 */
static int star_vif_bringup(star_state_t *st)
{
    i6_vif_dev device;
    i6_vif_port port;
    int ret;

    memset(&device, 0, sizeof(device));
    device.intf = st->pad.intf;
    /* RGB_REALTIME is the raw-sensor path; 1MULTIPLEX is for BT656. */
    device.work = device.intf == I6_INTF_BT656 ? I6_VIF_WORK_1MULTIPLEX : I6_VIF_WORK_RGB_REALTIME;
    device.hdr = I6_HDR_OFF;
    if (device.intf == I6_INTF_MIPI) {
        device.edge = I6_EDGE_DOUBLE;
        device.input = st->pad.intfAttr.mipi.input;
    } else if (device.intf == I6_INTF_BT656) {
        device.edge = st->pad.intfAttr.bt656.edge;
        device.sync = st->pad.intfAttr.bt656.sync;
        device.bitswap = (char)st->pad.intfAttr.bt656.bitswap;
    }

    ret = st->vif.fnSetDeviceConfig(STAR_VIF_DEV, &device);
    if (ret) {
        HAL_LOG_ERR("MI_VIF_SetDevAttr failed: %d", ret);
        return RSS_ERR_IO;
    }

    ret = st->vif.fnEnableDevice(STAR_VIF_DEV);
    if (ret) {
        HAL_LOG_ERR("MI_VIF_EnableDev failed: %d", ret);
        return RSS_ERR_IO;
    }
    st->vif_dev_enabled = true;

    memset(&port, 0, sizeof(port));
    port.capt = st->plane.capt;
    port.dest.width = st->plane.capt.width;
    port.dest.height = st->plane.capt.height;
    port.field = 0;
    port.interlaceOn = 0;
    port.pixFmt = star_vif_pixfmt(&st->plane);
    port.frate = I6_VIF_FRATE_FULL;
    port.frameLineCnt = 0;

    ret = st->vif.fnSetPortConfig(STAR_VIF_CHN, STAR_VIF_PORT, &port);
    if (ret) {
        HAL_LOG_ERR("MI_VIF_SetChnPortAttr failed: %d (pixFmt %d)", ret, port.pixFmt);
        return RSS_ERR_IO;
    }

    ret = st->vif.fnEnablePort(STAR_VIF_CHN, STAR_VIF_PORT);
    if (ret) {
        HAL_LOG_ERR("MI_VIF_EnableChnPort failed: %d", ret);
        return RSS_ERR_IO;
    }
    st->vif_port_enabled = true;

    HAL_LOG_INFO("VIF up: dev %d chn %d port %d, %ux%u, pixFmt %d", STAR_VIF_DEV, STAR_VIF_CHN,
                 STAR_VIF_PORT, port.dest.width, port.dest.height, port.pixFmt);

    return RSS_OK;
}

/*
 * star_vpe_bringup -- create the ISP/scaler channel and hook VIF to it.
 *
 * VPE is the ISP plus scaler block: one channel per sensor, with up to
 * four output ports that are raptor's framesource channels (see
 * hal_framesource.c). The channel is created here rather than on the
 * first fs_create_channel because it takes the sensor's geometry and
 * pixel format, and because nothing moves until it is bound to VIF --
 * which is also why 2b could not observe frames at the VIF port.
 *
 * Three things this deliberately does not do:
 *
 * No ports are configured or enabled. divinus binds VIF->VPE with no
 * ports set up (i6_hal.c:355) and configures them later per encoder
 * channel, which is the order raptor needs too: hal_init runs before
 * rvd knows its stream geometry. waybeam configures port 0 first only
 * because it is a single-purpose daemon that already knows it.
 *
 * The i6e_ structs are populated and cast to the shorter declared type.
 * MI_VPE_CreateChannel and MI_VPE_SetChannelParam read a longer struct
 * on Infinity6E than on Infinity6 (the LDC members); divinus branches
 * on series == 0xF1 and casts (i6_hal.c:302-345). Our target is 0xF1
 * only, so the i6e_ variants are always the ones filled in -- passing
 * the short struct would have MI read past its end. See i6_vpe.h.
 *
 * Nothing waits for the ISP. MI_VPE_CreateChannel returns before the
 * ISP channel has finished initialising, so anything touching MI_ISP
 * must poll MI_ISP_IQ_GetParaInitStatus first or the kernel logs
 * "IspApiGet channel not created" (waybeam star6e_pipeline.c:172-200).
 * No MI_ISP call exists in this backend yet; phase 3 needs that poll
 * before its first one.
 */
static int star_vpe_bringup(star_state_t *st)
{
    i6e_vpe_chn channel;
    i6e_vpe_para param;
    i6_sys_bind source, dest;
    unsigned int fps;
    int ret;
    int i;

    /*
     * -1, not the 0 that calloc left behind: 0 is a legitimate file
     * descriptor, so teardown must be able to tell "never opened" from
     * "opened as fd 0" before it calls MI_SYS_CloseFd on every port.
     */
    for (i = 0; i < STAR_VPE_PORT_NUM; i++)
        st->port[i].fd = -1;

    memset(&channel, 0, sizeof(channel));
    channel.capt.width = st->plane.capt.width;
    channel.capt.height = st->plane.capt.height;
    channel.pixFmt = star_vif_pixfmt(&st->plane);
    channel.hdr = I6_HDR_OFF;
    /* i6_vpe_sens is 1-based: ID0 == 1. Both references pass index + 1. */
    channel.sensor = (i6_vpe_sens)(STAR_SNR_INDEX + 1);
    channel.mode = I6_VPE_MODE_REALTIME;

    ret = st->vpe.fnCreateChannel(STAR_VPE_CHN, (i6_vpe_chn *)&channel);
    if (ret) {
        HAL_LOG_ERR("MI_VPE_CreateChannel(%d) failed: %d (%ux%u pixFmt %d)", STAR_VPE_CHN,
                    ret, channel.capt.width, channel.capt.height, channel.pixFmt);
        return RSS_ERR_IO;
    }
    st->vpe_chn_created = true;

    memset(&param, 0, sizeof(param));
    param.hdr = I6_HDR_OFF;
    /* 3DNR level 1, as both references default it. Range is 0-7 and
     * exposing it belongs with the rest of the ISP controls in phase 3. */
    param.level3DNR = 1;
    /* Digital mirror/flip stay off; orientation is a sensor register.
     * See the note in hal_framesource.c's star_fs_configure. */
    param.mirror = 0;
    param.flip = 0;
    param.lensAdjOn = 0;

    ret = st->vpe.fnSetChannelParam(STAR_VPE_CHN, (i6_vpe_para *)&param);
    if (ret) {
        HAL_LOG_ERR("MI_VPE_SetChannelParam(%d) failed: %d", STAR_VPE_CHN, ret);
        return RSS_ERR_IO;
    }

    ret = st->vpe.fnStartChannel(STAR_VPE_CHN);
    if (ret) {
        HAL_LOG_ERR("MI_VPE_StartChannel(%d) failed: %d", STAR_VPE_CHN, ret);
        return RSS_ERR_IO;
    }
    st->vpe_chn_started = true;

    /*
     * VIF -> VPE, hardware streaming link. I6_SYS_LINK_REALTIME pairs
     * with the channel's I6_VPE_MODE_REALTIME and VIF's
     * RGB_REALTIME work mode: pixels reach the ISP without a DRAM
     * round trip, which is why a realtime-bound port reports
     * MI_SYS_REALTIME_MAGIC_PADDR/VADDR instead of usable addresses
     * (SigmaStar MI_SYS reference, MI_SYS_FrameData_PhySignalType).
     * Frames become CPU-readable at the VPE *output* ports, which are
     * framebase.
     */
    fps = st->fps ? st->fps : st->res.maxFps;

    memset(&source, 0, sizeof(source));
    source.module = I6_SYS_MOD_VIF;
    source.device = STAR_VIF_DEV;
    source.channel = STAR_VIF_CHN;
    source.port = STAR_VIF_PORT;

    memset(&dest, 0, sizeof(dest));
    dest.module = I6_SYS_MOD_VPE;
    dest.device = STAR_VPE_DEV;
    dest.channel = STAR_VPE_CHN;
    dest.port = 0;

    ret = st->sys.fnBindExt(&source, &dest, fps, fps, I6_SYS_LINK_REALTIME, 0);
    if (ret) {
        HAL_LOG_ERR("MI_SYS_BindChnPort2 VIF->VPE failed: %d", ret);
        return RSS_ERR_IO;
    }
    st->vif_vpe_bound = true;

    HAL_LOG_INFO("VPE up: chn %d, %ux%u in, realtime link from VIF at %u fps", STAR_VPE_CHN,
                 channel.capt.width, channel.capt.height, fps);

    return RSS_OK;
}

static int star_teardown(star_state_t *st);

/*
 * hal_init -- bring up the MI pipeline as far as VIF.
 *
 * The Ingenic equivalent runs
 *   IMP_ISP_Open -> IMP_ISP_AddSensor -> IMP_ISP_EnableSensor
 *   -> IMP_System_Init -> IMP_ISP_EnableTuning
 * (src/hal_common.c:1204). The MI equivalent is
 *   dlopen the modules -> MI_SYS_Init -> MI_SNR_* -> MI_VIF_*
 * with VPE and VENC added by tasks 2c and 2d, which bind to the VIF port
 * this leaves enabled.
 */
static int hal_init(void *ctx, const rss_multi_sensor_config_t *cfg)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;
    star_state_t *st;
    int ret;

    if (!c || !cfg || cfg->sensor_count < 1 || cfg->sensor_count > RSS_MAX_SENSORS)
        return RSS_ERR_INVAL;

    if (c->initialized) {
        HAL_LOG_ERR("hal_init: already initialized");
        return RSS_ERR_BUSY;
    }

    /*
     * MI takes no sensor identity from us: MI_SNR is index-based and the
     * sensor is fixed when sensor_<name>_mipi.ko is insmod'd. So the I2C
     * address, GPIOs and driver name in cfg have no MI equivalent and are
     * stored only for the accessors and for deinit ordering. Say so when
     * more than one sensor is requested, rather than silently using one.
     */
    if (cfg->sensor_count > 1)
        HAL_LOG_WARN("hal_init: %d sensors requested, MI backend drives 1", cfg->sensor_count);

    memcpy(&c->multi_cfg, cfg, sizeof(c->multi_cfg));
    c->sensor_count = 1;
    memcpy(&c->sensors[0], &cfg->sensors[0], sizeof(c->sensors[0]));

    st = (star_state_t *)calloc(1, sizeof(*st));
    if (!st)
        return RSS_ERR_NOMEM;
    c->platform = st;

    /*
     * Load order matters: libcam_os_wrapper (pulled in by i6_sys_load) must
     * be RTLD_GLOBAL-resident before any other libmi_*, because each of them
     * leaves its cross-library symbols undefined for the loader to satisfy
     * from the global scope.
     */
    ret = i6_sys_load(&st->sys);
    if (ret)
        goto err_free;
    ret = i6_snr_load(&st->snr);
    if (ret)
        goto err_unload;
    ret = i6_vif_load(&st->vif);
    if (ret)
        goto err_unload;
    ret = i6_vpe_load(&st->vpe);
    if (ret)
        goto err_unload;

    ret = st->sys.fnInit();
    if (ret) {
        HAL_LOG_ERR("MI_SYS_Init failed: %d", ret);
        ret = RSS_ERR_IO;
        goto err_unload;
    }
    st->sys_inited = true;

    ret = star_sensor_bringup(st, c->hflip_state[0], c->vflip_state[0]);
    if (ret)
        goto err_teardown;

    ret = star_vif_bringup(st);
    if (ret)
        goto err_teardown;

    ret = star_vpe_bringup(st);
    if (ret)
        goto err_teardown;

    g_star = st;
    c->initialized = true;
    return RSS_OK;

err_teardown:
    star_teardown(st);
err_unload:
    i6_vpe_unload(&st->vpe);
    i6_vif_unload(&st->vif);
    i6_snr_unload(&st->snr);
    i6_sys_unload(&st->sys);
err_free:
    free(st);
    c->platform = NULL;
    return ret;
}

/*
 * star_teardown -- undo whatever bring-up actually completed.
 *
 * Driven by the flags rather than by assuming a fully-built pipeline, so a
 * failure partway through hal_init does not disable blocks that were never
 * enabled. Return codes are logged, never propagated: teardown has no
 * recovery, and a first failure must not skip the remaining steps.
 */
static int star_teardown(star_state_t *st)
{
    int ret;
    int i;

    if (!st)
        return RSS_OK;

    /*
     * Ports first, then the link, then the channel -- the reverse of
     * bring-up. divinus disables all four ports before unbinding
     * (i6_hal.c:363-377); a port left enabled with its VENC bind gone
     * is what leaves MI's kernel side holding buffers.
     */
    /* hal_framesource.c is a video-module source, so the audio archive
     * has no fs ops to have checked a frame out in the first place. */
#ifdef HAL_MODULE_VIDEO
    star_fs_release_all(st);
#endif
    for (i = 0; i < STAR_VPE_PORT_NUM; i++) {
        if (!st->port[i].enabled)
            continue;
        ret = st->vpe.fnDisablePort(STAR_VPE_CHN, i);
        if (ret)
            HAL_LOG_WARN("MI_VPE_DisablePort(%d, %d) failed: %d", STAR_VPE_CHN, i, ret);
        st->port[i].enabled = false;
        st->port[i].configured = false;
    }

    if (st->vif_vpe_bound) {
        i6_sys_bind source, dest;

        memset(&source, 0, sizeof(source));
        source.module = I6_SYS_MOD_VIF;
        source.device = STAR_VIF_DEV;
        source.channel = STAR_VIF_CHN;
        source.port = STAR_VIF_PORT;

        memset(&dest, 0, sizeof(dest));
        dest.module = I6_SYS_MOD_VPE;
        dest.device = STAR_VPE_DEV;
        dest.channel = STAR_VPE_CHN;
        dest.port = 0;

        ret = st->sys.fnUnbind(&source, &dest);
        if (ret)
            HAL_LOG_WARN("MI_SYS_UnBindChnPort VIF->VPE failed: %d", ret);
        st->vif_vpe_bound = false;
    }

    if (st->vpe_chn_started) {
        ret = st->vpe.fnStopChannel(STAR_VPE_CHN);
        if (ret)
            HAL_LOG_WARN("MI_VPE_StopChannel failed: %d", ret);
        st->vpe_chn_started = false;
    }

    if (st->vpe_chn_created) {
        ret = st->vpe.fnDestroyChannel(STAR_VPE_CHN);
        if (ret)
            HAL_LOG_WARN("MI_VPE_DestroyChannel failed: %d", ret);
        st->vpe_chn_created = false;
    }

    if (st->vif_port_enabled) {
        ret = st->vif.fnDisablePort(STAR_VIF_CHN, STAR_VIF_PORT);
        if (ret)
            HAL_LOG_WARN("MI_VIF_DisableChnPort failed: %d", ret);
        st->vif_port_enabled = false;
    }

    if (st->vif_dev_enabled) {
        ret = st->vif.fnDisableDevice(STAR_VIF_DEV);
        if (ret)
            HAL_LOG_WARN("MI_VIF_DisableDev failed: %d", ret);
        st->vif_dev_enabled = false;
    }

    if (st->snr_enabled) {
        ret = st->snr.fnDisable(STAR_SNR_INDEX);
        if (ret)
            HAL_LOG_WARN("MI_SNR_Disable failed: %d", ret);
        st->snr_enabled = false;
    }

    if (st->sys_inited) {
        ret = st->sys.fnExit();
        if (ret)
            HAL_LOG_WARN("MI_SYS_Exit failed: %d", ret);
        st->sys_inited = false;
    }

    return RSS_OK;
}

static int hal_deinit(void *ctx)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;
    star_state_t *st = star_state(ctx);

    if (!c)
        return RSS_ERR_INVAL;

    if (!st)
        return RSS_OK;

    star_teardown(st);

    i6_vpe_unload(&st->vpe);
    i6_vif_unload(&st->vif);
    i6_snr_unload(&st->snr);
    i6_sys_unload(&st->sys);

    if (g_star == st)
        g_star = NULL;

    free(st);
    c->platform = NULL;
    c->initialized = false;

    return RSS_OK;
}

/* ================================================================
 * SYSTEM UTILITIES
 * ================================================================ */

static int hal_sys_get_version(void *ctx, char *buf, int len)
{
    star_state_t *st = star_state(ctx);
    i6_sys_ver ver;
    int ret;

    if (!buf || len <= 0)
        return RSS_ERR_INVAL;
    if (!st || !st->sys.fnGetVersion)
        return RSS_ERR_NOTSUP;

    memset(&ver, 0, sizeof(ver));
    ret = st->sys.fnGetVersion(&ver);
    if (ret)
        return RSS_ERR_IO;

    /* version[] is not guaranteed terminated; bound the copy by both sizes. */
    snprintf(buf, (size_t)len, "%.*s", (int)sizeof(ver.version), (char *)ver.version);
    return RSS_OK;
}

/*
 * Media clock. rvd_frame_loop.c uses these to publish the
 * media-clock-to-UTC mapping that SEI timecodes are derived from; without
 * them the mapping early-returns and frames still flow, but timecodes
 * silently vanish. See i6_sys.h for how these signatures were established --
 * MI_SYS_GetCurPts takes one pointer on Infinity6E, not the leading device
 * argument waybeam uses on Mercury6.
 */
static int hal_sys_get_timestamp(void *ctx, int64_t *ts)
{
    star_state_t *st = star_state(ctx);
    unsigned long long pts = 0;
    int ret;

    if (!ts)
        return RSS_ERR_INVAL;
    if (!st || !st->sys.fnGetCurrentPts)
        return RSS_ERR_NOTSUP;

    ret = st->sys.fnGetCurrentPts(&pts);
    if (ret)
        return RSS_ERR_IO;

    *ts = (int64_t)pts;
    return RSS_OK;
}

static int hal_sys_rebase_timestamp(void *ctx, int64_t base)
{
    star_state_t *st = star_state(ctx);
    int ret;

    if (!st || !st->sys.fnInitPtsBase)
        return RSS_ERR_NOTSUP;

    ret = st->sys.fnInitPtsBase((unsigned long long)base);
    if (ret)
        return RSS_ERR_IO;

    return RSS_OK;
}

/*
 * hal_get_caps -- return the per-SoC capability struct.
 *
 * Copied into the context at create time from g_hal_caps.
 */
static const rss_hal_caps_t *hal_get_caps(void *ctx)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;

    if (!c)
        return NULL;

    return &c->caps;
}

/* ================================================================
 * OPS VTABLE
 *
 * Only implemented ops are listed. Everything else stays NULL and
 * resolves to RSS_ERR_NOTSUP through RSS_HAL_CALL.
 * ================================================================ */

static const rss_hal_ops_t g_ops = {
    /* System lifecycle */
    .init = hal_init,
    .deinit = hal_deinit,
    .get_caps = hal_get_caps,

    /* System utilities */
    .sys_get_version = hal_sys_get_version,
    .sys_get_timestamp = hal_sys_get_timestamp,
    .sys_rebase_timestamp = hal_sys_rebase_timestamp,

#ifdef HAL_MODULE_VIDEO
    /* Framesource -- VPE output ports (src/star/hal_framesource.c).
     * The ops MI has no equivalent for are listed, with reasons, in
     * that file's header comment. */
    .fs_create_channel = hal_fs_create_channel,
    .fs_set_channel_attr = hal_fs_set_channel_attr,
    .fs_destroy_channel = hal_fs_destroy_channel,
    .fs_enable_channel = hal_fs_enable_channel,
    .fs_disable_channel = hal_fs_disable_channel,
    .fs_set_fifo = hal_fs_set_fifo,
    .fs_get_fifo = hal_fs_get_fifo,
    .fs_set_frame_depth = hal_fs_set_frame_depth,
    .fs_get_frame_depth = hal_fs_get_frame_depth,
    .fs_get_frame = hal_fs_get_frame,
    .fs_release_frame = hal_fs_release_frame,
#endif

    /* Encoder, ISP, OSD and audio ops are added by the phases that
     * implement them. */

#ifdef HAL_MODULE_VIDEO
    /* GPIO / IR-cut — vendor-neutral sysfs, works as-is */
    .gpio_set = hal_gpio_set,
    .gpio_get = hal_gpio_get,
    .ircut_set = hal_ircut_set,
#endif
};

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

    for (i = 0; i < RSS_MAX_ENC_CHANNELS; i++) {
        free(ctx->scratch_buf[i]);
        ctx->scratch_buf[i] = NULL;
        if (ctx->nal_arrays[i]) {
            free(ctx->nal_arrays[i]);
            ctx->nal_arrays[i] = NULL;
        }
    }

    free(ctx);
}

const rss_hal_ops_t *rss_hal_get_ops(rss_hal_ctx_t *ctx)
{
    if (!ctx)
        return NULL;

    return ctx->ops;
}

/* ================================================================
 * SYSTEM INFO (no vtable, called directly)
 * ================================================================ */

/*
 * rss_hal_get_imp_version / rss_hal_get_sysutils_version
 *
 * Both names are IMP-specific but the daemons call them unconditionally to
 * print a build banner, with no context argument. MI's equivalent is
 * MI_SYS_GetVersion, reached through the g_star pointer, so this answers
 * only after hal_init -- before that there is no loaded library to ask.
 * There is no sysutils equivalent at all, so that one stays unsupported
 * permanently.
 */
int rss_hal_get_imp_version(char *buf, int size)
{
    i6_sys_ver ver;

    if (!buf || size <= 0)
        return RSS_ERR_INVAL;

    if (!g_star || !g_star->sys.fnGetVersion)
        return RSS_ERR_NOTSUP;

    memset(&ver, 0, sizeof(ver));
    if (g_star->sys.fnGetVersion(&ver))
        return RSS_ERR_IO;

    snprintf(buf, (size_t)size, "%.*s", (int)sizeof(ver.version), (char *)ver.version);
    return RSS_OK;
}

int rss_hal_get_sysutils_version(char *buf, int size)
{
    if (!buf || size <= 0)
        return RSS_ERR_INVAL;

    return RSS_ERR_NOTSUP;
}

/*
 * rss_hal_get_cpu_info -- SoC identification string.
 *
 * IMP exposes IMP_System_GetCPUInfo(); MI has no equivalent, so read the
 * "Hardware" line out of /proc/cpuinfo (the SigmaStar 4.9 kernel reports
 * e.g. "Sigmastar SSC338Q"). Cached after the first call because the
 * caller treats the result as a borrowed static string.
 */
const char *rss_hal_get_cpu_info(void)
{
    static char cpu[64];
    static bool loaded = false;

    if (loaded)
        return cpu;

    loaded = true;
    snprintf(cpu, sizeof(cpu), "%s", HAL_PLATFORM_NAME);

    FILE *f = fopen("/proc/cpuinfo", "r");
    if (!f)
        return cpu;

    char line[256];
    while (fgets(line, sizeof(line), f)) {
        if (strncmp(line, "Hardware", 8) != 0)
            continue;
        char *val = strchr(line, ':');
        if (!val)
            break;
        val++;
        while (*val == ' ' || *val == '\t')
            val++;
        char *end = val + strlen(val);
        while (end > val && (end[-1] == '\n' || end[-1] == '\r' || end[-1] == ' '))
            end--;
        *end = '\0';
        if (*val)
            snprintf(cpu, sizeof(cpu), "%s", val);
        break;
    }

    fclose(f);
    return cpu;
}

const char *rss_hal_get_platform_name(void)
{
    return HAL_PLATFORM_NAME;
}

/*
 * rss_hal_check_platform -- verify the binary matches the running SoC.
 *
 * The Ingenic path compares IMP_System_GetCPUInfo() against
 * HAL_PLATFORM_NAME, which works because IMP reports exactly "T31" etc.
 * /proc/cpuinfo reports a marketing string ("Sigmastar SSC338Q") that
 * does not contain "INFINITY6E", so the same prefix comparison would
 * reject every valid board. Checking properly needs a SoC-ID-to-family
 * table; until then this only warns, and never aborts.
 */
void rss_hal_check_platform(const char *name)
{
    (void)name;

    HAL_LOG_DBG("platform check: built for %s, running on %s", HAL_PLATFORM_NAME,
                rss_hal_get_cpu_info());
}
