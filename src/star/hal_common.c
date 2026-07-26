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

#include <ctype.h>
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
 * star_sensor_name_matches -- "is the config talking about this sensor?"
 *
 * Compared by containment after lowercasing, not by equality. MI reports
 * whatever case the driver author wrote ("GC4653"), config files are
 * conventionally lowercase, and either side may or may not carry an interface
 * suffix, so equality would produce false alarms far more often than it would
 * catch a genuinely wrong config.
 */
static bool star_sensor_name_matches(const char *cfg_name, const char *drv_name)
{
    char a[48], b[48];
    size_t i;

    for (i = 0; i + 1 < sizeof(a) && cfg_name[i]; i++)
        a[i] = (char)tolower((unsigned char)cfg_name[i]);
    a[i] = '\0';
    for (i = 0; i + 1 < sizeof(b) && drv_name[i]; i++)
        b[i] = (char)tolower((unsigned char)drv_name[i]);
    b[i] = '\0';

    if (!a[0] || !b[0])
        return true; /* nothing to contradict */

    return strstr(b, a) != NULL || strstr(a, b) != NULL;
}

/*
 * star_sensor_detect -- name the loaded sensor driver without touching MI.
 *
 * MI reports the sensor name in i6_snr_plane.sensName, but only after
 * MI_SNR_Enable, and a daemon needs the name earlier than that: it builds the
 * config it hands to hal_init, and on Ingenic that config is what tells IMP
 * which sensor to talk to. So this reads what the kernel has already published
 * instead, and is the one op valid before init.
 *
 * On SigmaStar the sensor is a kernel module, insmod'd as sensor_<name>_mipi
 * (or _dvp), so /proc/modules already carries the answer:
 *
 *     sensor_gc4653_mipi 20480 0 - Live 0xbf000000
 *
 * The `sensor_` prefix is required rather than optional. Matching any module
 * whose name merely contains something plausible would eventually pick up an
 * unrelated module on somebody's board, and the failure mode -- confidently
 * reporting the wrong sensor -- is worse than reporting none and letting
 * `[sensor] name` in the config settle it.
 */
static int star_sensor_detect_from(const char *path, char *buf, size_t len)
{
    FILE *fp;
    char line[256];
    char module[64] = "";
    bool found = false;

    if (!buf || len == 0)
        return RSS_ERR_INVAL;

    fp = fopen(path, "r");
    if (!fp) {
        HAL_LOG_WARN("sensor detect: %s: %s", path, strerror(errno));
        return RSS_ERR_IO;
    }

    while (!found && fgets(line, sizeof(line), fp)) {
        char *name = line;
        char *end;
        size_t n;

        /* First field is the module name. */
        end = strpbrk(name, " \t\n");
        if (end)
            *end = '\0';

        if (strncmp(name, "sensor_", 7) != 0)
            continue;
        snprintf(module, sizeof(module), "%s", name); /* keep for the log */
        name += 7;
        if (!*name)
            continue;

        /* Trim the interface suffix the driver names carry. */
        n = strlen(name);
        if (n > 5 && strcmp(name + n - 5, "_mipi") == 0)
            n -= 5;
        else if (n > 4 && strcmp(name + n - 4, "_dvp") == 0)
            n -= 4;
        if (n == 0)
            continue;

        if (n >= len) {
            HAL_LOG_WARN("sensor detect: name \"%.*s\" needs %zu bytes, caller gave %zu",
                         (int)n, name, n + 1, len);
            break;
        }
        memcpy(buf, name, n);
        buf[n] = '\0';
        found = true;
    }

    fclose(fp);

    if (!found) {
        HAL_LOG_WARN("sensor detect: no sensor_*.ko listed in %s", path);
        return RSS_ERR_NOENT;
    }

    /* Both names, so a mis-trimmed module is obvious from the log alone. */
    HAL_LOG_INFO("sensor detect: \"%s\" (module %s in %s)", buf, module, path);
    return RSS_OK;
}

static int hal_sensor_detect(void *ctx, char *buf, size_t len)
{
    /*
     * Deliberately does not go through star_state(): this op is legal before
     * init, when there is no MI state to consult, and it needs none.
     */
    if (!ctx)
        return RSS_ERR_INVAL;
    return star_sensor_detect_from("/proc/modules", buf, len);
}

/*
 * star_sensor_pick_mode -- choose among the modes the sensor driver enumerates.
 *
 * Index 0 is the native mode and the default. A config that asks for a
 * specific geometry gets an exact match on the mode's output dimensions; fps
 * only breaks ties, because MI_SNR_SetFps programs the rate separately within
 * a mode's [minFps, maxFps] range, so a mode whose range merely contains the
 * requested rate is a perfectly good answer.
 *
 * An unmatched request warns and falls back to native. The mode list belongs
 * to the sensor driver, and refusing to start because raptor.conf names a
 * resolution this sensor lacks would be a worse outcome than starting at its
 * native one and saying so plainly.
 */
static unsigned char star_sensor_pick_mode(star_state_t *st, const rss_sensor_config_t *cfg,
                                           unsigned int count)
{
    unsigned int want_w = cfg->width;
    unsigned int want_h = cfg->height;
    unsigned char best = 0;
    bool have_best = false;
    unsigned int i;

    /* fps alone does not select a mode -- it is programmed within whatever
     * mode is chosen, further down in star_sensor_bringup. */
    if (!want_w || !want_h)
        return 0;

    /* MI indexes modes with an unsigned char; nothing real comes close. */
    if (count > 255)
        count = 255;

    for (i = 0; i < count; i++) {
        i6_snr_res res;

        if (st->snr.fnGetResolution(STAR_SNR_INDEX, (unsigned char)i, &res))
            continue;
        if (res.output.width != want_w || res.output.height != want_h)
            continue;

        /* Exact geometry. Prefer a mode that can also run the wanted rate. */
        if (!have_best) {
            best = (unsigned char)i;
            have_best = true;
        }
        if (cfg->fps && cfg->fps >= res.minFps && cfg->fps <= res.maxFps) {
            best = (unsigned char)i;
            break;
        }
    }

    if (!have_best) {
        HAL_LOG_WARN("sensor: no mode matches the requested %ux%u; using native mode 0",
                     want_w, want_h);
        return 0;
    }

    HAL_LOG_INFO("sensor: mode %u matches the requested %ux%u", best, want_w, want_h);
    return best;
}

/*
 * star_sensor_bringup -- select a mode, start the sensor, read back what it is.
 *
 * Sequence follows both references: SetPlaneMode -> QueryResCount ->
 * SetRes -> SetFps -> SetOrien -> Enable.
 *
 * Two deliberate choices:
 *
 * Geometry comes from the sensor, not from a constant: the mode list is read
 * from the driver at runtime, and the config can only *select* among what the
 * driver offers, never assert a size. So there is no hardcoded 2560x1440 to
 * mislead on a different sensor, and a board with a multi-mode sensor is
 * configurable without a table in here.
 *
 * The descriptors are read back *after* Enable, unlike divinus. The sensor
 * driver's pCus_sensor_init runs on Enable, and before it does, pad.intfAttr
 * and the plane geometry read back as zero -- confirmed on this board with
 * star_probe, which reported "planes 0, lanes 0" pre-Enable and correct
 * values after. divinus queries beforehand and gets away with it only
 * because the single field it uses from intfAttr, mipi.input, is 0 for this
 * sensor anyway. waybeam queries after Enable (sensor_select.c:485); so do we.
 */
static int star_sensor_bringup(star_state_t *st, const rss_sensor_config_t *cfg, int mirror,
                               int flip)
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

    st->res_index = star_sensor_pick_mode(st, cfg, count);
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
     * Frame rate: honour the config's request within the mode's range, else
     * run the mode as fast as it goes.
     *
     * `[sensor] fps` reaches Ingenic through isp_set_sensor_fps, which MI has
     * no equivalent of -- MI_SNR_SetFps is part of sensor bring-up and is
     * called once, here, before Enable. So this is where that setting has to
     * be applied on this backend, and without it the key would silently do
     * nothing.
     *
     * A request outside [minFps, maxFps] is clamped rather than refused, and
     * says so: the range belongs to the selected mode, and dropping to a
     * working rate beats failing to start.
     *
     * The rate is recorded as well as programmed because MI_SYS_BindChnPort2
     * takes source and destination frame rates -- both the VIF->VPE bind below
     * and the VPE->VENC bind need it -- and MI offers no way to read back what
     * the sensor is actually running at.
     */
    if (st->res.maxFps) {
        unsigned int want = cfg->fps ? cfg->fps : st->res.maxFps;

        if (want > st->res.maxFps || (st->res.minFps && want < st->res.minFps)) {
            unsigned int clamped = want > st->res.maxFps ? st->res.maxFps : st->res.minFps;
            HAL_LOG_WARN("sensor: requested %u fps is outside the mode's %u-%u range; "
                         "using %u",
                         want, st->res.minFps, st->res.maxFps, clamped);
            want = clamped;
        }

        ret = st->snr.fnSetFramerate(STAR_SNR_INDEX, want);
        if (ret)
            HAL_LOG_WARN("MI_SNR_SetFps(%u) failed: %d", want, ret);
        else
            st->fps = want;
    }

    /* Orientation before Enable, so the driver's init picks it up.
     * Recorded because MI_SNR_SetOrien has no getter and it takes both
     * axes at once, so a later isp_set_hflip has to supply the flip it
     * is not changing. */
    st->hflip = mirror ? true : false;
    st->vflip = flip ? true : false;
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

    HAL_LOG_INFO("sensor \"%.32s\": mode %u \"%.32s\", %ux%u, %u-%u fps, bayer %d, "
                 "precision %d, pixFmt %d",
                 st->plane.sensName, st->res_index, st->res.desc, st->plane.capt.width,
                 st->plane.capt.height, st->res.minFps, st->res.maxFps, st->plane.bayer,
                 st->plane.precision, st->plane.pixFmt);

    /*
     * MI's own idea of the sensor name is authoritative and only available
     * here, after Enable. If the config named a different one, the config is
     * stale or was copied from another board -- worth saying, but not worth
     * refusing to start over, since MI addresses the sensor by index and the
     * name it was given never reached the hardware either way.
     */
    if (cfg->name[0] && !star_sensor_name_matches(cfg->name, st->plane.sensName))
        HAL_LOG_WARN("sensor: config says \"%.20s\" but the driver reports \"%.32s\"; "
                     "the config name is advisory on this backend and was not used",
                     cfg->name, st->plane.sensName);

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
    int i;

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

    /* -1, not the 0 calloc left behind: 0 is a legitimate file
     * descriptor, so "never opened" has to be distinguishable from
     * "opened as fd 0" before anything closes one. The VPE ports get
     * the same treatment in star_vpe_bringup. */
    for (i = 0; i < I6_VENC_CHN_NUM; i++) {
        st->enc[i].fd = -1;
        st->enc[i].src_port = -1;
    }

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
    /* libmi_venc is a video-module dependency: the audio archive has no
     * encoder ops, so it has no reason to pull the library in. */
#ifdef HAL_MODULE_VIDEO
    ret = i6_venc_load(&st->venc);
    if (ret)
        goto err_unload;
#endif

    ret = st->sys.fnInit();
    if (ret) {
        HAL_LOG_ERR("MI_SYS_Init failed: %d", ret);
        ret = RSS_ERR_IO;
        goto err_unload;
    }
    st->sys_inited = true;

    /*
     * Orientation comes from the sensor config, not from hflip_state[].
     * That array is where the generic ISP layer records what an op last
     * set, and no op can have run yet -- it is unavoidably still zero
     * here, so reading it was the same as hardcoding "no flip" and the
     * config's request never reached the sensor. It is set from the
     * config so a later read of it agrees with the hardware.
     */
    c->hflip_state[0] = c->sensors[0].hflip ? 1 : 0;
    c->vflip_state[0] = c->sensors[0].vflip ? 1 : 0;

    ret = star_sensor_bringup(st, &c->sensors[0], c->hflip_state[0], c->vflip_state[0]);
    if (ret)
        goto err_teardown;

    ret = star_vif_bringup(st);
    if (ret)
        goto err_teardown;

    ret = star_vpe_bringup(st);
    if (ret)
        goto err_teardown;

    /*
     * ISP last, and not before: the ISP channel is the front half of the
     * VPE channel, so nothing here -- not the readiness probe, not the
     * tuning load -- is legal until star_vpe_bringup has created and
     * started that channel. Returns void because every failure inside it
     * is best-effort; see star_isp_bringup.
     */
#ifdef HAL_MODULE_VIDEO
    star_isp_bringup(st, &c->sensors[0]);
#endif

    g_star = st;
    c->initialized = true;
    return RSS_OK;

err_teardown:
    star_teardown(st);
err_unload:
#ifdef HAL_MODULE_VIDEO
    i6_venc_unload(&st->venc);
#endif
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
    /* hal_framesource.c and hal_encoder.c are video-module sources, so
     * the audio archive has no fs or enc ops to have checked a frame out
     * in the first place. Encoders go first: they are downstream of the
     * ports, and a VENC channel left bound to a port that is about to be
     * disabled is exactly the state divinus's teardown avoids. */
#ifdef HAL_MODULE_VIDEO
    /* ISP first, mirroring bring-up: it is bound to the VPE channel that
     * is about to go away, and dropping the library handles cannot fail. */
    star_isp_teardown(st);
    star_enc_release_all(st);
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

#ifdef HAL_MODULE_VIDEO
    i6_venc_unload(&st->venc);
#endif
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

#ifdef HAL_MODULE_VIDEO

/* ================================================================
 * BIND
 *
 * rvd builds its pipeline as a chain of cells (rvd_pipeline.c:1156):
 * FS [-> IVS] [-> OSD] -> ENC. On MI that chain is a single link,
 * VPE output port -> VENC channel, because MI has no separate IVS or
 * OSD stage in the data path -- MI_RGN overlays composite onto a VPE
 * port in place rather than sitting between two modules.
 *
 * So the only supported pair is FS -> ENC. Anything else returns
 * RSS_ERR_NOTSUP, which is honest: silently accepting an OSD stage
 * that does not exist would leave rvd believing overlays are bound.
 */
static int star_bind_check(const rss_cell_t *src, const rss_cell_t *dst)
{
    if (!src || !dst)
        return RSS_ERR_INVAL;

    if (src->device != RSS_DEV_FS || dst->device != RSS_DEV_ENC) {
        HAL_LOG_ERR("bind: only FS -> ENC is available on this backend (got %d -> %d)",
                    src->device, dst->device);
        return RSS_ERR_NOTSUP;
    }

    return RSS_OK;
}

static int hal_bind(void *ctx, const rss_cell_t *src, const rss_cell_t *dst)
{
    star_state_t *st = star_state(ctx);
    int ret;

    if (!st)
        return RSS_ERR_INVAL;

    ret = star_bind_check(src, dst);
    if (ret)
        return ret;

    return star_enc_bind_port(st, src->group, dst->group);
}

static int hal_unbind(void *ctx, const rss_cell_t *src, const rss_cell_t *dst)
{
    star_state_t *st = star_state(ctx);
    int ret;

    if (!st)
        return RSS_ERR_INVAL;

    ret = star_bind_check(src, dst);
    if (ret)
        return ret;

    return star_enc_unbind_port(st, src->group, dst->group);
}

#endif /* HAL_MODULE_VIDEO */

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

#ifdef HAL_MODULE_VIDEO
/*
 * hal_isp_get_sensor_attr -- the sensor's active geometry.
 *
 * Reports what MI_SNR handed back for the mode actually selected, which is why
 * this is worth having even though a daemon could read a resolution out of its
 * own config: the config only *requests*, and an unmatched request falls back
 * to native. This is the answer after that negotiation.
 *
 * Post-init only, unlike sensor_detect -- the plane descriptor is not populated
 * until MI_SNR_Enable has run.
 */
static int hal_isp_get_sensor_attr(void *ctx, uint32_t *width, uint32_t *height)
{
    star_state_t *st = star_state(ctx);

    if (!st || !width || !height)
        return RSS_ERR_INVAL;
    /* Same convention the encoder ops use for "asked about something that
     * does not exist yet": there is no RSS_ERR_STATE. */
    if (!st->snr_enabled)
        return RSS_ERR_NOENT;

    *width = st->plane.capt.width;
    *height = st->plane.capt.height;
    return RSS_OK;
}
#endif /* HAL_MODULE_VIDEO */

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
    .sensor_detect = hal_sensor_detect,

    /* System utilities */
    .sys_get_version = hal_sys_get_version,
    .sys_get_timestamp = hal_sys_get_timestamp,
    .sys_rebase_timestamp = hal_sys_rebase_timestamp,

#ifdef HAL_MODULE_VIDEO
    /*
     * ISP (src/star/hal_isp.c). The tuning binary that CUS3A actually
     * runs on is loaded during hal_init, not through any op here -- it
     * has to happen at a specific point in the pipeline's construction,
     * which is not something a caller can be asked to know.
     *
     * The scalar ops below are read-modify-writes of MI's per-module IQ
     * structs, and they yield to that tuning binary: a knob left at its
     * neutral 128 puts its module back into *auto* rather than pinning a
     * midpoint over the tuned curve. That matters because rvd applies
     * the whole [image] block on every start, defaults included. The
     * ops MI cannot honour are listed, with reasons, in the OP COVERAGE
     * comment in that file.
     */
    .isp_get_sensor_attr = hal_isp_get_sensor_attr,

    .isp_set_brightness = hal_isp_set_brightness,
    .isp_set_contrast = hal_isp_set_contrast,
    .isp_set_saturation = hal_isp_set_saturation,
    .isp_set_sharpness = hal_isp_set_sharpness,
    .isp_set_sinter_strength = hal_isp_set_sinter_strength,
    .isp_set_temper_strength = hal_isp_set_temper_strength,
    .isp_set_ae_comp = hal_isp_set_ae_comp,
    .isp_set_defog = hal_isp_set_defog,
    .isp_set_antiflicker = hal_isp_set_antiflicker,
    .isp_set_max_again = hal_isp_set_max_again,
    .isp_set_max_dgain = hal_isp_set_max_dgain,
    .isp_set_running_mode = hal_isp_set_running_mode,
    .isp_set_hflip = hal_isp_set_hflip,
    .isp_set_vflip = hal_isp_set_vflip,

    .isp_get_brightness = hal_isp_get_brightness,
    .isp_get_contrast = hal_isp_get_contrast,
    .isp_get_saturation = hal_isp_get_saturation,
    .isp_get_sharpness = hal_isp_get_sharpness,
    .isp_get_sinter_strength = hal_isp_get_sinter_strength,
    .isp_get_temper_strength = hal_isp_get_temper_strength,
    .isp_get_ae_comp = hal_isp_get_ae_comp,
    .isp_get_antiflicker = hal_isp_get_antiflicker,
    .isp_get_max_again = hal_isp_get_max_again,
    .isp_get_max_dgain = hal_isp_get_max_dgain,
    .isp_get_running_mode = hal_isp_get_running_mode,
    .isp_get_hvflip = hal_isp_get_hvflip,

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

    /* Pipeline binds -- FS -> ENC only, see hal_bind above */
    .bind = hal_bind,
    .unbind = hal_unbind,

    /* Encoder -- MI VENC channels (src/star/hal_encoder.c). The ops MI
     * has no equivalent for are listed in that file's header comment. */
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
    .enc_set_gop_attr = hal_enc_set_gop_attr,
    .enc_set_fps = hal_enc_set_fps,
    .enc_get_channel_attr = hal_enc_get_channel_attr,
    .enc_get_fps = hal_enc_get_fps,
    .enc_get_gop_attr = hal_enc_get_gop_attr,
    .enc_get_avg_bitrate = hal_enc_get_avg_bitrate,
    .enc_query = hal_enc_query,
    .enc_get_fd = hal_enc_get_fd,
#endif

    /* ISP, OSD and audio ops are added by the phases that implement
     * them. */

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
