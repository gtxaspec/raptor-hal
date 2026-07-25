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

#include "hal_internal.h"

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
 * hal_init -- bring up the MI pipeline.
 *
 * The Ingenic equivalent runs
 *   IMP_ISP_Open -> IMP_ISP_AddSensor -> IMP_ISP_EnableSensor
 *   -> IMP_System_Init -> IMP_ISP_EnableTuning
 * (src/hal_common.c). The MI equivalent is
 *   MI_SYS_Init -> MI_SNR_SetPlaneMode/Enable -> MI_VIF_* -> MI_VPE_*
 * with explicit channel-port binding, per waybeam_venc's star6e_mi.c and
 * star6e_pipeline.c.
 *
 * Returning NOTSUP here (rather than leaving the op NULL) is deliberate:
 * rvd treats pipeline init failure as fatal and logs the return code, so
 * a build that is not yet functional fails immediately and legibly
 * instead of appearing to start and then serving nothing.
 */
static int hal_init(void *ctx, const rss_multi_sensor_config_t *cfg)
{
    (void)ctx;
    (void)cfg;

    HAL_LOG_ERR("SigmaStar MI backend: pipeline init not implemented yet "
                "(platform=%s)",
                HAL_PLATFORM_NAME);

    return RSS_ERR_NOTSUP;
}

static int hal_deinit(void *ctx)
{
    (void)ctx;
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

    /* Framesource, encoder, ISP, OSD and audio ops are added by the
     * phases that implement them. */

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
 * Both names are IMP-specific but the daemons call them unconditionally
 * to print a build banner. MI's equivalent is MI_SYS_GetVersion(), wired
 * up in phase 2; there is no sysutils equivalent at all, so that one
 * stays unsupported permanently.
 */
int rss_hal_get_imp_version(char *buf, int size)
{
    if (!buf || size <= 0)
        return RSS_ERR_INVAL;

    return RSS_ERR_NOTSUP;
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
