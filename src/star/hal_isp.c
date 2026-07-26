/*
 * star/hal_isp.c -- ISP tuning and 3A for the SigmaStar MI HAL
 *
 * ================================================================
 * THE POINT OF THIS FILE
 *
 * Phases 2c-2e produced a correct video pipeline with a visibly wrong
 * image: colours off, because enabling VPE auto-starts CUS3A and CUS3A
 * loads /etc/firmware/iqfile0.bin -- a generic tuning file that knows
 * nothing about the attached sensor. OpenIPC already ships a tuned
 * binary per sensor (/etc/sensors/<name>.bin), so the single most
 * valuable thing this file does is load the right one. Everything else
 * here is secondary, and some of it is actively better left undone --
 * see THE TUNING BINARY OUTRANKS THE CONFIG below.
 * ================================================================
 *
 * ================================================================
 * TWO SHAPES OF MI_ISP CALL, AND WHY THERE IS A TABLE
 *
 * libmi_isp.so exports ~360 functions. A handful are typed lifecycle
 * calls with real prototypes in i6_isp.h. The other ~340 are
 * MI_ISP_{IQ,AE,AWB,AF}_{Get,Set}<Module>, all of them
 * (int channel, void *payload), each with its own payload struct.
 *
 * Writing 340 structs to poke one field each would be absurd, and it is
 * not necessary: the payloads follow a fixed convention, discovered by
 * OpenIPC waybeam_venc (src/star6e_iq.c) and confirmed here against the
 * board's own library. An auto/manual module looks like
 *
 *     { uint32 bEnable; uint32 enOpType; <auto>[16]; <manual>; }
 *
 * where <auto> is 16 copies of a per-ISO parameter block, so the manual
 * block always begins at 8 + 16 * sizeof(per_iso_param). A manual-only
 * module drops enOpType and the auto array (manual at offset 4), and a
 * toggle-only module is just bEnable.
 *
 * So one field is reachable with (payload size, manual offset, width) and
 * no struct at all. Both numbers are verifiable without vendor headers:
 * each wrapper hardcodes its payload size, so
 *
 *     arm-openipc-linux-gnueabihf-objdump -d \
 *         --disassemble=MI_ISP_IQ_GetBrightness libmi_isp.so
 *
 * prints `mov.w r3, #76` into the size slot. Every entry in the table
 * below was checked that way, and the check is sharper than it sounds:
 * brightness is a u32 at manual offset 72 in a 76-byte payload, so the
 * manual value is provably the last four bytes and the convention is
 * confirmed rather than assumed. Saturation (392 in 416), sharpness
 * (1192 in 1268), NRLuma (104 in 112) and NR3D (1288 in 1776) all agree.
 *
 * Every access is read-modify-write. That is what makes poking one field
 * of a struct we have not fully described safe: whatever else the
 * payload holds -- and NR3D holds 1776 bytes of it -- survives untouched.
 * ================================================================
 *
 * ================================================================
 * THE TUNING BINARY OUTRANKS THE CONFIG
 *
 * rvd applies its whole [image] block unconditionally at startup, using
 * built-in defaults for every key the config omits (rvd_pipeline.c,
 * section 3c). A HAL that dutifully wrote all of them would flip nine
 * ISP modules from auto to manual on every boot -- overwriting, with
 * hardcoded midpoints, the tuned curves it had just loaded from the
 * sensor binary. The image would be worse than before this file existed.
 *
 * So a scalar knob at its neutral value (128) does not mean "write 128".
 * It means "nobody asked for anything", and this file answers it by
 * putting the module back into *auto*, which is where the tuning binary
 * wants it. Only a value that differs from neutral selects manual mode.
 * The mapping is reversible: setting a knob and then returning it to 128
 * restores auto rather than pinning the midpoint.
 *
 * A consequence worth stating plainly: a user who genuinely wants manual
 * brightness at exactly the midpoint cannot express it, and gets auto.
 * That trade is deliberate -- the alternative penalises every default
 * configuration to serve a request nobody has made.
 * ================================================================
 *
 * ================================================================
 * OP COVERAGE -- what is missing, and why
 *
 * Absent ops return RSS_ERR_NOTSUP through RSS_HAL_CALL's NULL guard,
 * and rvd treats all of these as advisory. Left unimplemented on
 * purpose:
 *
 *   isp_get_exposure      MI exposes no current-exposure query. There is
 *                         AE_GetManualExpo (the manual *setting*, not
 *                         what the AE converged on), AE_GetExposureLimit
 *                         (bounds), and raw histogram stats. None answer
 *                         "what is the shutter right now", so the op
 *                         would have to fabricate a number that phase 6
 *                         would then use for IR-cut decisions.
 *                         AE_GetAeHwAvgStats is the honest starting
 *                         point when phase 6 needs a luma signal.
 *   isp_set_dpc_strength  MI's DynamicDP manual field is one bit. A
 *                         0..255 strength knob does not map onto it, and
 *                         pretending otherwise reads as support for a
 *                         control that has two positions.
 *   isp_set_defog_strength / _adv
 *                         Same: MI's Defog is a toggle. The plain
 *                         isp_set_defog *is* implemented.
 *   isp_set_drc_strength, isp_set_highlight_depress,
 *   isp_set_backlight_comp
 *                         These live in WDR/HDR modules whose manual
 *                         blocks are multi-field curve descriptors, not
 *                         single strengths. Mapping one scalar onto a
 *                         curve is a tuning decision, not a HAL one.
 *   isp_set_hue           MI's hue control is a 64-entry HSV LUT
 *                         (manual@3096); a scalar rotation would mean
 *                         synthesising the whole table.
 *   isp_set_wb / isp_get_wb
 *                         AWB_SetAttr's 1464-byte payload is not
 *                         described by the offset convention above (it
 *                         is not an auto/manual pair), so it needs its
 *                         own derivation. Deferred rather than guessed.
 *   isp_set_bypass        No MI equivalent; the ISP cannot be bypassed
 *                         while VPE is the only path to the encoder.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/*
 * nanosleep needs this, and it has to precede every include.
 * raptor-hal builds -std=c11 rather than gnu11, so glibc defines
 * __STRICT_ANSI__ and hides everything outside ISO C -- including the
 * whole of POSIX. This is the first file in the HAL that has to wait for
 * hardware, so it is the first to need the macro. nanosleep in
 * preference to usleep: usleep was removed from POSIX in 2008.
 */
#define _POSIX_C_SOURCE 200809L

#include "star_state.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

/*
 * No HAL_MODULE_VIDEO guard here, deliberately. That define exists to
 * build hal_common.c twice, once per archive; this file is video-only by
 * construction -- it is in the Makefile's VIDEO_SRCS, so only
 * libraptor_hal_video.a ever links it. Wrapping it in the guard compiles
 * the whole translation unit away, which links as an empty object and
 * fails only later, as undefined vtable entries. hal_encoder.c and
 * hal_framesource.c are in the same position and carry no guard either.
 */

/*
 * raptor's scalar ISP knobs are 0..255 with 128 as neutral. Named
 * because the neutral value carries the auto/manual meaning described
 * above, so it is a protocol constant rather than a magic midpoint.
 */
#define STAR_ISP_NEUTRAL 128

/*
 * enOpType values. The vendor enum is E_MI_ISP_OP_TYPE_{AUTO,MANUAL},
 * auto first -- the same ordering every MI-family SDK uses for this
 * field. Not independently verified on hardware: a swap would show up
 * as a scalar knob having no effect, never as a crash, since the field
 * is a bounded enum inside a struct we read back before writing.
 */
#define STAR_ISP_OP_AUTO 0u
#define STAR_ISP_OP_MANUAL 1u

/* Offset of enOpType within an auto/manual payload -- always after the
 * leading bEnable, per the layout convention. */
#define STAR_ISP_OPTYPE_OFF 4u

/*
 * How long to wait for the IQ parameter store, and how often to look.
 *
 * The ISP channel initialises asynchronously after MI_VPE_CreateChannel
 * returns, so everything here has to wait for it once. 2000 ms is
 * waybeam's bound (star6e_pipeline.c:171); the observed wait on this
 * board is far shorter, but a cold boot with a slow sensor is the case
 * the bound exists for. divinus instead sleeps a flat second before its
 * load (media.c:827), which is the same wait without the evidence.
 */
#define STAR_ISP_READY_TIMEOUT_MS 2000
#define STAR_ISP_READY_POLL_MS 10

/* Largest payload the table below touches (NR3D, 1776). Sized generously
 * so a future entry does not silently overflow -- star_iq_call refuses
 * anything that does not fit rather than truncating. */
#define STAR_IQ_PAYLOAD_MAX 2048

static void star_isp_sleep_ms(unsigned int ms)
{
    struct timespec ts;

    ts.tv_sec = (time_t)(ms / 1000u);
    ts.tv_nsec = (long)(ms % 1000u) * 1000000L;
    (void)nanosleep(&ts, NULL);
}

typedef enum {
    IQ_FLAT,   /* value at offset 0, no bEnable and no enOpType */
    IQ_BOOL,   /* bEnable at offset 0 is itself the value */
    IQ_AUTOMAN /* bEnable, enOpType, auto[16], manual at manual_off */
} star_iq_shape_t;

typedef struct {
    const char *name; /* for diagnostics only */
    const char *get_sym;
    const char *set_sym;
    uint16_t payload;    /* wrapper's hardcoded payload size */
    uint16_t manual_off; /* where the value lives (0 for FLAT/BOOL) */
    uint8_t width;       /* 1, 2 or 4 bytes */
    uint8_t shape;       /* star_iq_shape_t */
    uint32_t mi_max;     /* MI's maximum for the field */
    uint32_t mi_unity;   /* MI value that means the same as raptor's 128 */

    /* Resolved on first use and cached. dlsym per call would work, but
     * these sit on rvd's control path and the symbol never changes. */
    i6_isp_cmd_fn fn_get;
    i6_isp_cmd_fn fn_set;
} star_iq_param_t;

enum {
    IQ_BRIGHTNESS,
    IQ_CONTRAST,
    IQ_SATURATION,
    IQ_SHARPNESS,
    IQ_SINTER,
    IQ_TEMPER,
    IQ_DEFOG,
    IQ_GRAY,
    IQ_EVCOMP,
    IQ_FLICKER,
    IQ_PARAM_COUNT
};

/*
 * Payload sizes are from disassembling the board's libmi_isp.so; manual
 * offsets are waybeam's, cross-checked against those sizes (see the
 * header comment). mi_unity is the MI value corresponding to raptor's
 * neutral 128, which is what keeps a default config from shifting the
 * image:
 *
 *   brightness/contrast  0..100, midpoint 50
 *   saturation           0..127 where 32 is unity gain (1X), *not* the
 *                        midpoint -- waybeam names this explicitly, and
 *                        a linear 0..255 -> 0..127 map would silently
 *                        double saturation at raptor's neutral
 *   sharpness, NR        0..255, midpoint 128
 *   EV compensation      0..200 where 100 is no compensation
 */
static star_iq_param_t g_iq[IQ_PARAM_COUNT] = {
    [IQ_BRIGHTNESS] = { "brightness", "MI_ISP_IQ_GetBrightness", "MI_ISP_IQ_SetBrightness", 76, 72,
                        4, IQ_AUTOMAN, 100, 50, NULL, NULL },
    [IQ_CONTRAST] = { "contrast", "MI_ISP_IQ_GetContrast", "MI_ISP_IQ_SetContrast", 76, 72, 4,
                      IQ_AUTOMAN, 100, 50, NULL, NULL },
    [IQ_SATURATION] = { "saturation", "MI_ISP_IQ_GetSaturation", "MI_ISP_IQ_SetSaturation", 416, 392,
                        1, IQ_AUTOMAN, 127, 32, NULL, NULL },
    [IQ_SHARPNESS] = { "sharpness", "MI_ISP_IQ_GetSharpness", "MI_ISP_IQ_SetSharpness", 1268, 1192,
                       1, IQ_AUTOMAN, 255, 128, NULL, NULL },
    /* Spatial (per-frame) luma noise reduction is raptor's "sinter". */
    [IQ_SINTER] = { "sinter", "MI_ISP_IQ_GetNRLuma", "MI_ISP_IQ_SetNRLuma", 112, 104, 1, IQ_AUTOMAN,
                    255, 128, NULL, NULL },
    /* Temporal noise reduction is raptor's "temper" -- MI calls it 3D NR. */
    [IQ_TEMPER] = { "temper", "MI_ISP_IQ_GetNR3D", "MI_ISP_IQ_SetNR3D", 1776, 1288, 1, IQ_AUTOMAN,
                    255, 128, NULL, NULL },
    [IQ_DEFOG] = { "defog", "MI_ISP_IQ_GetDefog", "MI_ISP_IQ_SetDefog", 28, 0, 4, IQ_BOOL, 1, 0,
                   NULL, NULL },
    [IQ_GRAY] = { "gray", "MI_ISP_IQ_GetColorToGray", "MI_ISP_IQ_SetColorToGray", 4, 0, 4, IQ_BOOL,
                  1, 0, NULL, NULL },
    [IQ_EVCOMP] = { "ae_comp", "MI_ISP_AE_GetEVComp", "MI_ISP_AE_SetEVComp", 8, 0, 4, IQ_FLAT, 200,
                    100, NULL, NULL },
    [IQ_FLICKER] = { "antiflicker", "MI_ISP_AE_GetFlicker", "MI_ISP_AE_SetFlicker", 4, 0, 4, IQ_FLAT,
                     3, 0, NULL, NULL },
};

/* ================================================================
 * GENERIC IQ FIELD ACCESS
 * ================================================================ */

static uint32_t star_iq_read(const uint8_t *buf, uint16_t off, uint8_t width)
{
    uint32_t v = 0;

    /* memcpy rather than a cast: the payload is a byte buffer and these
     * offsets are not guaranteed to be aligned for the wider types. */
    switch (width) {
    case 1:
        v = buf[off];
        break;
    case 2: {
        uint16_t t;
        memcpy(&t, buf + off, sizeof(t));
        v = t;
        break;
    }
    default: {
        uint32_t t;
        memcpy(&t, buf + off, sizeof(t));
        v = t;
        break;
    }
    }

    return v;
}

static void star_iq_write(uint8_t *buf, uint16_t off, uint8_t width, uint32_t val)
{
    switch (width) {
    case 1:
        buf[off] = (uint8_t)val;
        break;
    case 2: {
        uint16_t t = (uint16_t)val;
        memcpy(buf + off, &t, sizeof(t));
        break;
    }
    default:
        memcpy(buf + off, &val, sizeof(val));
        break;
    }
}

/* Resolve and cache a parameter's getter and setter. */
static int star_iq_resolve(star_state_t *st, star_iq_param_t *p)
{
    if (p->fn_get && p->fn_set)
        return RSS_OK;

    if (!st->isp_loaded || !st->isp.handle)
        return RSS_ERR_NOENT;

    if (p->payload > STAR_IQ_PAYLOAD_MAX) {
        HAL_LOG_ERR("isp: %s payload %u exceeds the %u-byte buffer", p->name, p->payload,
                    STAR_IQ_PAYLOAD_MAX);
        return RSS_ERR_INVAL;
    }

    p->fn_get = (i6_isp_cmd_fn)hal_symbol_load("i6_isp", st->isp.handle, p->get_sym);
    p->fn_set = (i6_isp_cmd_fn)hal_symbol_load("i6_isp", st->isp.handle, p->set_sym);
    if (!p->fn_get || !p->fn_set) {
        p->fn_get = NULL;
        p->fn_set = NULL;
        return RSS_ERR_NOTSUP;
    }

    return RSS_OK;
}

/*
 * Read the module's current payload. Callers modify one field of it and
 * hand it back to star_iq_store, so the ~1700 bytes of tuning we cannot
 * describe are preserved rather than zeroed.
 */
static int star_iq_fetch(star_state_t *st, int idx, uint8_t *buf)
{
    star_iq_param_t *p = &g_iq[idx];
    int ret;

    ret = star_iq_resolve(st, p);
    if (ret != RSS_OK)
        return ret;

    memset(buf, 0, p->payload);
    ret = p->fn_get(STAR_ISP_CHN, buf);
    if (ret) {
        HAL_LOG_WARN("isp: %s (%s) failed: %d", p->name, p->get_sym, ret);
        return RSS_ERR_IO;
    }

    return RSS_OK;
}

static int star_iq_store(star_state_t *st, int idx, uint8_t *buf)
{
    star_iq_param_t *p = &g_iq[idx];
    int ret;

    (void)st;
    ret = p->fn_set(STAR_ISP_CHN, buf);
    if (ret) {
        HAL_LOG_WARN("isp: %s (%s) failed: %d", p->name, p->set_sym, ret);
        return RSS_ERR_IO;
    }

    return RSS_OK;
}

/*
 * Map raptor's 0..255 onto MI's range, piecewise so that neutral lands
 * exactly on MI's own unity value. A single linear map would not: with
 * saturation's unity at 32 of 127, linear scaling puts raptor's neutral
 * at 64 -- twice unity gain -- and every default config would boost
 * colour.
 */
static uint32_t star_iq_scale(int val, uint32_t unity, uint32_t max)
{
    if (val <= 0)
        return 0;
    if (val >= 255)
        return max;
    if (val == STAR_ISP_NEUTRAL || unity == 0 || unity >= max)
        return unity;

    if (val < STAR_ISP_NEUTRAL)
        return (uint32_t)(((uint64_t)val * unity) / STAR_ISP_NEUTRAL);

    return unity + (uint32_t)(((uint64_t)(val - STAR_ISP_NEUTRAL) * (max - unity)) /
                              (255 - STAR_ISP_NEUTRAL));
}

/* Inverse of star_iq_scale, for the getters. */
static uint8_t star_iq_unscale(uint32_t mi, uint32_t unity, uint32_t max)
{
    if (max == 0 || mi >= max)
        return 255;
    if (mi == 0)
        return 0;
    if (unity == 0 || unity >= max)
        return STAR_ISP_NEUTRAL;
    if (mi == unity)
        return STAR_ISP_NEUTRAL;

    if (mi < unity)
        return (uint8_t)(((uint64_t)mi * STAR_ISP_NEUTRAL) / unity);

    return (uint8_t)(STAR_ISP_NEUTRAL +
                     ((uint64_t)(mi - unity) * (255 - STAR_ISP_NEUTRAL)) / (max - unity));
}

/*
 * Apply one of raptor's 0..255 scalars.
 *
 * Neutral restores auto and leaves the manual field alone -- see THE
 * TUNING BINARY OUTRANKS THE CONFIG. bEnable is never touched: if the
 * tuning binary disabled a module, re-enabling it behind the tuner's
 * back is not this layer's call.
 */
static int star_iq_set_scalar(void *ctx, int idx, int val)
{
    star_state_t *st = star_state(ctx);
    star_iq_param_t *p = &g_iq[idx];
    uint8_t buf[STAR_IQ_PAYLOAD_MAX];
    uint32_t mi_val;
    int ret;

    if (!st)
        return RSS_ERR_INVAL;

    ret = star_iq_fetch(st, idx, buf);
    if (ret != RSS_OK)
        return ret;

    if (p->shape == IQ_AUTOMAN) {
        if (val == STAR_ISP_NEUTRAL) {
            star_iq_write(buf, STAR_ISP_OPTYPE_OFF, 4, STAR_ISP_OP_AUTO);
            HAL_LOG_DBG("isp: %s left to the tuning file (auto)", p->name);
            return star_iq_store(st, idx, buf);
        }
        star_iq_write(buf, STAR_ISP_OPTYPE_OFF, 4, STAR_ISP_OP_MANUAL);
    }

    mi_val = star_iq_scale(val, p->mi_unity, p->mi_max);
    star_iq_write(buf, p->manual_off, p->width, mi_val);

    ret = star_iq_store(st, idx, buf);
    if (ret == RSS_OK)
        HAL_LOG_DBG("isp: %s = %d (MI %u/%u)", p->name, val, mi_val, p->mi_max);

    return ret;
}

static int star_iq_get_scalar(void *ctx, int idx, uint8_t *out)
{
    star_state_t *st = star_state(ctx);
    star_iq_param_t *p = &g_iq[idx];
    uint8_t buf[STAR_IQ_PAYLOAD_MAX];
    int ret;

    if (!st || !out)
        return RSS_ERR_INVAL;

    ret = star_iq_fetch(st, idx, buf);
    if (ret != RSS_OK)
        return ret;

    /*
     * A module in auto mode has no single value to report, and its
     * manual field holds whatever was last written there. Neutral is
     * the honest answer, and it round-trips: it is also the value that
     * puts the module back into auto.
     */
    if (p->shape == IQ_AUTOMAN &&
        star_iq_read(buf, STAR_ISP_OPTYPE_OFF, 4) == STAR_ISP_OP_AUTO) {
        *out = STAR_ISP_NEUTRAL;
        return RSS_OK;
    }

    *out = star_iq_unscale(star_iq_read(buf, p->manual_off, p->width), p->mi_unity, p->mi_max);
    return RSS_OK;
}

/* Raw field access for the enum- and bool-valued parameters, which have
 * no 0..255 scale to map through. */
static int star_iq_set_raw(void *ctx, int idx, uint32_t raw)
{
    star_state_t *st = star_state(ctx);
    star_iq_param_t *p = &g_iq[idx];
    uint8_t buf[STAR_IQ_PAYLOAD_MAX];
    int ret;

    if (!st)
        return RSS_ERR_INVAL;

    ret = star_iq_fetch(st, idx, buf);
    if (ret != RSS_OK)
        return ret;

    star_iq_write(buf, p->manual_off, p->width, raw);
    ret = star_iq_store(st, idx, buf);
    if (ret == RSS_OK)
        HAL_LOG_DBG("isp: %s = %u", p->name, raw);

    return ret;
}

static int star_iq_get_raw(void *ctx, int idx, uint32_t *raw)
{
    star_state_t *st = star_state(ctx);
    star_iq_param_t *p = &g_iq[idx];
    uint8_t buf[STAR_IQ_PAYLOAD_MAX];
    int ret;

    if (!st || !raw)
        return RSS_ERR_INVAL;

    ret = star_iq_fetch(st, idx, buf);
    if (ret != RSS_OK)
        return ret;

    *raw = star_iq_read(buf, p->manual_off, p->width);
    return RSS_OK;
}

/* ================================================================
 * BRING-UP: TUNING BINARY AND 3A
 * ================================================================ */

/*
 * Wait for the IQ parameter store to come up.
 *
 * Returns RSS_OK once the flag is set. A timeout is reported but is not
 * treated as fatal by any caller: the worst case is the bin load below
 * failing with the kernel's own "channel not created", which is a
 * clearer diagnostic than anything invented here.
 */
static int star_isp_wait_ready(star_state_t *st)
{
    unsigned int waited = 0;

    if (!st->isp.fnGetParaInitStatus)
        return RSS_ERR_NOTSUP;

    while (waited < STAR_ISP_READY_TIMEOUT_MS) {
        i6_isp_parainit status;

        memset(&status, 0, sizeof(status));
        if (st->isp.fnGetParaInitStatus(STAR_ISP_CHN, &status) == 0 && status.ready) {
            HAL_LOG_DBG("isp: parameter store ready after %u ms", waited);
            return RSS_OK;
        }

        star_isp_sleep_ms(STAR_ISP_READY_POLL_MS);
        waited += STAR_ISP_READY_POLL_MS;
    }

    HAL_LOG_WARN("isp: parameter store not ready after %u ms; continuing",
                 STAR_ISP_READY_TIMEOUT_MS);
    return RSS_ERR_TIMEOUT;
}

/*
 * Start the vendor 3A algorithms.
 *
 * The two-call sequence with {1,0,0} then {1,1,0} is reproduced from
 * both references (divinus i6_hal.c, waybeam star6e_pipeline.c:270-284),
 * neither of which documents the parameter block -- 13 ints of which
 * only the first three are ever non-zero. Since VPE already auto-starts
 * CUS3A, this is really a *re*-start after the bin load replaced the
 * tables underneath it.
 */
static void star_isp_enable_3a(star_state_t *st)
{
    i6_isp_p3a params;

    if (!st->isp.fnEnableUserspace3A)
        return;

    memset(&params, 0, sizeof(params));
    params.params[0] = 1;
    if (st->isp.fnEnableUserspace3A(STAR_ISP_CHN, &params))
        HAL_LOG_WARN("isp: CUS3A enable (phase 1) failed");

    memset(&params, 0, sizeof(params));
    params.params[0] = 1;
    params.params[1] = 1;
    if (st->isp.fnEnableUserspace3A(STAR_ISP_CHN, &params))
        HAL_LOG_WARN("isp: CUS3A enable (phase 2) failed");
}

/*
 * Work out which tuning binary to load.
 *
 * An explicit [sensor] iq_file wins. Otherwise the sensor's own name
 * gives it away: OpenIPC installs one binary per sensor as
 * /etc/sensors/<name>.bin, named exactly as the kernel module is
 * (sensor_gc4653_mipi.ko -> gc4653.bin), which is the same string 2e's
 * sensor_detect already produces. So on a stock OpenIPC image the
 * correct tuning file is found with nothing declared anywhere.
 *
 * Returns true and fills out[] when a readable file was found.
 */
static bool star_isp_resolve_iq(star_state_t *st, const rss_sensor_config_t *cfg, char *out,
                                size_t len)
{
    const char *name = NULL;
    char lower[64];
    size_t i;

    out[0] = '\0';

    if (cfg && cfg->iq_file[0]) {
        if (access(cfg->iq_file, R_OK) == 0) {
            snprintf(out, len, "%s", cfg->iq_file);
            HAL_LOG_INFO("isp: tuning file %s (from config)", out);
            return true;
        }
        HAL_LOG_WARN("isp: configured tuning file %s is not readable; trying the sensor default",
                     cfg->iq_file);
    }

    /*
     * cfg->name is preferred over MI's plane.sensName because the file
     * is named after the driver module, which is what cfg->name came
     * from. plane.sensName is the same identity as MI reports it
     * ("GC4653"), and is the fallback when nothing named the sensor.
     */
    if (cfg && cfg->name[0])
        name = cfg->name;
    else if (st->snr_enabled && st->plane.sensName[0])
        name = st->plane.sensName;

    if (!name) {
        HAL_LOG_WARN("isp: no sensor name and no iq_file; the generic vendor tuning stays loaded");
        return false;
    }

    for (i = 0; i + 1 < sizeof(lower) && name[i]; i++)
        lower[i] = (char)tolower((unsigned char)name[i]);
    lower[i] = '\0';

    snprintf(out, len, "/etc/sensors/%s.bin", lower);
    if (access(out, R_OK) == 0) {
        HAL_LOG_INFO("isp: tuning file %s (from sensor name)", out);
        return true;
    }

    HAL_LOG_WARN("isp: no tuning file at %s; the generic vendor tuning stays loaded "
                 "and colour will be off",
                 out);
    out[0] = '\0';
    return false;
}

int star_isp_cap_exposure(star_state_t *st, unsigned int fps)
{
    i6_isp_exp limit;
    unsigned int frame_us;
    int ret;

    if (!st || !st->isp_loaded || !fps)
        return RSS_ERR_INVAL;

    memset(&limit, 0, sizeof(limit));
    ret = st->isp.fnGetExposureLimit(STAR_ISP_CHN, &limit);
    if (ret) {
        HAL_LOG_WARN("isp: MI_ISP_AE_GetExposureLimit failed: %d", ret);
        return RSS_ERR_IO;
    }

    /*
     * A cold-booted AE has not processed enough frames to publish its
     * limits and answers all zeros; capping against that would write a
     * garbage floor. waybeam polls for up to 500 ms here
     * (pipeline_common.c:138-152). The bin load ahead of this already
     * waited on the parameter store, so one retry window is enough.
     */
    if (limit.maxShutterUs == 0 && limit.maxSensorGain == 0) {
        unsigned int waited;

        for (waited = 0; waited < 500; waited += 10) {
            star_isp_sleep_ms(10);
            memset(&limit, 0, sizeof(limit));
            if (st->isp.fnGetExposureLimit(STAR_ISP_CHN, &limit) == 0 &&
                (limit.maxShutterUs || limit.maxSensorGain))
                break;
        }
        if (limit.maxShutterUs == 0) {
            HAL_LOG_WARN("isp: AE published no exposure limits; shutter left uncapped");
            return RSS_ERR_TIMEOUT;
        }
    }

    frame_us = 1000000u / fps;
    if (limit.maxShutterUs <= frame_us) {
        HAL_LOG_DBG("isp: AE max shutter %u us already within the %u us frame period",
                    limit.maxShutterUs, frame_us);
        return RSS_OK;
    }

    HAL_LOG_INFO("isp: capping AE max shutter %u -> %u us to hold %u fps", limit.maxShutterUs,
                 frame_us, fps);
    limit.maxShutterUs = frame_us;

    ret = st->isp.fnSetExposureLimit(STAR_ISP_CHN, &limit);
    if (ret) {
        HAL_LOG_WARN("isp: MI_ISP_AE_SetExposureLimit failed: %d", ret);
        return RSS_ERR_IO;
    }

    /*
     * SetExposureLimit constrains the AE algorithm; it does not touch
     * the shutter register the sensor is already running with. If the
     * tuning binary brought the sensor up with an exposure longer than
     * the frame period, the sensor stays slow until something makes its
     * driver recompute timing -- and MI_SNR_SetFps is that something.
     * waybeam calls this the cold-boot fix (star6e_pipeline.c:2094).
     */
    if (st->snr.fnSetFramerate && st->snr.fnSetFramerate(STAR_SNR_INDEX, fps))
        HAL_LOG_WARN("isp: MI_SNR_SetFps(%u) after the exposure cap failed", fps);

    return RSS_OK;
}

void star_isp_bringup(star_state_t *st, const rss_sensor_config_t *cfg)
{
    char path[sizeof(st->iq_file)];
    int ret;

    if (!st)
        return;

    /*
     * Everything below is best-effort. An untuned image streams; a
     * pipeline aborted over a tuning file does not, and phase 2 shipped
     * for weeks with the generic tuning precisely because a wrong-looking
     * image is a defect rather than an outage.
     */
    ret = i6_isp_load(&st->isp);
    if (ret != RSS_OK) {
        HAL_LOG_WARN("isp: MI_ISP unavailable (%d); no tuning or 3A control this run", ret);
        return;
    }
    st->isp_loaded = true;

    star_isp_wait_ready(st);

    if (star_isp_resolve_iq(st, cfg, path, sizeof(path))) {
        /*
         * Hand the running 3A off before swapping the tables it is
         * reading, then start it again afterwards. Both references do
         * this in the same order; neither explains it, and the failure
         * mode of skipping it has not been observed here.
         */
        if (st->isp.fnDisableUserspace3A(STAR_ISP_CHN))
            HAL_LOG_WARN("isp: MI_ISP_DisableUserspace3A failed; loading anyway");

        star_isp_wait_ready(st);

        ret = st->isp.fnLoadChannelConfig(STAR_ISP_CHN, path, STAR_IQ_LOAD_KEY);
        if (ret) {
            HAL_LOG_WARN("isp: loading %s failed: %d; the generic vendor tuning stays in "
                         "effect and colour will be off",
                         path, ret);
        } else {
            snprintf(st->iq_file, sizeof(st->iq_file), "%s", path);
            HAL_LOG_INFO("isp: loaded tuning file %s", path);
        }

        star_isp_enable_3a(st);
    }

    /* Worth doing whether or not a tuning file loaded: the limits come
     * from whichever tuning is in effect, and neither is obliged to suit
     * the framerate this pipeline asked for. */
    star_isp_cap_exposure(st, st->fps);
}

void star_isp_teardown(star_state_t *st)
{
    size_t i;

    if (!st || !st->isp_loaded)
        return;

    /* The cached function pointers belong to the handle about to be
     * closed, so they have to go with it. */
    for (i = 0; i < IQ_PARAM_COUNT; i++) {
        g_iq[i].fn_get = NULL;
        g_iq[i].fn_set = NULL;
    }

    i6_isp_unload(&st->isp);
    st->isp_loaded = false;
    st->iq_file[0] = '\0';
}

/* ================================================================
 * OPS
 * ================================================================ */

int hal_isp_set_brightness(void *ctx, int val)
{
    return star_iq_set_scalar(ctx, IQ_BRIGHTNESS, val);
}

int hal_isp_set_contrast(void *ctx, int val)
{
    return star_iq_set_scalar(ctx, IQ_CONTRAST, val);
}

int hal_isp_set_saturation(void *ctx, int val)
{
    return star_iq_set_scalar(ctx, IQ_SATURATION, val);
}

int hal_isp_set_sharpness(void *ctx, int val)
{
    return star_iq_set_scalar(ctx, IQ_SHARPNESS, val);
}

int hal_isp_set_sinter_strength(void *ctx, int val)
{
    return star_iq_set_scalar(ctx, IQ_SINTER, val);
}

int hal_isp_set_temper_strength(void *ctx, int val)
{
    return star_iq_set_scalar(ctx, IQ_TEMPER, val);
}

int hal_isp_set_ae_comp(void *ctx, int val)
{
    return star_iq_set_scalar(ctx, IQ_EVCOMP, val);
}

int hal_isp_set_defog(void *ctx, int enable)
{
    return star_iq_set_raw(ctx, IQ_DEFOG, enable ? 1u : 0u);
}

int hal_isp_get_brightness(void *ctx, uint8_t *val)
{
    return star_iq_get_scalar(ctx, IQ_BRIGHTNESS, val);
}

int hal_isp_get_contrast(void *ctx, uint8_t *val)
{
    return star_iq_get_scalar(ctx, IQ_CONTRAST, val);
}

int hal_isp_get_saturation(void *ctx, uint8_t *val)
{
    return star_iq_get_scalar(ctx, IQ_SATURATION, val);
}

int hal_isp_get_sharpness(void *ctx, uint8_t *val)
{
    return star_iq_get_scalar(ctx, IQ_SHARPNESS, val);
}

int hal_isp_get_sinter_strength(void *ctx, uint8_t *val)
{
    return star_iq_get_scalar(ctx, IQ_SINTER, val);
}

int hal_isp_get_temper_strength(void *ctx, uint8_t *val)
{
    return star_iq_get_scalar(ctx, IQ_TEMPER, val);
}

int hal_isp_get_ae_comp(void *ctx, int *val)
{
    uint8_t v;
    int ret;

    if (!val)
        return RSS_ERR_INVAL;

    ret = star_iq_get_scalar(ctx, IQ_EVCOMP, &v);
    if (ret == RSS_OK)
        *val = v;

    return ret;
}

/*
 * Anti-flicker. raptor's OFF/50HZ/60HZ are 0/1/2 and MI's flicker enum
 * is bounded at 3, so the two line up position for position. Passed
 * through rather than translated, with the range checked so a future
 * raptor value cannot land on an MI mode by accident.
 *
 * Applied unconditionally, unlike the tuning scalars: mains frequency
 * is a property of where the camera is installed, which a tuning file
 * shipped with a sensor cannot know.
 */
int hal_isp_set_antiflicker(void *ctx, rss_antiflicker_t mode)
{
    if ((unsigned int)mode > RSS_ANTIFLICKER_60HZ) {
        HAL_LOG_WARN("isp: antiflicker mode %d out of range", (int)mode);
        return RSS_ERR_INVAL;
    }

    return star_iq_set_raw(ctx, IQ_FLICKER, (uint32_t)mode);
}

int hal_isp_get_antiflicker(void *ctx, rss_antiflicker_t *mode)
{
    uint32_t raw;
    int ret;

    if (!mode)
        return RSS_ERR_INVAL;

    ret = star_iq_get_raw(ctx, IQ_FLICKER, &raw);
    if (ret == RSS_OK)
        *mode = (rss_antiflicker_t)(raw > RSS_ANTIFLICKER_60HZ ? RSS_ANTIFLICKER_OFF : raw);

    return ret;
}

/*
 * Gain ceilings.
 *
 * MI keeps both in the AE exposure-limit struct, so each setter is a
 * read-modify-write of the same 32 bytes star_isp_cap_exposure uses --
 * which is the point: writing the struct wholesale here would undo that
 * shutter cap.
 *
 * The units are MI's own and are not raptor's 0..255: rvd's defaults
 * (max_again 160, max_dgain 80) are Ingenic gain codes. They are passed
 * through unscaled because there is no published conversion, and a
 * fabricated one would be worse than a documented pass-through -- so
 * treat these two keys as MI-native on this platform.
 */
static int star_isp_set_gain_limit(void *ctx, bool sensor_gain, int gain)
{
    star_state_t *st = star_state(ctx);
    i6_isp_exp limit;
    int ret;

    if (!st || !st->isp_loaded)
        return RSS_ERR_NOENT;
    if (gain < 0)
        return RSS_ERR_INVAL;

    memset(&limit, 0, sizeof(limit));
    ret = st->isp.fnGetExposureLimit(STAR_ISP_CHN, &limit);
    if (ret) {
        HAL_LOG_WARN("isp: MI_ISP_AE_GetExposureLimit failed: %d", ret);
        return RSS_ERR_IO;
    }

    if (sensor_gain)
        limit.maxSensorGain = (unsigned int)gain;
    else
        limit.maxIspGain = (unsigned int)gain;

    ret = st->isp.fnSetExposureLimit(STAR_ISP_CHN, &limit);
    if (ret) {
        HAL_LOG_WARN("isp: MI_ISP_AE_SetExposureLimit failed: %d", ret);
        return RSS_ERR_IO;
    }

    HAL_LOG_DBG("isp: max %s gain = %d", sensor_gain ? "sensor" : "isp", gain);
    return RSS_OK;
}

static int star_isp_get_gain_limit(void *ctx, bool sensor_gain, uint32_t *gain)
{
    star_state_t *st = star_state(ctx);
    i6_isp_exp limit;
    int ret;

    if (!st || !st->isp_loaded)
        return RSS_ERR_NOENT;
    if (!gain)
        return RSS_ERR_INVAL;

    memset(&limit, 0, sizeof(limit));
    ret = st->isp.fnGetExposureLimit(STAR_ISP_CHN, &limit);
    if (ret) {
        HAL_LOG_WARN("isp: MI_ISP_AE_GetExposureLimit failed: %d", ret);
        return RSS_ERR_IO;
    }

    *gain = sensor_gain ? limit.maxSensorGain : limit.maxIspGain;
    return RSS_OK;
}

int hal_isp_set_max_again(void *ctx, int gain)
{
    return star_isp_set_gain_limit(ctx, true, gain);
}

int hal_isp_set_max_dgain(void *ctx, int gain)
{
    return star_isp_set_gain_limit(ctx, false, gain);
}

int hal_isp_get_max_again(void *ctx, uint32_t *gain)
{
    return star_isp_get_gain_limit(ctx, true, gain);
}

int hal_isp_get_max_dgain(void *ctx, uint32_t *gain)
{
    return star_isp_get_gain_limit(ctx, false, gain);
}

/*
 * Day/night.
 *
 * On MI this is the colour-to-gray switch and nothing more. That is the
 * ISP half of a day/night transition -- under IR illumination the colour
 * channels carry no useful chroma, so monochrome output is both cleaner
 * and what the scene actually contains. Driving the physical IR-cut
 * filter is phase 6's GPIO work; this op does not pretend to do it, and
 * a board with no IR-cut wiring still benefits from the switch.
 */
int hal_isp_set_running_mode(void *ctx, rss_isp_mode_t mode)
{
    star_state_t *st = star_state(ctx);
    int ret;

    if (!st)
        return RSS_ERR_INVAL;

    ret = star_iq_set_raw(ctx, IQ_GRAY, mode == RSS_ISP_NIGHT ? 1u : 0u);
    if (ret == RSS_OK) {
        st->gray = (mode == RSS_ISP_NIGHT);
        HAL_LOG_INFO("isp: %s mode", st->gray ? "night (monochrome)" : "day (colour)");
    }

    return ret;
}

int hal_isp_get_running_mode(void *ctx, rss_isp_mode_t *mode)
{
    uint32_t raw;
    int ret;

    if (!mode)
        return RSS_ERR_INVAL;

    ret = star_iq_get_raw(ctx, IQ_GRAY, &raw);
    if (ret == RSS_OK)
        *mode = raw ? RSS_ISP_NIGHT : RSS_ISP_DAY;

    return ret;
}

/*
 * Mirror and flip.
 *
 * Done in the sensor, not the ISP: MI_SNR_SetOrien takes both at once,
 * so each op has to supply the other axis from tracked state. The
 * initial values come from the config at bring-up
 * (star_sensor_bringup), and these keep that state in step when
 * something changes one at runtime.
 */
static int star_isp_set_orien(star_state_t *st)
{
    int ret;

    if (!st->snr.fnSetOrientation)
        return RSS_ERR_NOTSUP;

    ret = st->snr.fnSetOrientation(STAR_SNR_INDEX, st->hflip ? 1 : 0, st->vflip ? 1 : 0);
    if (ret) {
        HAL_LOG_WARN("isp: MI_SNR_SetOrien(mirror=%d, flip=%d) failed: %d", st->hflip, st->vflip,
                     ret);
        return RSS_ERR_IO;
    }

    HAL_LOG_DBG("isp: orientation mirror=%d flip=%d", st->hflip, st->vflip);
    return RSS_OK;
}

int hal_isp_set_hflip(void *ctx, int enable)
{
    star_state_t *st = star_state(ctx);
    bool prev;
    int ret;

    if (!st)
        return RSS_ERR_INVAL;

    prev = st->hflip;
    st->hflip = enable ? true : false;
    ret = star_isp_set_orien(st);
    if (ret != RSS_OK)
        st->hflip = prev;

    return ret;
}

int hal_isp_set_vflip(void *ctx, int enable)
{
    star_state_t *st = star_state(ctx);
    bool prev;
    int ret;

    if (!st)
        return RSS_ERR_INVAL;

    prev = st->vflip;
    st->vflip = enable ? true : false;
    ret = star_isp_set_orien(st);
    if (ret != RSS_OK)
        st->vflip = prev;

    return ret;
}

int hal_isp_get_hvflip(void *ctx, int *hflip, int *vflip)
{
    star_state_t *st = star_state(ctx);

    if (!st)
        return RSS_ERR_INVAL;

    /* Tracked rather than queried -- MI_SNR_SetOrien has no getter. */
    if (hflip)
        *hflip = st->hflip ? 1 : 0;
    if (vflip)
        *vflip = st->vflip ? 1 : 0;

    return RSS_OK;
}
