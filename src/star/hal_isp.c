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
#include <stdlib.h>
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

/*
 * Budget for the early opportunistic attempt, made the moment a VPE port
 * is enabled. Short on purpose: frames need about one frame period to
 * start, so a ready ISP answers well inside this, and an unready one must
 * not spend the full timeout here only for the attempt at encoder start
 * to spend it again.
 */
#define STAR_ISP_READY_QUICK_MS 400

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

    /*
     * Value requested before the ISP would accept it, flushed by
     * star_isp_tune_when_ready. rvd applies its whole [image] block
     * during pipeline *construction*, well before any VPE port is
     * enabled, so without this queue every one of those calls fails and
     * the operator's settings are silently lost. Flushing after the
     * tuning binary loads is also the only correct order -- applied
     * before, the load would overwrite them.
     *
     * Recorded whether or not it could be applied straight away, and
     * *not* cleared by the flush: a tuning reload resets each module to
     * whatever the binary says, so the last value asked for is also the
     * value a re-tune has to put back. Without that, the first hot
     * restart silently reverts every knob the operator had set.
     *
     * Lives in the table beside the cached symbols, on the same
     * single-instance assumption, and is cleared by star_isp_teardown.
     */
    int pending;
    bool has_pending;
    bool pending_is_raw; /* set via star_iq_set_raw, not _set_scalar */
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
static int star_iq_apply_scalar(star_state_t *st, int idx, int val)
{
    star_iq_param_t *p = &g_iq[idx];
    uint8_t buf[STAR_IQ_PAYLOAD_MAX];
    uint32_t mi_val;
    int ret;

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

/*
 * Queue-or-apply. Splitting this from star_iq_apply_scalar lets the
 * flush drain the queue without re-entering it, and keeps the "is the
 * ISP reachable yet" question in exactly one place per direction.
 */
static int star_iq_set_scalar(void *ctx, int idx, int val)
{
    star_state_t *st = star_state(ctx);
    star_iq_param_t *p = &g_iq[idx];

    if (!st)
        return RSS_ERR_INVAL;

    /* Recorded first and unconditionally, so a re-tune can put it back
     * whether or not it reached MI on this attempt. */
    p->pending = val;
    p->has_pending = true;
    p->pending_is_raw = false;

    if (!st->isp_tuned) {
        HAL_LOG_DBG("isp: %s = %d queued until the ISP is up", p->name, val);
        return RSS_OK;
    }

    return star_iq_apply_scalar(st, idx, val);
}

static int star_iq_get_scalar(void *ctx, int idx, uint8_t *out)
{
    star_state_t *st = star_state(ctx);
    star_iq_param_t *p = &g_iq[idx];
    uint8_t buf[STAR_IQ_PAYLOAD_MAX];
    int ret;

    if (!st || !out)
        return RSS_ERR_INVAL;

    /* Report what was asked for while the ISP cannot be read, so a
     * set/get pair is consistent even before the pipeline runs. */
    if (!st->isp_tuned) {
        *out = p->has_pending ? (uint8_t)p->pending : (uint8_t)STAR_ISP_NEUTRAL;
        return RSS_OK;
    }

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
static int star_iq_apply_raw(star_state_t *st, int idx, uint32_t raw)
{
    star_iq_param_t *p = &g_iq[idx];
    uint8_t buf[STAR_IQ_PAYLOAD_MAX];
    int ret;

    ret = star_iq_fetch(st, idx, buf);
    if (ret != RSS_OK)
        return ret;

    star_iq_write(buf, p->manual_off, p->width, raw);
    ret = star_iq_store(st, idx, buf);
    if (ret == RSS_OK)
        HAL_LOG_DBG("isp: %s = %u", p->name, raw);

    return ret;
}

static int star_iq_set_raw(void *ctx, int idx, uint32_t raw)
{
    star_state_t *st = star_state(ctx);
    star_iq_param_t *p = &g_iq[idx];

    if (!st)
        return RSS_ERR_INVAL;

    p->pending = (int)raw;
    p->has_pending = true;
    p->pending_is_raw = true;

    if (!st->isp_tuned) {
        HAL_LOG_DBG("isp: %s = %u queued until the ISP is up", p->name, raw);
        return RSS_OK;
    }

    return star_iq_apply_raw(st, idx, raw);
}

static int star_iq_get_raw(void *ctx, int idx, uint32_t *raw)
{
    star_state_t *st = star_state(ctx);
    star_iq_param_t *p = &g_iq[idx];
    uint8_t buf[STAR_IQ_PAYLOAD_MAX];
    int ret;

    if (!st || !raw)
        return RSS_ERR_INVAL;

    if (!st->isp_tuned) {
        *raw = p->has_pending ? (uint32_t)p->pending : 0u;
        return RSS_OK;
    }

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
 * Distinguishes "the call failed" from "the flag is not set yet", which
 * matters more than it looks: the first board run of this file reported
 * only a flat "not ready after 2000 ms" while every underlying call was
 * in fact returning 6, and collapsing those two cases into one message
 * is what made a plain ordering bug look like a timing problem.
 */
static int star_isp_wait_ready(star_state_t *st, unsigned int timeout_ms, bool verbose)
{
    unsigned int waited = 0;
    int last_ret = 0;

    if (!st->isp.fnGetParaInitStatus)
        return RSS_ERR_NOTSUP;

    while (waited < timeout_ms) {
        i6_isp_parainit status;
        int ret;

        memset(&status, 0, sizeof(status));
        ret = st->isp.fnGetParaInitStatus(STAR_ISP_CHN, &status);
        if (ret == 0 && status.ready) {
            HAL_LOG_DBG("isp: parameter store ready after %u ms", waited);
            return RSS_OK;
        }
        last_ret = ret;

        star_isp_sleep_ms(STAR_ISP_READY_POLL_MS);
        waited += STAR_ISP_READY_POLL_MS;
    }

    if (!verbose) {
        /* An early opportunistic attempt; a later one will retry. */
        HAL_LOG_DBG("isp: parameter store not up yet after %u ms (last return %d)", timeout_ms,
                    last_ret);
    } else if (last_ret) {
        HAL_LOG_WARN("isp: MI_ISP_IQ_GetParaInitStatus keeps returning %d after %u ms -- the ISP "
                     "is not answering on VPE channel %d, so tuning is being skipped",
                     last_ret, timeout_ms, STAR_ISP_CHN);
    } else {
        HAL_LOG_WARN("isp: parameter store still not ready after %u ms; skipping tuning",
                     timeout_ms);
    }

    return RSS_ERR_TIMEOUT;
}

/*
 * Restart the vendor 3A algorithms after the tuning binary has replaced
 * the tables underneath them. VPE already auto-starts CUS3A, so this is
 * a re-start, not a start.
 *
 * AE alone first, then AE+AWB: the order waybeam uses
 * (star6e_pipeline.c:270-284) and the order CUS3A's own bring-up follows.
 * AF stays off -- these are fixed-focus modules.
 *
 * The parameter block is three bytes, not three ints; getting that wrong
 * is what silently ran this pipeline without auto white balance. See
 * i6_isp_p3a for the disassembly that settles the field widths.
 */
static void star_isp_enable_3a(star_state_t *st)
{
    i6_isp_p3a params;

    if (!st->isp.fnCus3aEnable)
        return;

    memset(&params, 0, sizeof(params));
    params.ae = 1;
    if (st->isp.fnCus3aEnable(STAR_ISP_CHN, &params))
        HAL_LOG_WARN("isp: CUS3A enable (AE) failed");

    memset(&params, 0, sizeof(params));
    params.ae = 1;
    params.awb = 1;
    if (st->isp.fnCus3aEnable(STAR_ISP_CHN, &params))
        HAL_LOG_WARN("isp: CUS3A enable (AE+AWB) failed");

    /*
     * Worth its own line because the driver prints the flags it received,
     * so the two are checkable against each other: MI's own
     * "[MI_ISP_CUS3A_Enable] AE = 1, AWB = 1" beside this line is the
     * proof white balance is running. An AWB = 0 there means the block's
     * field widths have drifted again, and the picture will have a colour
     * cast that looks exactly like a missing tuning file.
     */
    HAL_LOG_INFO("isp: CUS3A restarted with AE and AWB (AF off)");
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

/*
 * Read the AE's exposure limits, waiting for it to publish them.
 *
 * A cold-booted AE has not processed enough frames to publish its limits
 * and answers all zeros; capping or clamping against that would treat a
 * garbage floor as calibration. waybeam polls for up to 500 ms here
 * (pipeline_common.c:138-152). The bin load ahead of this already waited
 * on the parameter store, so one retry window is enough.
 */
static int star_isp_read_limits(star_state_t *st, i6_isp_exp *limit)
{
    unsigned int waited;
    int ret;

    memset(limit, 0, sizeof(*limit));
    ret = st->isp.fnGetExposureLimit(STAR_ISP_CHN, limit);
    if (ret) {
        HAL_LOG_WARN("isp: MI_ISP_AE_GetExposureLimit failed: %d", ret);
        return RSS_ERR_IO;
    }
    if (limit->maxShutterUs || limit->maxSensorGain)
        return RSS_OK;

    for (waited = 0; waited < 500; waited += 10) {
        star_isp_sleep_ms(10);
        memset(limit, 0, sizeof(*limit));
        if (st->isp.fnGetExposureLimit(STAR_ISP_CHN, limit) == 0 &&
            (limit->maxShutterUs || limit->maxSensorGain))
            return RSS_OK;
    }

    return RSS_ERR_TIMEOUT;
}

/*
 * Record the tuning's own gain ceilings, once, before the config knobs
 * land on top of them.
 *
 * The ordering is the whole point and it is easy to get wrong:
 * star_isp_flush_pending runs before star_isp_cap_exposure, so by the time
 * anything else reads this struct a requested ceiling is already in it --
 * and because every writer here is a read-modify-write, one bad ceiling
 * propagates into every later write. Snapshotting after the flush would
 * record the overwrite and call it calibration.
 */
static void star_isp_snapshot_bin_limits(star_state_t *st)
{
    i6_isp_exp limit;

    if (!st || !st->isp_loaded)
        return;
    if (!st->isp.fnGetExposureLimit || !st->isp.fnSetExposureLimit)
        return;

    if (star_isp_read_limits(st, &limit) != RSS_OK) {
        HAL_LOG_WARN("isp: AE published no exposure limits; gain ceilings go on unchecked");
        return;
    }

    st->bin_min_sensor_gain = limit.minSensorGain;
    st->bin_max_sensor_gain = limit.maxSensorGain;
    st->bin_min_isp_gain = limit.minIspGain;
    st->bin_max_isp_gain = limit.maxIspGain;

    /*
     * INFO because this is the line every night-mode threshold gets
     * calibrated against. total_gain cannot exceed maxSensorGain *
     * maxIspGain / 1024, so this is what says whether a given night_gain
     * is reachable on this board at all -- and a night_gain that is not
     * reachable means auto night mode simply never triggers.
     */
    HAL_LOG_INFO("isp: AE tuning limits (x1024): sensor gain %u..%u, isp gain %u..%u, "
                 "shutter %u..%u us -- so total_gain tops out at %llu",
                 limit.minSensorGain, limit.maxSensorGain, limit.minIspGain, limit.maxIspGain,
                 limit.minShutterUs, limit.maxShutterUs,
                 (unsigned long long)limit.maxSensorGain *
                         (limit.maxIspGain ? limit.maxIspGain : 1024u) / 1024u);
}

int star_isp_cap_exposure(star_state_t *st, unsigned int fps)
{
    i6_isp_exp limit;
    unsigned int frame_us;
    int ret;

    if (!st || !st->isp_loaded || !fps)
        return RSS_ERR_INVAL;
    if (!st->isp.fnGetExposureLimit || !st->isp.fnSetExposureLimit)
        return RSS_ERR_NOTSUP;

    /*
     * Divinus parity, for bisecting a dark picture.
     *
     * Writing this struct and calling MI_SNR_SetFps after the tuning load
     * is the *only* thing raptor does here that divinus does not -- divinus
     * defines i6_sensor_exposure and never calls it, so it runs on whatever
     * the tuning and CUS3A settle on between themselves. When divinus looks
     * better in the dark on the same board and bin, that difference is the
     * first thing to remove, and removing it by rebuild-and-reflash costs a
     * night. This makes it one env var.
     *
     * Not a config key: it exists to answer a question, not to be a
     * supported way to run. If it turns out to be the answer, the fix is a
     * considered change to what this function does by default, not this.
     */
    if (getenv("RSS_ISP_NO_EXPO_CAP")) {
        static bool said;

        if (!said) {
            said = true;
            HAL_LOG_INFO("isp: RSS_ISP_NO_EXPO_CAP -- leaving the AE's exposure limits and "
                         "the sensor framerate entirely alone, as divinus does");
        }
        return RSS_OK;
    }

    ret = star_isp_read_limits(st, &limit);
    if (ret == RSS_ERR_IO)
        return ret;
    if (ret != RSS_OK || limit.maxShutterUs == 0) {
        HAL_LOG_WARN("isp: AE published no exposure limits; shutter left uncapped");
        return RSS_ERR_TIMEOUT;
    }

    /*
     * An explicit max_exposure_us owns the ceiling. Without this the
     * framerate clamp below would immediately undo it, and a config key
     * that silently does nothing is worse than no key.
     */
    if (st->pend_ae_it_max > 0) {
        HAL_LOG_DBG("isp: max shutter left at the configured %d us", st->pend_ae_it_max);
        return RSS_OK;
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

/*
 * ================================================================
 * WHY THE TUNING LOAD IS NOT DONE HERE
 *
 * The first board run of this file had every single MI_ISP call fail
 * with 6 and the parameter store never come ready. The cause is
 * ordering, and it is structural rather than a matter of waiting longer.
 *
 * MI has no independent ISP device. Disassembling any
 * MI_ISP_{IQ,AE}_Get<Module> shows it delegate to
 * _MI_ISP_GetIspApiData, which dispatches through a per-channel handler
 * table and, where no handler is registered, falls back to
 * MI_VPE_GetIspApiData -- the ISP is answered *by the VPE channel*. And
 * a VPE channel with no enabled output port has nowhere to send frames,
 * so it does not run, so its ISP front end never initialises and every
 * query is refused.
 *
 * At hal_init time that is exactly the state: star_vpe_bringup has
 * created, started and bound the channel, but the output ports belong to
 * the framesource ops and no caller has enabled one yet. Waiting longer
 * cannot help -- nothing was going to happen.
 *
 * Both references get this right by construction rather than by
 * explanation: divinus loads its IQ file at the very end of sdk_init,
 * after the encoding thread is already running (media.c:827), and
 * waybeam loads after its output and video stages are up. So the load
 * moves to star_isp_tune_when_ready, which the framesource and encoder
 * start paths call once frames can actually flow.
 *
 * What stays here is only what is safe before the pipeline runs: binding
 * the library and working out which file to load. Neither touches MI.
 * ================================================================
 */
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
    st->pend_max_again = -1;
    st->pend_max_dgain = -1;
    st->pend_ae_it_max = -1;

    ret = i6_isp_load(&st->isp);
    if (ret != RSS_OK) {
        HAL_LOG_WARN("isp: MI_ISP unavailable (%d); no tuning or 3A control this run", ret);
        return;
    }
    st->isp_loaded = true;

    /* Resolution is pure path arithmetic plus access(), so it is legal
     * now, and cfg is only in scope during init. */
    if (star_isp_resolve_iq(st, cfg, path, sizeof(path)))
        snprintf(st->iq_file, sizeof(st->iq_file), "%s", path);
}

/*
 * Load the tuning binary, once the ISP is actually answering.
 *
 * Called from the framesource enable and encoder start paths rather than
 * from hal_init -- see the comment above star_isp_bringup for why that
 * is not a detail. Idempotent, and deliberately does *not* mark itself
 * done when the ISP is not ready yet, so a later call retries; the
 * quiet/verbose split keeps the first attempt from logging a warning
 * that the second one is about to make untrue.
 */
static void star_isp_flush_pending(star_state_t *st);
static int star_isp_set_orien(star_state_t *st);

void star_isp_tune_when_ready(star_state_t *st, bool verbose)
{
    int ret;

    if (!st || !st->isp_loaded || st->isp_tuned)
        return;

    if (star_isp_wait_ready(st, verbose ? STAR_ISP_READY_TIMEOUT_MS : STAR_ISP_READY_QUICK_MS,
                            verbose) != RSS_OK)
        return;

    /* Ready is a one-way transition, so one attempt from here on. */
    st->isp_tuned = true;

    if (st->iq_file[0]) {
        /*
         * THE 3A HANDOFF IS OFF BY DEFAULT, and that is the whole point.
         *
         * The code used to stop userspace 3A before the load and restart
         * CUS3A after it, copying waybeam (star6e_pipeline.c:270-284).
         * divinus does neither -- it sleeps a second and loads
         * (media.c:827) -- and divinus is the reference with known-good
         * colour on this board.
         *
         * Board evidence 2026-07-26: with the handoff in place and
         * MI_ISP_CUS3A_Enable demonstrably passing AWB = 1 (the driver
         * prints the flags it received), the picture still had a magenta
         * cast under artificial light. That is what an auto white balance
         * which is *enabled* but has no algorithm behind it looks like.
         * MI_ISP_DisableUserspace3A tears the vendor algorithms down --
         * libmi_isp imports CUS3A_Init and CUS3A_EnableUserspaceAE/AWB/AF
         * from libcus3a, and that registration is what
         * MI_ISP_DisableUserspace3A undoes. MI_ISP_CUS3A_Enable only sets
         * flags; it cannot put the algorithms back. The one entry point
         * that can is MI_ISP_EnableUserspace3A, which is a *different*
         * symbol, and waybeam's own 6E notes say not to call it on the
         * normal internal-AE path.
         *
         * So: load the binary and leave 3A alone, which is both the
         * smaller change and the one with a working precedent.
         *
         * RSS_ISP_3A_HANDOFF=1 restores the old sequence, so the two can
         * be compared on hardware from one binary.
         */
        const char *want_handoff = getenv("RSS_ISP_3A_HANDOFF");
        bool handoff = want_handoff && want_handoff[0] == '1';

        if (handoff && st->isp.fnDisableUserspace3A &&
            st->isp.fnDisableUserspace3A(STAR_ISP_CHN))
            HAL_LOG_WARN("isp: MI_ISP_DisableUserspace3A failed; loading anyway");

        ret = st->isp.fnLoadChannelConfig(STAR_ISP_CHN, st->iq_file, STAR_IQ_LOAD_KEY);
        if (ret) {
            HAL_LOG_WARN("isp: loading %s failed: %d; the generic vendor tuning stays in "
                         "effect and colour will be off",
                         st->iq_file, ret);
            st->iq_file[0] = '\0';
        } else {
            HAL_LOG_INFO("isp: loaded tuning file %s (3A %s)", st->iq_file,
                         handoff ? "handed off and restarted (RSS_ISP_3A_HANDOFF)"
                                 : "left running, as divinus does");
        }

        if (handoff)
            star_isp_enable_3a(st);
    }

    /*
     * Snapshot the tuning's exposure limits before the knobs land on them,
     * so a requested gain ceiling can be judged against the calibration
     * rather than against whatever the previous write left behind.
     */
    star_isp_snapshot_bin_limits(st);

    /* Config knobs go on after the tuning file, never before. */
    star_isp_flush_pending(st);

    /* Worth doing whether or not a tuning file loaded: the limits come
     * from whichever tuning is in effect, and neither is obliged to suit
     * the framerate this pipeline asked for. */
    star_isp_cap_exposure(st, st->fps);

    /*
     * Orientation goes on last, and only when something was asked for.
     *
     * MI_SNR_SetOrien does not write the sensor's mirror register. The
     * driver stores the value, marks it dirty, and writes it on the next
     * frame notification from AE -- pCus_SetOrien and
     * pCus_AEStatusNotify(CUS_FRAME_ACTIVE) in the vendor
     * sensor_<name>_mipi.c. Anything between bring-up and here that
     * reprograms sensor timing or restarts 3A can therefore lose that
     * pending write, and both the tuning load and the MI_SNR_SetFps
     * inside star_isp_cap_exposure are candidates. waybeam hit exactly
     * this after a live bin reload and fixed it by re-issuing SetOrien
     * once afterwards; this is the same repair at the same point in the
     * sequence.
     *
     * No re-apply is needed when nothing was asked for, because anything
     * that resets orientation resets it to unmirrored.
     */
    if (st->hflip || st->vflip)
        star_isp_set_orien(st);
}

/*
 * Forget that the tuning was applied, so the next star_isp_tune_when_ready
 * does it again.
 *
 * Called when the last VPE output port goes down. The VPE channel only
 * runs while a port is enabled, and when it comes back CUS3A auto-starts
 * from scratch and loads the generic /etc/firmware/iqfile0.bin -- so the
 * sensor's own binary, the AE shutter cap and the control knobs are all
 * gone, and only a latch that survived the restart made it look otherwise.
 * That is what turned any hot restart (rvd's stream-restart,
 * set-resolution, set-codec, osd-restart) into generic colour for the rest
 * of the process's life, with nothing in the log to say so.
 */
void star_isp_untune(star_state_t *st)
{
    if (!st || !st->isp_tuned)
        return;

    st->isp_tuned = false;
    HAL_LOG_INFO("isp: VPE channel stopped; tuning will be re-applied when it restarts");
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
        g_iq[i].has_pending = false;
    }

    i6_isp_unload(&st->isp);
    st->isp_loaded = false;
    st->isp_tuned = false;
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
 * The units are MI's own and are not raptor's 0..255: they are x1024
 * fixed point, 1024 being unity. rvd's defaults (max_again 160, max_dgain
 * 80) are Ingenic gain codes, and rvd applies them whether or not the
 * config mentions the keys, so the pass-through this code used to do wrote
 * sub-unity ceilings on every boot. Values below unity are now refused and
 * values above the tuning's calibrated ceiling clamped to it -- see the
 * reasoning inside. Treat these two keys as MI-native on this platform.
 */
static int star_isp_apply_gain_limit(star_state_t *st, bool sensor_gain, int gain)
{
    i6_isp_exp limit;
    int ret;

    /* Guarded like every other vendor pointer in this file. i6_isp_load
     * refuses to report success without these two, so a live pipeline
     * always has them -- but this is reachable from the flush, and calling
     * through a null pointer is a worse answer than NOTSUP. */
    if (!st->isp.fnGetExposureLimit || !st->isp.fnSetExposureLimit)
        return RSS_ERR_NOTSUP;

    memset(&limit, 0, sizeof(limit));
    ret = st->isp.fnGetExposureLimit(STAR_ISP_CHN, &limit);
    if (ret) {
        HAL_LOG_WARN("isp: MI_ISP_AE_GetExposureLimit failed: %d", ret);
        return RSS_ERR_IO;
    }

    {
        unsigned int want = (unsigned int)gain;
        unsigned int bin_min = sensor_gain ? st->bin_min_sensor_gain : st->bin_min_isp_gain;
        unsigned int bin_max = sensor_gain ? st->bin_max_sensor_gain : st->bin_max_isp_gain;
        const char *which = sensor_gain ? "sensor" : "isp";

        /*
         * MI's ceilings are x1024 fixed point: 1024 is unity, and the
         * vendor's own constant for a 32x cap is 32768 (waybeam's
         * AE_GAIN_MAX_DEFAULT, "32x sensor cap"). raptor's max_again and
         * max_dgain keys are Ingenic gain codes, and rvd applies its
         * Ingenic defaults -- 160 and 80 -- on every platform whether or
         * not the config mentions them. Written through unscaled those are
         * ceilings of 0.16x and 0.08x: below unity, so not gain ceilings
         * at all. maxIspGain = 80 is the damaging one, because it pins the
         * ISP's digital gain at its floor and so removes all the headroom
         * above the sensor's own ceiling -- which is why total_gain on
         * this board stopped dead at 8192 (8x, the tuning's own
         * maxSensorGain) instead of climbing through it as the light went.
         *
         * Refused rather than scaled: no conversion turns an Ingenic gain
         * code into an MI one, so the only honest answer is to leave the
         * tuning's calibrated ceiling alone and say why, once per load.
         */
        if (want < 1024u) {
            HAL_LOG_WARN("isp: ignoring max %s gain %u -- MI wants x1024 units, so that "
                         "reads as %u.%02ux, a ceiling below unity. The tuning's own limit "
                         "stands. Set max_again/max_dgain in x1024 (1024 = 1.0x).",
                         which, want, want / 1024u, (want % 1024u) * 100u / 1024u);
            return RSS_ERR_INVAL;
        }

        /*
         * MI validates against the calibrated range and quietly keeps its
         * own value when a ceiling is out of it, so clamping here is only
         * about the log: an unexplained ceiling that did not take is much
         * harder to spot than one that says it was clamped. waybeam found
         * the same wall -- gainMax 32000 against a bin ceiling of 8192.
         */
        if (bin_max && want > bin_max) {
            HAL_LOG_INFO("isp: max %s gain %u is above the tuning's calibrated ceiling %u; "
                         "clamping, because MI does not honour a higher one",
                         which, want, bin_max);
            want = bin_max;
        }
        if (bin_min && want < bin_min) {
            HAL_LOG_INFO("isp: max %s gain %u is below the tuning's floor %u; raising to it",
                         which, want, bin_min);
            want = bin_min;
        }

        if (sensor_gain)
            limit.maxSensorGain = want;
        else
            limit.maxIspGain = want;
    }

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

    if (!st->isp_tuned) {
        int pend = sensor_gain ? st->pend_max_again : st->pend_max_dgain;

        *gain = pend >= 0 ? (uint32_t)pend : 0u;
        return RSS_OK;
    }

    memset(&limit, 0, sizeof(limit));
    ret = st->isp.fnGetExposureLimit(STAR_ISP_CHN, &limit);
    if (ret) {
        HAL_LOG_WARN("isp: MI_ISP_AE_GetExposureLimit failed: %d", ret);
        return RSS_ERR_IO;
    }

    *gain = sensor_gain ? limit.maxSensorGain : limit.maxIspGain;
    return RSS_OK;
}

static int star_isp_set_gain_limit(void *ctx, bool sensor_gain, int gain)
{
    star_state_t *st = star_state(ctx);

    if (!st || !st->isp_loaded)
        return RSS_ERR_NOENT;
    if (gain < 0)
        return RSS_ERR_INVAL;

    /* Recorded like the IQ knobs, and for the same two reasons: the ISP
     * may not be up yet, and a later tuning load will need it back. */
    if (sensor_gain)
        st->pend_max_again = gain;
    else
        st->pend_max_dgain = gain;

    if (!st->isp_tuned) {
        HAL_LOG_DBG("isp: max %s gain = %d queued until the ISP is up",
                    sensor_gain ? "sensor" : "isp", gain);
        return RSS_OK;
    }

    return star_isp_apply_gain_limit(st, sensor_gain, gain);
}

/*
 * Re-apply everything that has been asked for.
 *
 * Called from star_isp_tune_when_ready *after* the tuning binary has
 * loaded, which is the only correct order: applied first, the load would
 * overwrite them.
 *
 * Nothing is consumed here. Every load resets the modules to the
 * binary's own state, so these values have to be re-applied after each
 * one, not drained once -- see the comment on has_pending.
 */
static int star_isp_apply_ae_it_max(star_state_t *st, unsigned int it_max);

static void star_isp_flush_pending(star_state_t *st)
{
    size_t i;

    for (i = 0; i < IQ_PARAM_COUNT; i++) {
        star_iq_param_t *p = &g_iq[i];

        if (!p->has_pending)
            continue;

        if (p->pending_is_raw)
            (void)star_iq_apply_raw(st, (int)i, (uint32_t)p->pending);
        else
            (void)star_iq_apply_scalar(st, (int)i, p->pending);
    }

    if (st->pend_max_again >= 0)
        (void)star_isp_apply_gain_limit(st, true, st->pend_max_again);
    if (st->pend_max_dgain >= 0)
        (void)star_isp_apply_gain_limit(st, false, st->pend_max_dgain);

    /* Before star_isp_cap_exposure, which only lowers, so a request at or
     * under the frame period survives it untouched. */
    if (st->pend_ae_it_max > 0)
        (void)star_isp_apply_ae_it_max(st, (unsigned int)st->pend_ae_it_max);
}

/*
 * AE max integration time, in microseconds.
 *
 * The unit is this platform's, like the gain ceilings above: MI states
 * the shutter limit in microseconds, so that is what this op takes here.
 * Ingenic's own isp_set_ae_it_max is a different quantity on a different
 * scale; nothing converts between them.
 *
 * Why this op exists at all: star_isp_cap_exposure only ever *lowers* the
 * ceiling, to hold the requested framerate, and both references do the
 * same -- waybeam clamps with `want_shutter <= cur_limit.maxShutterUs`
 * and divinus never calls SetExposureLimit at all (i6_sensor_exposure is
 * defined with no caller). So a tuning file that publishes a conservative
 * shutter ceiling is the ceiling, and on a sensor whose gain ceiling is
 * also low -- gc4653.bin allows 8x sensor and no ISP gain at all -- there
 * is no lever left for a dark scene. This is that lever, opt-in, because
 * trading motion blur for light is a decision this code cannot make.
 *
 * Still bounded by the frame period: a longer exposure than the frame
 * period cannot be honoured without dropping framerate, which is a
 * surprising way to get a brighter picture.
 */
static int star_isp_apply_ae_it_max(star_state_t *st, unsigned int it_max)
{
    i6_isp_exp limit;
    unsigned int frame_us;
    int ret;

    if (!st->isp.fnGetExposureLimit || !st->isp.fnSetExposureLimit)
        return RSS_ERR_NOTSUP;

    ret = st->isp.fnGetExposureLimit(STAR_ISP_CHN, &limit);
    if (ret) {
        HAL_LOG_WARN("isp: MI_ISP_AE_GetExposureLimit failed: %d", ret);
        return RSS_ERR_IO;
    }

    /*
     * An explicit request is honoured even past the frame period, because
     * that is the whole point of having it: a tuning that publishes a
     * ceiling *longer* than the frame period is telling the AE it may
     * trade framerate for light in the dark, and clamping that away is
     * exactly what this key exists to undo. gc4653.bin asks for 50000 us
     * against a 33333 us period at 30 fps, and divinus -- which never
     * writes the limit at all -- lets it have that.
     *
     * Warned rather than clamped, because a silently variable framerate is
     * a surprise worth one log line.
     */
    if (st->fps) {
        frame_us = 1000000u / st->fps;
        if (it_max > frame_us)
            HAL_LOG_WARN("isp: max exposure %u us is longer than the %u us frame period at "
                         "%u fps -- the AE may drop below %u fps in low light, which is the "
                         "trade this asks for",
                         it_max, frame_us, st->fps, st->fps);
    }

    if (limit.maxShutterUs == it_max) {
        HAL_LOG_DBG("isp: max exposure already %u us", it_max);
        return RSS_OK;
    }

    HAL_LOG_INFO("isp: max exposure %u -> %u us", limit.maxShutterUs, it_max);
    limit.maxShutterUs = it_max;
    if (limit.minShutterUs > it_max)
        limit.minShutterUs = it_max;

    ret = st->isp.fnSetExposureLimit(STAR_ISP_CHN, &limit);
    if (ret) {
        HAL_LOG_WARN("isp: MI_ISP_AE_SetExposureLimit failed: %d", ret);
        return RSS_ERR_IO;
    }

    /*
     * Read back and say what stuck. MI validates a gain ceiling against
     * the tuning's and quietly keeps its own, and there is no reason to
     * assume the shutter ceiling is treated differently -- so a silent
     * success here would be indistinguishable from a write that MI threw
     * away, which is exactly the failure that made the gain ceiling take
     * a board run to understand.
     */
    memset(&limit, 0, sizeof(limit));
    if (st->isp.fnGetExposureLimit(STAR_ISP_CHN, &limit) == 0 && limit.maxShutterUs != it_max)
        HAL_LOG_WARN("isp: max exposure did not take -- asked %u us, AE reports %u; the tuning's "
                     "ceiling is likely authoritative",
                     it_max, limit.maxShutterUs);

    return RSS_OK;
}

int hal_isp_set_ae_it_max(void *ctx, uint32_t it_max)
{
    star_state_t *st = star_state(ctx);

    if (!st || !st->isp_loaded)
        return RSS_ERR_NOENT;

    /* 0 means "leave the tuning's ceiling alone", which is also the
     * default, so it must clear a previous request rather than ask for a
     * zero-microsecond exposure. */
    st->pend_ae_it_max = it_max ? (int)it_max : -1;

    if (!st->isp_tuned || !it_max) {
        HAL_LOG_DBG("isp: max exposure %u us queued until the ISP is up", it_max);
        return RSS_OK;
    }

    return star_isp_apply_ae_it_max(st, it_max);
}

int hal_isp_get_ae_it_max(void *ctx, uint32_t *it_max)
{
    star_state_t *st = star_state(ctx);
    i6_isp_exp limit;

    if (!st || !st->isp_loaded)
        return RSS_ERR_NOENT;
    if (!it_max)
        return RSS_ERR_INVAL;

    if (!st->isp_tuned) {
        *it_max = st->pend_ae_it_max >= 0 ? (uint32_t)st->pend_ae_it_max : 0u;
        return RSS_OK;
    }
    if (!st->isp.fnGetExposureLimit)
        return RSS_ERR_NOTSUP;

    memset(&limit, 0, sizeof(limit));
    if (st->isp.fnGetExposureLimit(STAR_ISP_CHN, &limit)) {
        HAL_LOG_WARN("isp: MI_ISP_AE_GetExposureLimit failed");
        return RSS_ERR_IO;
    }

    *it_max = limit.maxShutterUs;
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
 * Exposure readback -- what the AE converged on, for ric's day/night
 * detection.
 *
 * This op was in the "left unimplemented on purpose" list above until
 * phase 6, on the grounds that MI exposes no current-exposure query. That
 * was wrong about one symbol: AE_GetManualExpo returns the manual
 * *setting* and AE_GetExposureLimit the bounds, but CUS3A_GetAeStatus
 * returns what the AE actually converged on.
 *
 * Two calls, and the second one is optional. MI_ISP_CUS3A_GetAeStatus
 * gives shutter and both gains, which is a complete ambient-light signal
 * on its own: a scene going dark drives the shutter and then the gain up,
 * and ric's night->day rule compares gain against its own night baseline
 * so it needs no absolute scale. MI_ISP_AE_GetAeHwAvgStats adds scene
 * luma, which ric's day->night rule prefers because IR illumination does
 * not inflate it the way it inflates gain.
 *
 * Deliberately not filled: ev (an Ingenic GetEVAttr concept with no MI
 * equivalent) and wb_rgain/wb_bgain (MI_ISP_AWB_GetAwbHwAvgStats exists,
 * but its 128x90 grid is not the two global gains the field wants).
 * Leaving them zero is what tells ric the photo trigger has nothing to
 * work with -- see the zero convention below.
 *
 * ================================================================
 * ZERO MEANS "NOT AVAILABLE", AND THAT IS A CONTRACT, NOT A HABIT
 *
 * The Ingenic backend already works this way: when GetAeStatistics
 * fails it warns once and leaves ae_luma at 0 so that "day/night falls
 * back to gain-only behavior" (src/hal_isp.c). A backend that cannot
 * answer for a field leaves it zero, and the consumer must read zero as
 * silence rather than as a reading.
 *
 * It matters most for luma, where the two readings sit at opposite ends:
 * a live sensor never reports a mean luma of exactly 0, and ric's
 * day->night test is `ae_luma < night_luma`. So a zero read as data is
 * the darkest possible scene, and a backend with no luma source would
 * pin the camera in night mode forever. ric was doing exactly that on
 * this platform before phase 6.
 * ================================================================
 */

/* Both gains are x1024 fixed point (1024 = 1.0x), so multiplying two of
 * them needs the divide to get back to x1024 -- and 64-bit intermediates,
 * since 32x/32x overflows u32 at gain 64x. */
static uint32_t star_ae_total_gain(const i6_isp_ae_status *ae)
{
    uint64_t sensor = ae->sensorGain;
    uint64_t isp = ae->ispGain ? ae->ispGain : 1024u;
    uint64_t total = sensor * isp / 1024u;

    return total > UINT32_MAX ? UINT32_MAX : (uint32_t)total;
}

/*
 * Mean of the AE grid's Y lane, or 0 if the layout cannot be confirmed.
 *
 * The confirmation is the point. i6_isp.h derives a 128x90 grid of
 * 4-byte cells from two wrappers' payload sizes but cannot place the
 * eight spare bytes, so this reads the grid dimensions from the AE status
 * (which has its own field offsets) and looks for them at both ends of
 * the stats block. A match places the cells; no match means the layout
 * guess is wrong, and then the honest answer is no luma -- averaging the
 * wrong offset produces a number that looks like a reading and moves the
 * IR-cut filter.
 */
static uint32_t star_ae_luma(star_state_t *st, const i6_isp_ae_status *ae)
{
    static bool layout_logged;
    static bool layout_warned;
    i6_isp_ae_hw_stats *stats;
    const unsigned char *cell;
    unsigned int blk_x = ae->avgBlkX;
    unsigned int blk_y = ae->avgBlkY;
    unsigned int cells;
    uint64_t sum[I6_ISP_AE_CELL_SZ] = {0};
    uint32_t luma;
    int ret;

    if (!st->isp.fnGetAeHwAvgStats)
        return 0;

    if (blk_x == 0 || blk_x > I6_ISP_AE_BLK_X || blk_y == 0 || blk_y > I6_ISP_AE_BLK_Y) {
        if (!layout_warned) {
            HAL_LOG_WARN("isp: AE grid dimensions %ux%u are outside the %ux%u block -- "
                         "no scene luma, day/night falls back to gain only",
                         blk_x, blk_y, I6_ISP_AE_BLK_X, I6_ISP_AE_BLK_Y);
            layout_warned = true;
        }
        return 0;
    }

    /* 46KB, so off the stack. Same size every call, so the allocator
     * hands back the same chunk; ric polls this once a second. */
    stats = malloc(sizeof(*stats));
    if (!stats)
        return 0;

    memset(stats, 0, sizeof(*stats));
    ret = st->isp.fnGetAeHwAvgStats(STAR_ISP_CHN, stats);
    if (ret) {
        if (!layout_warned) {
            HAL_LOG_WARN("isp: MI_ISP_AE_GetAeHwAvgStats failed: %d -- no scene luma", ret);
            layout_warned = true;
        }
        free(stats);
        return 0;
    }

    if (stats->lead.blkX == blk_x && stats->lead.blkY == blk_y) {
        cell = stats->lead.cell;
    } else if (stats->trail.blkX == blk_x && stats->trail.blkY == blk_y) {
        cell = stats->trail.cell;
    } else {
        /*
         * Both ends disagree with the AE status. One log line with the
         * eight candidate bytes is enough to place them from a board
         * log, which is the only place the answer exists.
         */
        if (!layout_warned) {
            HAL_LOG_WARN("isp: AE stats layout unconfirmed for a %ux%u grid "
                         "(lead %u,%u trail %u,%u) -- no scene luma, day/night "
                         "falls back to gain only",
                         blk_x, blk_y, stats->lead.blkX, stats->lead.blkY, stats->trail.blkX,
                         stats->trail.blkY);
            layout_warned = true;
        }
        free(stats);
        return 0;
    }

    cells = blk_x * blk_y;
    for (unsigned int i = 0; i < cells; i++)
        for (unsigned int lane = 0; lane < I6_ISP_AE_CELL_SZ; lane++)
            sum[lane] += cell[i * I6_ISP_AE_CELL_SZ + lane];

    luma = (uint32_t)(sum[I6_ISP_AE_CELL_Y] / cells);

    /*
     * All four lane means, once. The cell layout is waybeam's r, g, b, y
     * and nothing here proves which lane is Y; on a coloured scene the
     * four means differ, and this line is what says whether lane 3 is
     * behaving like luma. Cheap to compute in the same pass and worth
     * far more than a debug build on a camera nobody is watching.
     */
    if (!layout_logged) {
        HAL_LOG_INFO("isp: AE grid %ux%u, cells at offset %u, lane means "
                     "r=%llu g=%llu b=%llu y=%llu (y is the one used)",
                     blk_x, blk_y, cell == stats->lead.cell ? 8u : 0u,
                     (unsigned long long)(sum[0] / cells), (unsigned long long)(sum[1] / cells),
                     (unsigned long long)(sum[2] / cells), (unsigned long long)(sum[3] / cells));
        layout_logged = true;
    }

    free(stats);
    return luma;
}

/*
 * Periodic AE diagnostic, off unless RSS_AE_DIAG is set.
 *
 * The question this exists to answer cannot be answered from the readings
 * ric already has. A dark picture with gain at its ceiling and shutter well
 * short of it looks like a bug, but the AE choosing that exposure and the
 * AE being prevented from going further are indistinguishable from outside
 * -- so this reports `boundary`, which says which it is, alongside the
 * target it was aiming at and the light value it measured.
 *
 * The long/short pair is printed because the difference matters: if the AE
 * is running an HDR pair, `exposure_us` in ric's status is whichever field
 * the AE status struct calls the primary one, and comparing it against
 * these two says whether that is the exposure setting image brightness.
 *
 * Env-gated and rate-limited rather than a build option, because it is
 * wanted on a board that is already flashed and already dark -- the same
 * reason RSS_OSD_PIXFMT and RSS_ISP_3A_HANDOFF are env hatches. At INFO
 * because HAL_LOG_DBG is compiled out of release builds.
 */
#define STAR_AE_DIAG_INTERVAL_S 5

static void star_ae_diag(star_state_t *st)
{
    static int enabled = -1;
    static time_t last;
    i6_isp_ae_expo_info info;
    i6_isp_exp limit;
    struct timespec now;

    if (enabled < 0)
        enabled = getenv("RSS_AE_DIAG") != NULL;
    if (!enabled || !st->isp.fnQueryExposureInfo)
        return;

    if (clock_gettime(CLOCK_MONOTONIC, &now))
        return;
    if (last && now.tv_sec - last < STAR_AE_DIAG_INTERVAL_S)
        return;
    last = now.tv_sec;

    memset(&info, 0, sizeof(info));
    if (st->isp.fnQueryExposureInfo(STAR_ISP_CHN, &info)) {
        HAL_LOG_WARN("isp: MI_ISP_AE_QueryExposureInfo failed");
        return;
    }

    memset(&limit, 0, sizeof(limit));
    if (st->isp.fnGetExposureLimit)
        (void)st->isp.fnGetExposureLimit(STAR_ISP_CHN, &limit);

    HAL_LOG_INFO("ae: stable=%d boundary=%d target=%u avgY=%u lumY=%u lv=%u.%u bv=%d",
                 info.stable, info.reachBoundary, info.sceneTarget, info.histAvgY, info.histLumY,
                 info.lvX10 / 10u, info.lvX10 % 10u, info.bv);
    HAL_LOG_INFO("ae: long %uus gain %u/%u | short %uus gain %u/%u", info.expoLong.us,
                 info.expoLong.sensorGain, info.expoLong.ispGain, info.expoShort.us,
                 info.expoShort.sensorGain, info.expoShort.ispGain);
    HAL_LOG_INFO("ae: limits shutter %u..%u us, sensor gain %u..%u, isp gain %u..%u",
                 limit.minShutterUs, limit.maxShutterUs, limit.minSensorGain,
                 limit.maxSensorGain, limit.minIspGain, limit.maxIspGain);
}

/*
 * Re-assert an explicitly configured exposure ceiling, because a single
 * write does not hold.
 *
 * Board evidence, 2026-07-27. The tuning published `sensor gain
 * 1024..131072, shutter 22..50000`; we clamped the shutter to 33333 for
 * 30 fps and wrote nothing else. Ninety seconds later the AE was operating
 * on `shutter 300..14000, sensor gain 1024..8192` -- a window 3.5x shorter
 * and 16x less sensitive than the tuning allows, with the AE pinned on both
 * ceilings (`boundary=1`) against a measured `avgY=0`. So CUS3A rewrites
 * this struct while it runs, and our value lasted only until it did.
 *
 * waybeam already worked around this and the shape of its code says so:
 * its cus3a_thread re-reads the limit struct and re-pushes any field that
 * drifted, every AE tick, for the whole run (maruko_cus3a.c). A one-shot
 * write at tuning-load time is simply not how this interface works.
 *
 * Deliberately only re-asserts what was *asked for*. With nothing
 * configured this does nothing at all, because then CUS3A narrowing its own
 * window is its business -- fighting an algorithm over values nobody
 * requested is how you get an AE that oscillates. But a ceiling the config
 * states is a ceiling the config should get, and the alternative is the
 * failure this whole area keeps producing: a write that silently did not
 * take.
 *
 * Hooked onto get_exposure rather than a thread of its own: ric already
 * polls it once a second, which is an order of magnitude slower than
 * waybeam's loop and plenty to hold a ceiling.
 */
#define STAR_LIMIT_REASSERT_S 2

static void star_isp_reassert_limits(star_state_t *st)
{
    i6_isp_exp limit;
    static time_t last;
    static bool reported;
    struct timespec now;
    unsigned int want_shutter, want_gain;
    bool fix_shutter, fix_gain;

    if (st->pend_ae_it_max <= 0 && st->pend_max_again < 1024)
        return;
    if (!st->isp.fnGetExposureLimit || !st->isp.fnSetExposureLimit)
        return;

    if (clock_gettime(CLOCK_MONOTONIC, &now))
        return;
    if (last && now.tv_sec - last < STAR_LIMIT_REASSERT_S)
        return;
    last = now.tv_sec;

    memset(&limit, 0, sizeof(limit));
    if (st->isp.fnGetExposureLimit(STAR_ISP_CHN, &limit))
        return;

    want_shutter = st->pend_ae_it_max > 0 ? (unsigned int)st->pend_ae_it_max : 0u;
    want_gain = st->pend_max_again >= 1024 ? (unsigned int)st->pend_max_again : 0u;
    if (want_gain && st->bin_max_sensor_gain && want_gain > st->bin_max_sensor_gain)
        want_gain = st->bin_max_sensor_gain;

    fix_shutter = want_shutter && limit.maxShutterUs != want_shutter;
    fix_gain = want_gain && limit.maxSensorGain != want_gain;
    if (!fix_shutter && !fix_gain)
        return;

    if (!reported) {
        reported = true;
        HAL_LOG_INFO("isp: AE narrowed its limits to shutter ..%u us / sensor gain ..%u; "
                     "restoring the configured ..%u us / ..%u and holding them",
                     limit.maxShutterUs, limit.maxSensorGain,
                     want_shutter ? want_shutter : limit.maxShutterUs,
                     want_gain ? want_gain : limit.maxSensorGain);
    }

    if (fix_shutter) {
        limit.maxShutterUs = want_shutter;
        if (limit.minShutterUs > want_shutter)
            limit.minShutterUs = want_shutter;
    }
    if (fix_gain) {
        limit.maxSensorGain = want_gain;
        if (limit.minSensorGain > want_gain)
            limit.minSensorGain = want_gain;
    }

    (void)st->isp.fnSetExposureLimit(STAR_ISP_CHN, &limit);
}

int hal_isp_get_exposure(void *ctx, rss_exposure_t *exposure)
{
    star_state_t *st = star_state(ctx);
    i6_isp_ae_status ae;
    int ret;

    if (!st || !st->isp_loaded)
        return RSS_ERR_NOENT;
    if (!exposure)
        return RSS_ERR_INVAL;
    if (!st->isp.fnGetAeStatus)
        return RSS_ERR_NOTSUP;

    memset(exposure, 0, sizeof(*exposure));

    /*
     * Before the tuning binary lands the ISP channel is not up and the
     * call errors. That is a startup window, not a fault, and ric polls
     * through it once a second -- so say "busy" and log nothing.
     */
    if (!st->isp_tuned)
        return RSS_ERR_BUSY;

    memset(&ae, 0, sizeof(ae));
    ret = st->isp.fnGetAeStatus(STAR_ISP_CHN, &ae);
    if (ret) {
        static bool warned;

        if (!warned) {
            HAL_LOG_WARN("isp: MI_ISP_CUS3A_GetAeStatus failed: %d -- "
                         "no exposure readback, ric will hold its current mode",
                         ret);
            warned = true;
        }
        return RSS_ERR_IO;
    }

    exposure->exposure_time = ae.shutterUs;
    exposure->total_gain = star_ae_total_gain(&ae);
    exposure->ae_luma = star_ae_luma(st, &ae);

    star_isp_reassert_limits(st);
    star_ae_diag(st);

    return RSS_OK;
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
 * Done in the sensor, not the ISP. MI_SNR_SetOrien takes both axes at
 * once, so each op has to supply the one it is not changing from tracked
 * state; the starting values come from the sensor config, applied before
 * MI_SNR_Enable in star_sensor_bringup.
 *
 * That bring-up path is the one both references use and the one to trust.
 * A runtime change is best-effort by comparison, because of how the
 * vendor sensor driver implements it: SetOrien only stores the value and
 * sets a dirty flag, and the register is written by
 * pCus_AEStatusNotify(CUS_FRAME_ACTIVE) -- so it lands on the next AE
 * frame notification if 3A is running, and sits pending if it is not.
 * MI_SNR_GetOrien is no help in telling which happened: the vendor
 * driver reads it back from its static default table rather than the live
 * value, so it reports unmirrored however the image actually looks (which
 * matches waybeam's note that GetOrien "reads 0 while the image is
 * plainly held"). Hence the tracked state here, and hence the re-apply at
 * the end of star_isp_tune_when_ready.
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
