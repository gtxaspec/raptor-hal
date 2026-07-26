/*
 * Host-side test of the pure logic in star/hal_isp.c.
 *
 * Includes the real translation unit rather than a copy, so the scaling
 * and field-access code under test is exactly what ships. MI itself is
 * never called: every test here drives the static helpers directly.
 */

#define PLATFORM_INFINITY6E 1
#define HAL_MODULE_VIDEO 1

#include "star/hal_isp.c"

#include <assert.h>
#include <stdio.h>

/*
 * The HAL's logging indirection, which libraptor_hal normally provides.
 * Silent here: these tests drive failure paths on purpose and the noise
 * would bury the results.
 *
 * Note the host build also suppresses the i6_*.h _Static_asserts via
 * -D'_Static_assert(c,m)='. They assert 32-bit pointer layouts on
 * structs none of these tests touch, and the real ARM build still
 * checks every one of them.
 */
/*
 * A real (silent) logger, not a NULL pointer: HAL_LOG_* call through this
 * without a NULL guard, because rss_hal_init always installs one. Any test
 * that exercises a warning path segfaults if this is left NULL.
 */
static void quiet_log(int level, const char *file, int line, const char *fmt, ...)
{
    (void)level;
    (void)file;
    (void)line;
    (void)fmt;
}

rss_hal_log_func_t rss_hal_log_fn = quiet_log;

static int failures;

#define CHECK(cond, fmt, ...)                                                                      \
    do {                                                                                           \
        if (!(cond)) {                                                                             \
            printf("FAIL %s:%d: " fmt "\n", __func__, __LINE__, ##__VA_ARGS__);                    \
            failures++;                                                                            \
        }                                                                                          \
    } while (0)

/*
 * The invariant that matters most: every field the table addresses must
 * lie wholly inside the payload MI copies. An offset past the end would
 * be written into our buffer and silently dropped -- or, if the payload
 * size were wrong the other way, MI would read past what we filled.
 */
static void test_table_bounds(void)
{
    size_t i;

    for (i = 0; i < IQ_PARAM_COUNT; i++) {
        const star_iq_param_t *p = &g_iq[i];

        CHECK(p->name && p->get_sym && p->set_sym, "entry %zu has a NULL string", i);
        CHECK(p->width == 1 || p->width == 2 || p->width == 4, "%s: width %u", p->name, p->width);
        CHECK(p->payload <= STAR_IQ_PAYLOAD_MAX, "%s: payload %u exceeds buffer", p->name,
              p->payload);
        CHECK((size_t)p->manual_off + p->width <= p->payload,
              "%s: field at %u+%u overruns the %u-byte payload", p->name, p->manual_off, p->width,
              p->payload);
        CHECK(p->mi_max > 0, "%s: mi_max is zero", p->name);
        CHECK(p->mi_unity <= p->mi_max, "%s: unity %u above max %u", p->name, p->mi_unity,
              p->mi_max);

        /* An auto/manual entry must leave room for bEnable+enOpType. */
        if (p->shape == IQ_AUTOMAN)
            CHECK(p->manual_off >= 8, "%s: AUTOMAN manual offset %u below the 8-byte header",
                  p->name, p->manual_off);
        else
            CHECK(p->manual_off == 0, "%s: FLAT/BOOL must live at offset 0, not %u", p->name,
                  p->manual_off);
    }
}

/* The documented payload sizes, restated here so a typo in the table is a
 * test failure rather than a silently wrong ioctl. Values from
 * disassembling libmi_isp.so. */
static void test_table_matches_disassembly(void)
{
    CHECK(g_iq[IQ_BRIGHTNESS].payload == 76 && g_iq[IQ_BRIGHTNESS].manual_off == 72, "brightness");
    CHECK(g_iq[IQ_CONTRAST].payload == 76 && g_iq[IQ_CONTRAST].manual_off == 72, "contrast");
    CHECK(g_iq[IQ_SATURATION].payload == 416 && g_iq[IQ_SATURATION].manual_off == 392,
          "saturation");
    CHECK(g_iq[IQ_SHARPNESS].payload == 1268 && g_iq[IQ_SHARPNESS].manual_off == 1192, "sharpness");
    CHECK(g_iq[IQ_SINTER].payload == 112 && g_iq[IQ_SINTER].manual_off == 104, "sinter");
    CHECK(g_iq[IQ_TEMPER].payload == 1776 && g_iq[IQ_TEMPER].manual_off == 1288, "temper");
    CHECK(g_iq[IQ_DEFOG].payload == 28, "defog");
    CHECK(g_iq[IQ_GRAY].payload == 4, "gray");
    CHECK(g_iq[IQ_EVCOMP].payload == 8, "evcomp");
    CHECK(g_iq[IQ_FLICKER].payload == 4, "flicker");

    /* Brightness is the entry that proves the convention: a u32 at 72 in
     * a 76-byte payload is exactly the last four bytes. */
    CHECK(g_iq[IQ_BRIGHTNESS].manual_off + g_iq[IQ_BRIGHTNESS].width ==
              g_iq[IQ_BRIGHTNESS].payload,
          "brightness manual field should be the payload's last 4 bytes");
}

/* Read-modify-write must not disturb a single byte outside the field. */
static void test_field_access_is_surgical(void)
{
    uint8_t buf[64], ref[64];
    size_t i;
    unsigned widths[] = { 1, 2, 4 };
    size_t w;

    for (w = 0; w < 3; w++) {
        unsigned width = widths[w];
        uint16_t off = 20;

        for (i = 0; i < sizeof(buf); i++)
            buf[i] = ref[i] = (uint8_t)(i * 7 + 1);

        star_iq_write(buf, off, (uint8_t)width, 0);
        for (i = 0; i < sizeof(buf); i++) {
            if (i >= off && i < off + width)
                continue;
            CHECK(buf[i] == ref[i], "width %u: byte %zu changed (%u -> %u)", width, i, ref[i],
                  buf[i]);
        }
    }

    /* Round-trip at every width, including values that must truncate. */
    memset(buf, 0, sizeof(buf));
    star_iq_write(buf, 4, 1, 200);
    CHECK(star_iq_read(buf, 4, 1) == 200, "u8 round-trip");
    star_iq_write(buf, 8, 2, 40000);
    CHECK(star_iq_read(buf, 8, 2) == 40000, "u16 round-trip");
    star_iq_write(buf, 12, 4, 3000000000u);
    CHECK(star_iq_read(buf, 12, 4) == 3000000000u, "u32 round-trip");

    /* Unaligned offsets must work -- manual offsets are not aligned in
     * general (NRLuma's is 104, but sharpness's manual block starts at
     * 1192 and its neighbours are byte fields). */
    memset(buf, 0, sizeof(buf));
    star_iq_write(buf, 3, 4, 0x01020304u);
    CHECK(star_iq_read(buf, 3, 4) == 0x01020304u, "unaligned u32 round-trip");
    star_iq_write(buf, 7, 2, 0xBEEF);
    CHECK(star_iq_read(buf, 7, 2) == 0xBEEF, "unaligned u16 round-trip");
}

/*
 * The scaling trap this test exists for: raptor's neutral must land on
 * MI's unity, not on the middle of MI's range. Saturation is the case
 * where those differ sharply -- unity is 32 of 127, so a linear map
 * would put neutral at 64 and double the colour on every default config.
 */
static void test_scale_neutral_is_unity(void)
{
    size_t i;

    for (i = 0; i < IQ_PARAM_COUNT; i++) {
        const star_iq_param_t *p = &g_iq[i];

        if (p->mi_unity == 0 || p->mi_unity >= p->mi_max)
            continue; /* bool/enum entries have no scale */

        CHECK(star_iq_scale(STAR_ISP_NEUTRAL, p->mi_unity, p->mi_max) == p->mi_unity,
              "%s: neutral 128 -> %u, expected unity %u", p->name,
              star_iq_scale(STAR_ISP_NEUTRAL, p->mi_unity, p->mi_max), p->mi_unity);
    }

    CHECK(star_iq_scale(128, 32, 127) == 32, "saturation neutral must be unity gain (32), not 64");
    CHECK(star_iq_scale(128, 50, 100) == 50, "brightness neutral");
    CHECK(star_iq_scale(128, 100, 200) == 100, "ev comp neutral means no compensation");
}

static void test_scale_endpoints_and_monotonicity(void)
{
    size_t i;

    for (i = 0; i < IQ_PARAM_COUNT; i++) {
        const star_iq_param_t *p = &g_iq[i];
        uint32_t prev;
        int v;

        if (p->mi_unity == 0 || p->mi_unity >= p->mi_max)
            continue;

        CHECK(star_iq_scale(0, p->mi_unity, p->mi_max) == 0, "%s: 0 -> 0", p->name);
        CHECK(star_iq_scale(255, p->mi_unity, p->mi_max) == p->mi_max, "%s: 255 -> max", p->name);

        /* Never decreasing, and never out of range. */
        prev = 0;
        for (v = 0; v <= 255; v++) {
            uint32_t got = star_iq_scale(v, p->mi_unity, p->mi_max);

            CHECK(got >= prev, "%s: not monotonic at %d (%u after %u)", p->name, v, got, prev);
            CHECK(got <= p->mi_max, "%s: %d -> %u exceeds max %u", p->name, v, got, p->mi_max);
            prev = got;
        }
    }

    /* Out-of-range input is clamped rather than wrapped. */
    CHECK(star_iq_scale(-40, 32, 127) == 0, "negative clamps to 0");
    CHECK(star_iq_scale(9999, 32, 127) == 127, "over-range clamps to max");
}

/*
 * Round-tripping matters because rvd can read a value back and write it
 * again. Exact identity is impossible where MI's range is coarser than
 * raptor's (brightness has 101 steps against 256), so the requirement is
 * that a scale/unscale round trip stays close and that the neutral point
 * is exact.
 */
static void test_unscale_round_trip(void)
{
    size_t i;

    for (i = 0; i < IQ_PARAM_COUNT; i++) {
        const star_iq_param_t *p = &g_iq[i];
        int v;

        if (p->mi_unity == 0 || p->mi_unity >= p->mi_max)
            continue;

        CHECK(star_iq_unscale(p->mi_unity, p->mi_unity, p->mi_max) == STAR_ISP_NEUTRAL,
              "%s: unity must read back as neutral", p->name);
        CHECK(star_iq_unscale(0, p->mi_unity, p->mi_max) == 0, "%s: 0 reads back as 0", p->name);
        CHECK(star_iq_unscale(p->mi_max, p->mi_unity, p->mi_max) == 255, "%s: max reads back as 255",
              p->name);

        for (v = 0; v <= 255; v += 5) {
            uint32_t mi = star_iq_scale(v, p->mi_unity, p->mi_max);
            int back = star_iq_unscale(mi, p->mi_unity, p->mi_max);
            int drift = back > v ? back - v : v - back;
            /* One MI step is worth 255/mi_max raptor steps; allow that
             * plus a rounding unit. */
            int tolerance = (int)(255 / p->mi_max) + 2;

            CHECK(drift <= tolerance, "%s: %d -> MI %u -> %d (drift %d > %d)", p->name, v, mi, back,
                  drift, tolerance);
        }
    }
}

/* Degenerate table entries must not divide by zero or misreport. */
static void test_scale_degenerate_inputs(void)
{
    CHECK(star_iq_scale(200, 0, 1) == 0, "unity 0 short-circuits");
    CHECK(star_iq_scale(200, 5, 5) == 5, "unity == max short-circuits");
    CHECK(star_iq_unscale(3, 0, 10) == STAR_ISP_NEUTRAL, "unity 0 reads back neutral");
    CHECK(star_iq_unscale(0, 0, 0) == 255, "max 0 saturates rather than dividing by zero");
    /* In-range mi with a degenerate unity: falls back to neutral. An mi
     * at or above max saturates first, which is why this uses 5 not 99. */
    CHECK(star_iq_unscale(5, 10, 10) == STAR_ISP_NEUTRAL, "unity >= max reads back neutral");
    CHECK(star_iq_unscale(99, 10, 10) == 255, "mi above max saturates");
}

/*
 * The deferral queue, which is what the first board run needed and did
 * not have. Reaches no MI call: every path exercised here returns before
 * touching the vtable, which is the property being tested.
 */
static void test_pending_queue(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;
    void *c = &ctx;
    uint8_t v8;
    uint32_t v32;
    int vi;

    memset(&ctx, 0, sizeof(ctx));
    memset(&st, 0, sizeof(st));
    ctx.platform = &st;
    st.isp_loaded = true;
    st.isp_tuned = false; /* ISP not answering yet */
    st.pend_max_again = -1;
    st.pend_max_dgain = -1;

    /* A set before the ISP is up must succeed and be remembered, not
     * fail -- rvd applies the whole [image] block at this point. */
    CHECK(hal_isp_set_saturation(c, 200) == RSS_OK, "set_saturation should queue, not fail");
    CHECK(g_iq[IQ_SATURATION].has_pending, "saturation should be queued");
    CHECK(g_iq[IQ_SATURATION].pending == 200, "queued value should be 200");
    CHECK(!g_iq[IQ_SATURATION].pending_is_raw, "saturation queues as a scalar");

    /* And reading it back must agree with what was asked for. */
    v8 = 0;
    CHECK(hal_isp_get_saturation(c, &v8) == RSS_OK, "get_saturation should succeed while queued");
    CHECK(v8 == 200, "queued saturation should read back as 200, got %u", v8);

    /* An untouched knob reads as neutral rather than failing. */
    v8 = 0;
    CHECK(hal_isp_get_brightness(c, &v8) == RSS_OK, "get_brightness should succeed");
    CHECK(v8 == STAR_ISP_NEUTRAL, "untouched brightness should read neutral, got %u", v8);

    /* Raw-valued params take the raw path. */
    CHECK(hal_isp_set_antiflicker(c, RSS_ANTIFLICKER_50HZ) == RSS_OK, "set_antiflicker queues");
    CHECK(g_iq[IQ_FLICKER].has_pending && g_iq[IQ_FLICKER].pending_is_raw,
          "antiflicker queues as raw");
    CHECK(g_iq[IQ_FLICKER].pending == RSS_ANTIFLICKER_50HZ, "queued flicker value");

    CHECK(hal_isp_set_defog(c, 1) == RSS_OK, "set_defog queues");
    CHECK(g_iq[IQ_DEFOG].has_pending && g_iq[IQ_DEFOG].pending == 1, "defog queued as 1");

    /* Gain ceilings live outside the table and queue in star_state. */
    CHECK(hal_isp_set_max_again(c, 160) == RSS_OK, "set_max_again queues");
    CHECK(st.pend_max_again == 160, "max_again queued, got %d", st.pend_max_again);
    CHECK(hal_isp_set_max_dgain(c, 80) == RSS_OK, "set_max_dgain queues");
    CHECK(st.pend_max_dgain == 80, "max_dgain queued, got %d", st.pend_max_dgain);

    v32 = 0;
    CHECK(hal_isp_get_max_again(c, &v32) == RSS_OK, "get_max_again succeeds while queued");
    CHECK(v32 == 160, "queued max_again reads back, got %u", v32);

    vi = 0;
    CHECK(hal_isp_set_ae_comp(c, 140) == RSS_OK, "set_ae_comp queues");
    CHECK(hal_isp_get_ae_comp(c, &vi) == RSS_OK, "get_ae_comp succeeds while queued");
    CHECK(vi == 140, "queued ae_comp reads back, got %d", vi);

    /* Flip goes to the sensor, not the ISP, so it is tracked regardless
     * of ISP readiness -- but with no MI_SNR loaded it must not crash. */
    CHECK(hal_isp_get_hvflip(c, &vi, NULL) == RSS_OK, "get_hvflip succeeds");
    CHECK(vi == 0, "flip defaults off");

    /* A NULL context must be rejected rather than dereferenced. */
    CHECK(hal_isp_set_saturation(NULL, 200) == RSS_ERR_INVAL, "NULL ctx rejected");

    /* Leave the table clean for any later test. */
    memset(&st, 0, sizeof(st));
    for (size_t i = 0; i < IQ_PARAM_COUNT; i++)
        g_iq[i].has_pending = false;
}

/*
 * Orientation. MI_SNR_SetOrien takes both axes at once, so setting one has
 * to carry the other over from tracked state -- get that wrong and enabling
 * vflip silently cancels an hflip that is already in effect. The whole path
 * is function pointers, so it tests without hardware.
 */
static unsigned int orien_calls;
static unsigned char orien_last_mirror, orien_last_flip;
static int orien_ret;

static int fake_set_orien(unsigned int sensor, unsigned char mirror, unsigned char flip)
{
    (void)sensor;
    orien_calls++;
    orien_last_mirror = mirror;
    orien_last_flip = flip;
    return orien_ret;
}

static void test_orientation_carries_both_axes(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;
    void *c = &ctx;
    int hf, vf;

    memset(&ctx, 0, sizeof(ctx));
    memset(&st, 0, sizeof(st));
    ctx.platform = &st;
    st.snr.fnSetOrientation = fake_set_orien;
    orien_calls = 0;
    orien_ret = 0;

    CHECK(hal_isp_set_hflip(c, 1) == RSS_OK, "set_hflip succeeds");
    CHECK(orien_calls == 1, "set_hflip issues one SetOrien, got %u", orien_calls);
    CHECK(orien_last_mirror == 1 && orien_last_flip == 0, "hflip alone -> (1,0), got (%u,%u)",
          orien_last_mirror, orien_last_flip);

    /* The one that would regress: vflip must not drop the live hflip. */
    CHECK(hal_isp_set_vflip(c, 1) == RSS_OK, "set_vflip succeeds");
    CHECK(orien_last_mirror == 1 && orien_last_flip == 1, "vflip must keep hflip -> (1,1), "
          "got (%u,%u)", orien_last_mirror, orien_last_flip);

    hf = vf = -1;
    CHECK(hal_isp_get_hvflip(c, &hf, &vf) == RSS_OK, "get_hvflip succeeds");
    CHECK(hf == 1 && vf == 1, "get_hvflip reports both set, got (%d,%d)", hf, vf);

    /* Clearing one leaves the other alone. */
    CHECK(hal_isp_set_hflip(c, 0) == RSS_OK, "clearing hflip succeeds");
    CHECK(orien_last_mirror == 0 && orien_last_flip == 1, "clearing hflip keeps vflip -> (0,1), "
          "got (%u,%u)", orien_last_mirror, orien_last_flip);

    /* A failed SetOrien must not leave the tracked state claiming a change
     * that never reached the sensor, or the next set would carry a lie. */
    orien_ret = -1;
    CHECK(hal_isp_set_hflip(c, 1) != RSS_OK, "a failing SetOrien is reported");
    hf = -1;
    CHECK(hal_isp_get_hvflip(c, &hf, NULL) == RSS_OK, "get_hvflip still succeeds");
    CHECK(hf == 0, "a failed set must not be recorded, got %d", hf);

    /* No MI_SNR resolved at all is NOTSUP, not a crash. */
    orien_ret = 0;
    st.snr.fnSetOrientation = NULL;
    CHECK(hal_isp_set_hflip(c, 1) == RSS_ERR_NOTSUP, "missing SetOrien symbol is NOTSUP");
}

int main(void)
{
    test_table_bounds();
    test_table_matches_disassembly();
    test_field_access_is_surgical();
    test_scale_neutral_is_unity();
    test_scale_endpoints_and_monotonicity();
    test_unscale_round_trip();
    test_scale_degenerate_inputs();
    test_pending_queue();
    test_orientation_carries_both_axes();

    if (failures) {
        printf("\n%d check(s) failed\n", failures);
        return 1;
    }

    printf("all hal_isp logic tests passed\n");
    return 0;
}
