/*
 * Host-side logic tests for src/star/hal_osd.c.
 *
 * Includes the real translation unit, so the code under test is the code
 * that ships. Everything MI-side is a function pointer in star_state_t's
 * i6_rgn_impl, which is what makes the region lifecycle testable without
 * hardware.
 *
 * What this is actually for, in priority order:
 *
 *   1. The BGRA -> MI conversion. hal_osd.c claims ARGB8888 is a straight
 *      copy because BGRA-in-memory *is* 0xAARRGGBB little-endian. If that
 *      claim is wrong every overlay comes out with swapped channels, and
 *      it is pure arithmetic a host can check exactly.
 *   2. The deferred attach. rvd registers regions before it binds the
 *      chain, so an implementation that attached eagerly would silently
 *      draw nothing -- the same class of bug as phase 4's missing port
 *      queue, which reached hardware because no test drove the real
 *      sequence.
 *   3. The format probe, which decides 1 on hardware and cannot be
 *      checked there without a board.
 *
 * As in t_isp.c/t_audio.c the i6_*.h _Static_asserts are suppressed via
 * -D'_Static_assert(c,m)=': they assert 32-bit pointer layouts, and the
 * real ARM build still checks every one.
 */

#define PLATFORM_INFINITY6E 1

#include "star/hal_osd.c"

#include <stdio.h>

/* HAL_LOG_* call through this with no NULL guard. */
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

/* ---- MI_RGN stand-ins ------------------------------------------------ */

#define TRACE_MAX 64

static struct {
    int create_calls, destroy_calls, attach_calls, detach_calls;
    int setchn_calls, setbmp_calls, deinit_calls;

    /* Last arguments seen. */
    unsigned int last_create_handle, last_destroy_handle;
    i6_rgn_cnf last_cnf;
    unsigned int last_attach_handle;
    i6_sys_bind last_port;
    i6_rgn_chn last_chn;
    i6_rgn_bmp last_bmp;

    /* Which formats MI_RGN_Create will accept; index is i6_rgn_pixfmt. */
    bool fmt_ok[I6_RGN_PIXFMT_END];

    /* Ordered trace of calls, for the teardown-ordering test. */
    char trace[TRACE_MAX][16];
    int trace_len;
} mi;

static void trace(const char *what)
{
    if (mi.trace_len < TRACE_MAX)
        snprintf(mi.trace[mi.trace_len++], sizeof(mi.trace[0]), "%s", what);
}

static int fake_create(unsigned int handle, i6_rgn_cnf *cnf)
{
    mi.create_calls++;
    mi.last_create_handle = handle;
    mi.last_cnf = *cnf;
    trace("create");

    return mi.fmt_ok[cnf->pixFmt] ? 0 : (int)0xA0052003;
}

static int fake_destroy(unsigned int handle)
{
    mi.destroy_calls++;
    mi.last_destroy_handle = handle;
    trace("destroy");
    return 0;
}

static int fake_attach(unsigned int handle, i6_sys_bind *port, i6_rgn_chn *chn)
{
    mi.attach_calls++;
    mi.last_attach_handle = handle;
    mi.last_port = *port;
    mi.last_chn = *chn;
    trace("attach");
    return 0;
}

static int fake_detach(unsigned int handle, i6_sys_bind *port)
{
    (void)handle;
    (void)port;
    mi.detach_calls++;
    trace("detach");
    return 0;
}

static int fake_setchn(unsigned int handle, i6_sys_bind *port, i6_rgn_chn *chn)
{
    (void)handle;
    (void)port;
    mi.setchn_calls++;
    mi.last_chn = *chn;
    trace("setchn");
    return 0;
}

static int fake_setbmp(unsigned int handle, i6_rgn_bmp *bmp)
{
    (void)handle;
    mi.setbmp_calls++;
    mi.last_bmp = *bmp;
    trace("setbmp");
    return 0;
}

static int fake_rgn_init(i6_rgn_pal *pal)
{
    (void)pal;
    return 0;
}

static int fake_rgn_deinit(void)
{
    mi.deinit_calls++;
    trace("deinit");
    return 0;
}

/*
 * Presets rgn_loaded/rgn_inited so the code under test does not dlopen
 * libmi_rgn.so, which does not exist on the host.
 */
static void setup(rss_hal_ctx_t *ctx, star_state_t *st)
{
    unsigned int i;

    memset(ctx, 0, sizeof(*ctx));
    memset(st, 0, sizeof(*st));
    memset(&mi, 0, sizeof(mi));

    for (i = 0; i < I6_RGN_PIXFMT_END; i++)
        mi.fmt_ok[i] = true;

    ctx->platform = st;
    st->rgn_loaded = true;
    st->rgn_inited = true;
    st->rgn.fnInit = fake_rgn_init;
    st->rgn.fnDeinit = fake_rgn_deinit;
    st->rgn.fnCreateRegion = fake_create;
    st->rgn.fnDestroyRegion = fake_destroy;
    st->rgn.fnAttachChannel = fake_attach;
    st->rgn.fnDetachChannel = fake_detach;
    st->rgn.fnSetChannelConfig = fake_setchn;
    st->rgn.fnSetBitmap = fake_setbmp;

    for (i = 0; i < I6_VENC_CHN_NUM; i++) {
        st->enc[i].src_port = -1;
        st->osd_src_port[i] = -1;
    }
}

/* Mark encoder channel `chn` as bound to VPE port `port`. */
static void bind_enc(star_state_t *st, int chn, int port)
{
    st->enc[chn].bound = true;
    st->enc[chn].src_port = port;
}

static rss_osd_region_t region_attr(int w, int h)
{
    rss_osd_region_t a;

    memset(&a, 0, sizeof(a));
    a.type = RSS_OSD_PIC;
    a.x = 10;
    a.y = 20;
    a.width = w;
    a.height = h;
    a.layer = 1;
    a.fg_alpha = 255;
    a.bg_alpha = 0;
    /*
     * global_alpha_en mirrors rvd: rvd_osd.c's create_region sets it true
     * for every region it makes. The helper used to leave it false, so no
     * test ever ran the configuration the board runs -- which is how the
     * const-alpha bug reached hardware. See
     * test_global_alpha_never_becomes_mi_const_alpha.
     */
    a.global_alpha_en = true;
    a.bitmap_fmt = RSS_PIXFMT_BGRA;

    return a;
}

/* ---- conversion ------------------------------------------------------ */

/*
 * The claim in hal_osd.c's PIXEL FORMAT note: rvd's BGRA byte order is
 * already MI's ARGB8888 when read as a little-endian word, so the 8888
 * path must be byte-identical. If this fails, overlays get swapped
 * channels on hardware and nothing else here would catch it.
 */
static void test_argb8888_is_byte_identical(void)
{
    const uint8_t src[8] = {0x11, 0x22, 0x33, 0x44, 0xAA, 0xBB, 0xCC, 0xDD};
    uint8_t dst[8];

    memset(dst, 0, sizeof(dst));
    star_osd_convert(src, dst, 2, I6_RGN_PIXFMT_ARGB888);

    CHECK(memcmp(src, dst, sizeof(src)) == 0, "ARGB8888 must be a straight copy");
    CHECK(star_osd_bpp(I6_RGN_PIXFMT_ARGB888) == 4, "ARGB8888 is 4 bytes per pixel");
}

static void test_argb4444_packing(void)
{
    /* B=0x1F G=0x2F R=0x3F A=0xFF -> A=F R=3 G=2 B=1 -> 0xF321 */
    const uint8_t src[4] = {0x1F, 0x2F, 0x3F, 0xFF};
    uint16_t dst = 0;

    star_osd_convert(src, &dst, 1, I6_RGN_PIXFMT_ARGB4444);
    CHECK(dst == 0xF321, "ARGB4444 packing: expected 0xF321, got %#04X", dst);
    CHECK(star_osd_bpp(I6_RGN_PIXFMT_ARGB4444) == 2, "ARGB4444 is 2 bytes per pixel");

    /* A fully transparent pixel keeps its alpha nibble at 0. */
    {
        const uint8_t clear[4] = {0xFF, 0xFF, 0xFF, 0x00};

        star_osd_convert(clear, &dst, 1, I6_RGN_PIXFMT_ARGB4444);
        CHECK((dst & 0xF000) == 0, "transparent pixel has alpha nibble 0, got %#04X", dst);
    }
}

static void test_argb1555_alpha_threshold(void)
{
    uint16_t dst = 0;

    /* Opaque white -> alpha bit set, all colour bits set. */
    {
        const uint8_t white[4] = {0xFF, 0xFF, 0xFF, 0xFF};

        star_osd_convert(white, &dst, 1, I6_RGN_PIXFMT_ARGB1555);
        CHECK(dst == 0xFFFF, "opaque white -> 0xFFFF, got %#04X", dst);
    }

    /* Fully transparent -> alpha bit clear. */
    {
        const uint8_t clear[4] = {0xFF, 0xFF, 0xFF, 0x00};

        star_osd_convert(clear, &dst, 1, I6_RGN_PIXFMT_ARGB1555);
        CHECK((dst & 0x8000) == 0, "transparent -> alpha bit clear, got %#04X", dst);
    }

    /*
     * The deliberate choice: *any* non-zero alpha is drawn. The opposite
     * threshold (>=128) erases the faint edge pixels antialiased text is
     * mostly made of, so small overlays come out ragged.
     */
    {
        const uint8_t faint[4] = {0x00, 0x00, 0xFF, 0x01};

        star_osd_convert(faint, &dst, 1, I6_RGN_PIXFMT_ARGB1555);
        CHECK((dst & 0x8000) != 0, "alpha 1 is still drawn, got %#04X", dst);
    }
}

/* ---- format probe ---------------------------------------------------- */

static void test_probe_prefers_no_conversion(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;
    rss_osd_region_t attr = region_attr(64, 32);
    int handle = -1;

    setup(&ctx, &st);

    CHECK(hal_osd_create_region(&ctx, &handle, &attr) == RSS_OK, "create succeeds");
    CHECK(st.rgn_fmt_known, "probe recorded a format");
    CHECK(st.rgn_fmt == I6_RGN_PIXFMT_ARGB888, "ARGB8888 preferred when accepted, got %d",
          st.rgn_fmt);

    /* The probe must not consume a real region slot, and must clean up
     * after itself. */
    CHECK(handle == 0, "first region gets slot 0, got %d", handle);
    CHECK(mi.destroy_calls == 1, "probe destroyed its trial region, destroys=%d", mi.destroy_calls);
}

static void test_probe_falls_back_through_the_list(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;
    rss_osd_region_t attr = region_attr(64, 32);
    int handle = -1;

    /* 8888 rejected -> 4444. */
    setup(&ctx, &st);
    mi.fmt_ok[I6_RGN_PIXFMT_ARGB888] = false;
    CHECK(hal_osd_create_region(&ctx, &handle, &attr) == RSS_OK, "create succeeds without 8888");
    CHECK(st.rgn_fmt == I6_RGN_PIXFMT_ARGB4444, "falls back to ARGB4444, got %d", st.rgn_fmt);

    /* 8888 and 4444 rejected -> 1555, which is what divinus uses. */
    setup(&ctx, &st);
    mi.fmt_ok[I6_RGN_PIXFMT_ARGB888] = false;
    mi.fmt_ok[I6_RGN_PIXFMT_ARGB4444] = false;
    CHECK(hal_osd_create_region(&ctx, &handle, &attr) == RSS_OK, "create succeeds on 1555 only");
    CHECK(st.rgn_fmt == I6_RGN_PIXFMT_ARGB1555, "falls back to ARGB1555, got %d", st.rgn_fmt);

    /* Nothing accepted: fail loudly rather than draw garbage. */
    setup(&ctx, &st);
    memset(mi.fmt_ok, 0, sizeof(mi.fmt_ok));
    CHECK(hal_osd_create_region(&ctx, &handle, &attr) == RSS_ERR_NOTSUP,
          "no usable format -> NOTSUP");
    CHECK(!st.osd[0].used, "no slot is claimed when create fails");
}

/* ---- the deferred attach -------------------------------------------- */

/*
 * The order rvd actually uses: create_group, create_region,
 * register_region, ... then bind. Registering before the bind cannot
 * attach, because the VPE port feeding the encoder is not known until
 * the bind happens -- so it must be remembered, not dropped.
 */
static void test_attach_waits_for_the_bind(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;
    rss_osd_region_t attr = region_attr(128, 32);
    int handle = -1;

    setup(&ctx, &st);

    CHECK(hal_osd_create_group(&ctx, 0) == RSS_OK, "create_group succeeds");
    CHECK(hal_osd_create_region(&ctx, &handle, &attr) == RSS_OK, "create_region succeeds");
    CHECK(hal_osd_register_region(&ctx, handle, 0) == RSS_OK,
          "register before bind succeeds (deferred, not failed)");
    CHECK(mi.attach_calls == 0, "nothing attached yet, attaches=%d", mi.attach_calls);
    CHECK(!st.osd[handle].attached, "region is not marked attached");
    CHECK(st.osd[handle].grp == 0, "but the group is remembered");

    /* A flush before the bind still cannot attach. */
    star_osd_flush_pending(&st, 0);
    CHECK(mi.attach_calls == 0, "flush without a bind attaches nothing");

    /* Now the bind exists: FS port 2 -> encoder 0. */
    bind_enc(&st, 0, 2);
    star_osd_flush_pending(&st, 0);

    CHECK(mi.attach_calls == 1, "flush after bind attaches once, attaches=%d", mi.attach_calls);
    CHECK(st.osd[handle].attached, "region is marked attached");
    CHECK(mi.last_attach_handle == (unsigned int)handle, "attached the right handle");

    /* The port has to name RGN's VPE module id (0) and the bound port,
     * not the encoder channel. Getting this wrong returns success and
     * draws nothing. */
    CHECK(mi.last_port.module == (i6_sys_mod)I6_RGN_MOD_VPE, "port names RGN's VPE module id");
    CHECK(mi.last_port.channel == STAR_VPE_CHN, "port names the VPE channel");
    CHECK(mi.last_port.port == 2, "port is the *bound* VPE port, got %u", mi.last_port.port);

    /* Flushing again must not double-attach. */
    star_osd_flush_pending(&st, 0);
    CHECK(mi.attach_calls == 1, "a second flush is a no-op, attaches=%d", mi.attach_calls);
}

static void test_register_attaches_immediately_when_running(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;
    rss_osd_region_t attr = region_attr(64, 16);
    int handle = -1;

    setup(&ctx, &st);
    bind_enc(&st, 1, 3);

    CHECK(hal_osd_create_region(&ctx, &handle, &attr) == RSS_OK, "create succeeds");
    CHECK(hal_osd_register_region(&ctx, handle, 1) == RSS_OK, "register succeeds");
    CHECK(mi.attach_calls == 1, "a region added to a running stream attaches at once");
    CHECK(mi.last_port.port == 3, "on the port that stream is bound to, got %u", mi.last_port.port);

    /* Only regions of the requested group are flushed. */
    mi.attach_calls = 0;
    star_osd_flush_pending(&st, 0);
    CHECK(mi.attach_calls == 0, "flushing a different group touches nothing");
}

/*
 * The regression test for the invisible-overlay bug.
 *
 * rvd sends global_alpha_en = true with bg_alpha = 0 and fg_alpha = 255.
 * Mapping that onto MI's constAlphaOn made every overlay fully
 * transparent: const-alpha mode ignores the bitmap's own alpha channel
 * and paints the whole rectangle with constAlpha, which shares a union
 * with bgFgAlpha and so was bg_alpha, i.e. 0.
 *
 * The union is also why no existing test caught it -- both branches wrote
 * the same two bytes, and constAlphaOn was the only observable
 * difference. So that flag is what this asserts.
 */
static void test_global_alpha_never_becomes_mi_const_alpha(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;
    rss_osd_region_t attr = region_attr(64, 16);
    int handle = -1;

    setup(&ctx, &st);
    bind_enc(&st, 0, 0);

    CHECK(attr.global_alpha_en, "the helper sends what rvd sends");
    CHECK(hal_osd_create_region(&ctx, &handle, &attr) == RSS_OK, "create succeeds");
    CHECK(hal_osd_register_region(&ctx, handle, 0) == RSS_OK, "register succeeds");

    CHECK(mi.last_chn.osd.constAlphaOn == 0,
          "per-pixel alpha stays on even when the caller asks for global alpha, got %d",
          mi.last_chn.osd.constAlphaOn);
    CHECK(mi.last_chn.osd.bgFgAlpha[0] == 0, "background alpha is bg_alpha, got %u",
          mi.last_chn.osd.bgFgAlpha[0]);
    CHECK(mi.last_chn.osd.bgFgAlpha[1] == 255, "foreground alpha is fg_alpha, got %u",
          mi.last_chn.osd.bgFgAlpha[1]);

    /* And it survives a display-attr push, which is the per-frame path. */
    attr.fg_alpha = 200;
    CHECK(hal_osd_set_region_attr(&ctx, handle, &attr) == RSS_OK, "push succeeds");
    CHECK(mi.last_chn.osd.constAlphaOn == 0, "a push does not turn const alpha back on");
    CHECK(mi.last_chn.osd.bgFgAlpha[1] == 200, "the new foreground alpha reached MI, got %u",
          mi.last_chn.osd.bgFgAlpha[1]);

    /* A COVER region is the thing that genuinely has one uniform alpha,
     * and it must keep using the cover half of the union. */
    {
        rss_osd_region_t cov = region_attr(32, 8);
        int ch = -1;

        cov.type = RSS_OSD_COVER;
        cov.cover_color = 0xFF123456u;
        CHECK(hal_osd_create_region(&ctx, &ch, &cov) == RSS_OK, "cover create succeeds");
        CHECK(hal_osd_register_region(&ctx, ch, 0) == RSS_OK, "cover register succeeds");
        CHECK(mi.last_chn.cover.color == 0xFF123456u, "cover colour reached MI, got %#x",
              mi.last_chn.cover.color);
    }
}

/* ---- attributes, show, data ----------------------------------------- */

static void test_resize_recreates_but_move_does_not(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;
    rss_osd_region_t attr = region_attr(64, 16);
    int handle = -1;

    setup(&ctx, &st);
    bind_enc(&st, 0, 0);
    hal_osd_create_region(&ctx, &handle, &attr);
    hal_osd_register_region(&ctx, handle, 0);

    /* A move/alpha change is a display-attr push: no destroy, no
     * re-attach, so the overlay does not blink. */
    mi.create_calls = mi.destroy_calls = mi.attach_calls = mi.detach_calls = mi.setchn_calls = 0;
    attr.x = 99;
    attr.fg_alpha = 128;
    CHECK(hal_osd_set_region_attr(&ctx, handle, &attr) == RSS_OK, "move succeeds");
    CHECK(mi.setchn_calls == 1, "move pushes the display attr, setchn=%d", mi.setchn_calls);
    CHECK(mi.create_calls == 0 && mi.destroy_calls == 0, "move does not recreate the region");
    CHECK(mi.last_chn.point.x == 99, "new position reached MI, got %u", mi.last_chn.point.x);
    CHECK(mi.last_chn.osd.bgFgAlpha[1] == 128, "new alpha reached MI, got %u",
          mi.last_chn.osd.bgFgAlpha[1]);

    /* A size change cannot be applied in place: MI fixes dimensions at
     * create time, so it must be destroy + create + re-attach. */
    mi.create_calls = mi.destroy_calls = mi.attach_calls = mi.detach_calls = 0;
    attr.width = 256;
    CHECK(hal_osd_set_region_attr(&ctx, handle, &attr) == RSS_OK, "resize succeeds");
    CHECK(mi.destroy_calls == 1 && mi.create_calls == 1, "resize recreates, d=%d c=%d",
          mi.destroy_calls, mi.create_calls);
    CHECK(mi.last_cnf.size.width == 256, "recreated at the new width, got %u",
          mi.last_cnf.size.width);
    CHECK(mi.detach_calls == 1 && mi.attach_calls == 1, "resize re-attaches, det=%d att=%d",
          mi.detach_calls, mi.attach_calls);
    CHECK(st.osd[handle].grp == 0, "the group survives a resize");
}

static void test_show_and_data(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;
    rss_osd_region_t attr = region_attr(4, 2);
    uint8_t bitmap[4 * 2 * 4];
    int handle = -1;
    void *first_bmp;

    setup(&ctx, &st);
    memset(bitmap, 0x7F, sizeof(bitmap));
    bind_enc(&st, 0, 1);
    hal_osd_create_region(&ctx, &handle, &attr);
    hal_osd_register_region(&ctx, handle, 0);

    /* rvd creates regions hidden and shows them when there is something
     * to draw. */
    CHECK(!st.osd[handle].show, "a new region starts hidden");

    mi.setchn_calls = 0;
    CHECK(hal_osd_show_region(&ctx, handle, 0, 1, 2) == RSS_OK, "show succeeds");
    CHECK(mi.setchn_calls == 1 && mi.last_chn.show == 1, "show reached MI");
    CHECK(mi.last_chn.osd.layer == 2, "layer reached MI, got %u", mi.last_chn.osd.layer);

    /* A NULL bitmap is rvd's "attr only" call, not an error. */
    CHECK(hal_osd_update_region_data(&ctx, handle, NULL) == RSS_OK, "NULL data is accepted");
    CHECK(mi.setbmp_calls == 0, "NULL data does not reach MI");

    CHECK(hal_osd_update_region_data(&ctx, handle, bitmap) == RSS_OK, "update succeeds");
    CHECK(mi.setbmp_calls == 1, "bitmap reached MI");
    CHECK(mi.last_bmp.pixFmt == st.rgn_fmt, "bitmap declares the probed format");
    CHECK(mi.last_bmp.size.width == 4 && mi.last_bmp.size.height == 2, "bitmap declares its size");
    CHECK(mi.last_bmp.data == st.osd[handle].bmp, "bitmap points at the region's own buffer");

    /* The per-region buffer is reused, so a per-frame overlay update does
     * not allocate. */
    first_bmp = st.osd[handle].bmp;
    hal_osd_update_region_data(&ctx, handle, bitmap);
    CHECK(st.osd[handle].bmp == first_bmp, "the conversion buffer is reused between updates");

    /* A cover region is a solid rectangle; MI has no bitmap for it. */
    {
        rss_osd_region_t cov = region_attr(8, 8);
        int ch = -1;

        cov.type = RSS_OSD_COVER;
        cov.cover_color = 0xFF00FF00;
        CHECK(hal_osd_create_region(&ctx, &ch, &cov) == RSS_OK, "cover region created");
        CHECK(mi.last_cnf.type == I6_RGN_TYPE_COVER, "created as an MI cover region");
        CHECK(hal_osd_update_region_data(&ctx, ch, bitmap) == RSS_ERR_NOTSUP,
              "a cover takes no bitmap");
        CHECK(hal_osd_destroy_region(&ctx, ch) == RSS_OK, "cover destroy succeeds");
    }

    /*
     * Destroy releases the conversion buffer. Asserted rather than merely
     * tidied up: it is the only allocation the OSD path owns, one per
     * region, reallocated on every geometry change -- and LeakSanitizer
     * is what noticed the test used to just walk away from it.
     */
    CHECK(hal_osd_destroy_region(&ctx, handle) == RSS_OK, "destroy succeeds");
    CHECK(st.osd[handle].bmp == NULL, "destroy released the conversion buffer");
}

static void test_slots_and_bad_handles(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;
    rss_osd_region_t attr = region_attr(8, 8);
    int handle = -1;
    int i;

    setup(&ctx, &st);

    for (i = 0; i < STAR_OSD_REGION_MAX; i++)
        CHECK(hal_osd_create_region(&ctx, &handle, &attr) == RSS_OK, "region %d created", i);

    CHECK(hal_osd_create_region(&ctx, &handle, &attr) == RSS_ERR_NOMEM,
          "the %dth region is refused", STAR_OSD_REGION_MAX + 1);

    /* Destroying one frees its slot for reuse. */
    CHECK(hal_osd_destroy_region(&ctx, 3) == RSS_OK, "destroy succeeds");
    CHECK(hal_osd_create_region(&ctx, &handle, &attr) == RSS_OK, "the freed slot is reusable");
    CHECK(handle == 3, "and it is the slot that was freed, got %d", handle);

    /* Handles that were never created, or already destroyed. */
    CHECK(hal_osd_destroy_region(&ctx, 99) == RSS_ERR_NOENT, "out-of-range handle -> NOENT");
    CHECK(hal_osd_show_region(&ctx, 99, 0, 1, 0) == RSS_ERR_NOENT, "show on a bad handle -> NOENT");
    CHECK(hal_osd_update_region_data(&ctx, 99, NULL) == RSS_ERR_NOENT,
          "update on a bad handle -> NOENT");

    /* Empty geometry is refused rather than passed to MI. */
    {
        rss_osd_region_t bad = region_attr(0, 8);

        CHECK(hal_osd_create_region(&ctx, &handle, &bad) == RSS_ERR_INVAL, "zero width refused");
    }

    CHECK(hal_osd_create_group(&ctx, I6_VENC_CHN_NUM) == RSS_ERR_INVAL, "group out of range");
}

/*
 * Teardown order: MI_RGN_DeInit with regions still attached leaves the
 * driver holding references, so detach must precede destroy, and both
 * must precede deinit.
 */
static void test_release_all_order(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;
    rss_osd_region_t attr = region_attr(16, 16);
    int handle = -1;
    int detach_at = -1, destroy_at = -1, deinit_at = -1;
    int i;

    setup(&ctx, &st);
    bind_enc(&st, 0, 0);
    hal_osd_create_region(&ctx, &handle, &attr);
    hal_osd_register_region(&ctx, handle, 0);

    mi.trace_len = 0;
    star_osd_release_all(&st);

    for (i = 0; i < mi.trace_len; i++) {
        if (!strcmp(mi.trace[i], "detach") && detach_at < 0)
            detach_at = i;
        if (!strcmp(mi.trace[i], "destroy") && destroy_at < 0)
            destroy_at = i;
        if (!strcmp(mi.trace[i], "deinit") && deinit_at < 0)
            deinit_at = i;
    }

    CHECK(detach_at >= 0 && destroy_at >= 0 && deinit_at >= 0,
          "teardown detaches, destroys and deinits (det=%d des=%d dei=%d)", detach_at, destroy_at,
          deinit_at);
    CHECK(detach_at < destroy_at, "detach before destroy");
    CHECK(destroy_at < deinit_at, "destroy before deinit");
    CHECK(!st.rgn_inited && !st.rgn_loaded, "teardown clears the loaded/inited flags");
    CHECK(!st.osd[handle].used, "region slots are released");

    /* Idempotent: a second teardown must not touch MI again. */
    mi.deinit_calls = 0;
    star_osd_release_all(&st);
    CHECK(mi.deinit_calls == 0, "a second teardown does nothing");
}

/*
 * destroy_group detaches its regions but leaves them created, because MI
 * regions are global objects and rvd tears one stream down while others
 * keep running.
 */
static void test_destroy_group_detaches_only(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;
    rss_osd_region_t attr = region_attr(16, 16);
    int a = -1, b = -1;

    setup(&ctx, &st);
    bind_enc(&st, 0, 0);
    bind_enc(&st, 1, 1);
    hal_osd_create_region(&ctx, &a, &attr);
    hal_osd_create_region(&ctx, &b, &attr);
    hal_osd_register_region(&ctx, a, 0);
    hal_osd_register_region(&ctx, b, 1);

    mi.detach_calls = mi.destroy_calls = 0;
    CHECK(hal_osd_destroy_group(&ctx, 0) == RSS_OK, "destroy_group succeeds");
    CHECK(mi.detach_calls == 1, "only group 0's region is detached, detaches=%d", mi.detach_calls);
    CHECK(mi.destroy_calls == 0, "regions survive their group");
    CHECK(st.osd[a].used && !st.osd[a].attached, "group 0's region is created but detached");
    CHECK(st.osd[b].attached, "group 1's region is untouched");
}

int main(void)
{
    test_argb8888_is_byte_identical();
    test_argb4444_packing();
    test_argb1555_alpha_threshold();
    test_probe_prefers_no_conversion();
    test_probe_falls_back_through_the_list();
    test_attach_waits_for_the_bind();
    test_register_attaches_immediately_when_running();
    test_global_alpha_never_becomes_mi_const_alpha();
    test_resize_recreates_but_move_does_not();
    test_show_and_data();
    test_slots_and_bad_handles();
    test_release_all_order();
    test_destroy_group_detaches_only();

    if (failures) {
        printf("\n%d check(s) failed\n", failures);
        return 1;
    }

    printf("all hal_osd logic tests passed\n");
    return 0;
}
