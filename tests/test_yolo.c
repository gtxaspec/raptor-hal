/*
 * test_yolo.c -- the shared YOLO decode (src/hal_ivs_yolo.c)
 *
 * Fixed head tensors in, the expected boxes out. The heads and the boxes are
 * in yolo_vectors.h, written by yolo_ref.py, a decode of its own on the
 * same formulas. Two layouts: two heads at strides 32 and 16 (persondet's)
 * and three at 8, 16 and 32 (YOLOv5's). Every packed output is allocated at
 * its exact length, so a read past it is an ASan error.
 */

#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

#include "greatest.h"
#include "hal_ivs_yolo.h"

#include "yolo_vectors.h"

/* hal_common.c's logger, counted */
static int log_errors;

static void count_log(int level, const char *file, int line, const char *fmt, ...)
{
    (void)file;
    (void)line;
    (void)fmt;
    if (level == HAL_LOG_LVL_ERR)
        log_errors++;
}

rss_hal_log_func_t rss_hal_log_fn = count_log;

/* one head, background everywhere but the given slots */
static float *head(const yv_slot_t *s, int ns, int gh, int gw)
{
    float *h = malloc(sizeof(float) * gh * gw * YV_C);
    for (int i = 0; i < gh * gw * YV_C; i++)
        h[i] = YV_BG;
    for (int i = 0; i < ns; i++)
        memcpy(h + (s[i].gy * gw + s[i].gx) * YV_C + s[i].a * YV_PER, s[i].v, sizeof(s[i].v));
    return h;
}

typedef struct {
    const yv_slot_t *slots;
    int ns, gh, gw;
} yv_head_t;

#define HEAD(slots, w, h, stride)                                                                  \
    {slots, sizeof(slots) / sizeof(slots[0]), (h) / (stride), (w) / (stride)}

static const yv_head_t two32 = HEAD(yv_two32, YV_TWO_W, YV_TWO_H, 32);
static const yv_head_t two16 = HEAD(yv_two16, YV_TWO_W, YV_TWO_H, 16);
static const yv_head_t three8 = HEAD(yv_three8, YV_THREE_W, YV_THREE_H, 8);
static const yv_head_t three16 = HEAD(yv_three16, YV_THREE_W, YV_THREE_H, 16);
static const yv_head_t three32 = HEAD(yv_three32, YV_THREE_W, YV_THREE_H, 32);

/* JZDL's packed output (type 76): the heads in order, then h w c per head
 * and the head count; exactly *n floats */
static float *pack(const yv_head_t *const *hs, int nh, size_t *n)
{
    size_t len = 0, at = 0;
    for (int k = 0; k < nh; k++)
        len += (size_t)hs[k]->gh * hs[k]->gw * YV_C;
    *n = len + 3 * nh + 1;
    float *out = malloc(sizeof(float) * *n);
    for (int k = 0; k < nh; k++) {
        size_t sz = (size_t)hs[k]->gh * hs[k]->gw * YV_C;
        float *h = head(hs[k]->slots, hs[k]->ns, hs[k]->gh, hs[k]->gw);
        memcpy(out + at, h, sizeof(float) * sz);
        free(h);
        at += sz;
    }
    for (int k = 0; k < nh; k++) {
        out[at++] = (float)hs[k]->gh;
        out[at++] = (float)hs[k]->gw;
        out[at++] = (float)YV_C;
    }
    out[at] = (float)nh;
    return out;
}

static void param(rss_ivs_jzdl_param_t *p, float conf, float nms)
{
    memset(p, 0, sizeof(*p));
    p->num_classes = YV_NC;
    p->conf_threshold = conf;
    p->nms_threshold = nms;
}

/* the result against the expected boxes: class, score, frame box */
static enum greatest_test_res same(const rss_ivs_detect_result_t *r, const yv_box_t *want, int n)
{
    ASSERT_EQ_FMT(n, r->count, "%d");
    for (int i = 0; i < n; i++) {
        const rss_ivs_detection_t *d = &r->detections[i];
        ASSERT_EQ_FMT(want[i].cls, d->class_id, "%d");
        ASSERT_IN_RANGE(want[i].score, d->confidence, 1e-5f);
        ASSERT_EQ_FMT(want[i].fx0, d->box.p0_x, "%d");
        ASSERT_EQ_FMT(want[i].fy0, d->box.p0_y, "%d");
        ASSERT_EQ_FMT(want[i].fx1, d->box.p1_x, "%d");
        ASSERT_EQ_FMT(want[i].fy1, d->box.p1_y, "%d");
    }
    PASS();
}

/* the candidates left by finish() against the expected boxes, model pixels */
static enum greatest_test_res same_cand(const hal_yolo_t *y, int got, const yv_box_t *want, int n)
{
    ASSERT_EQ_FMT(n, got, "%d");
    for (int i = 0; i < n; i++) {
        const hal_yolo_box_t *b = &y->cand[i];
        ASSERT_EQ_FMT(want[i].cls, b->cls, "%d");
        ASSERT_IN_RANGE(want[i].score, b->score, 1e-5f);
        ASSERT_IN_RANGE(want[i].x1, b->x1, 1e-3f);
        ASSERT_IN_RANGE(want[i].y1, b->y1, 1e-3f);
        ASSERT_IN_RANGE(want[i].x2, b->x2, 1e-3f);
        ASSERT_IN_RANGE(want[i].y2, b->y2, 1e-3f);
    }
    PASS();
}

/* hal_yolo_run() on a packed output, frame 640x360 */
static enum greatest_test_res run(const rss_ivs_jzdl_param_t *p, int iw, int ih,
                                  const yv_head_t *const *hs, int nh, const yv_box_t *want, int n)
{
    static hal_yolo_t y;
    rss_ivs_detect_result_t r;
    size_t len;
    float *out = pack(hs, nh, &len);

    hal_yolo_init(&y, p, iw, ih);
    log_errors = 0;
    int ret = hal_yolo_run(&y, out, len, 640, 360, &r);
    free(out);
    ASSERT_EQ_FMT(RSS_OK, ret, "%d");
    ASSERT_EQ_FMT(0, log_errors, "%d");
    ASSERT_EQ_FMT(nh, y.nheads, "%d");
    CHECK_CALL(same(&r, want, n));
    PASS();
}

TEST two_heads_persondet_order(void)
{
    rss_ivs_jzdl_param_t p;
    const yv_head_t *hs[] = {&two32, &two16};

    param(&p, 0.5f, 0.45f);
    CHECK_CALL(run(&p, YV_TWO_W, YV_TWO_H, hs, 2, yv_two, YV_TWO_N));
    PASS();
}

TEST two_heads_either_order(void)
{
    rss_ivs_jzdl_param_t p;
    const yv_head_t *hs[] = {&two16, &two32};

    param(&p, 0.5f, 0.45f);
    CHECK_CALL(run(&p, YV_TWO_W, YV_TWO_H, hs, 2, yv_two, YV_TWO_N));
    PASS();
}

/* The decode before the shared one read three heads at strides 8, 16 and 32
 * whatever the model: on a two-head output that is far past the end. */
TEST two_heads_not_read_as_three(void)
{
    size_t len, old = 0;
    const yv_head_t *hs[] = {&two32, &two16};
    float *out = pack(hs, 2, &len);

    free(out);
    for (int s = 8; s <= 32; s *= 2)
        old += (size_t)(YV_TWO_H / s) * (YV_TWO_W / s) * YV_C;
    ASSERT(old > len);
    /* and the exact-length buffer decodes clean under ASan */
    CHECK_CALL(two_heads_persondet_order());
    PASS();
}

TEST three_heads_yolov5_order(void)
{
    rss_ivs_jzdl_param_t p;
    const yv_head_t *hs[] = {&three8, &three16, &three32};

    param(&p, 0.5f, 0.45f);
    CHECK_CALL(run(&p, YV_THREE_W, YV_THREE_H, hs, 3, yv_three, YV_THREE_N));
    PASS();
}

TEST anchors_stock_unless_param_sets_a_stride(void)
{
    static const float stock[3][6] = {
        {10, 13, 16, 30, 33, 23}, {30, 61, 62, 45, 59, 119}, {116, 90, 156, 198, 373, 326}};
    static hal_yolo_t y;
    rss_ivs_jzdl_param_t p;

    param(&p, 0.5f, 0.45f);
    hal_yolo_init(&y, &p, YV_TWO_W, YV_TWO_H);
    ASSERT_MEM_EQ(stock, y.anchors, sizeof(stock));
    memcpy(p.anchors[2], yv_param32, sizeof(yv_param32));
    hal_yolo_init(&y, &p, YV_TWO_W, YV_TWO_H);
    ASSERT_MEM_EQ(stock[0], y.anchors[0], sizeof(stock[0]));
    ASSERT_MEM_EQ(stock[1], y.anchors[1], sizeof(stock[1]));
    ASSERT_MEM_EQ(yv_param32, y.anchors[2], sizeof(yv_param32));
    PASS();
}

TEST anchors_from_param_decode(void)
{
    rss_ivs_jzdl_param_t p;
    const yv_head_t *hs[] = {&two32, &two16};

    param(&p, 0.5f, 0.45f);
    memcpy(p.anchors[2], yv_param32, sizeof(yv_param32)); /* stride 32; 16 stays stock */
    CHECK_CALL(run(&p, YV_TWO_W, YV_TWO_H, hs, 2, yv_two_param, YV_TWO_PARAM_N));
    PASS();
}

TEST per_class_thresholds(void)
{
    rss_ivs_jzdl_param_t p;
    const yv_head_t *hs[] = {&two32, &two16};

    param(&p, 0.5f, 0.45f);
    memcpy(p.class_threshold, yv_clsthr, sizeof(yv_clsthr));
    CHECK_CALL(run(&p, YV_TWO_W, YV_TWO_H, hs, 2, yv_two_clsthr, YV_TWO_CLSTHR_N));
    PASS();
}

/* yv_two has a class-2 box under its better twin (IoU 0.6) and a class-1 box
 * over it (IoU 0.48): NMS at 0.45 drops the first only; at 1 neither */
TEST nms_per_class(void)
{
    static hal_yolo_t y;
    rss_ivs_jzdl_param_t p;
    rss_ivs_detect_result_t r;
    size_t len;
    const yv_head_t *hs[] = {&two32, &two16};
    float *out = pack(hs, 2, &len);

    param(&p, 0.5f, 1.0f);
    hal_yolo_init(&y, &p, YV_TWO_W, YV_TWO_H);
    ASSERT_EQ_FMT(RSS_OK, hal_yolo_run(&y, out, len, 640, 360, &r), "%d");
    free(out);
    ASSERT_EQ_FMT(YV_TWO_N + 1, r.count, "%d");
    ASSERT_EQ_FMT(2, r.detections[3].class_id, "%d");
    ASSERT_IN_RANGE(0.6f, r.detections[3].confidence, 1e-5f);
    PASS();
}

/* one tensor per head, then a letterbox: 1920x1080 into 96x64 is scale 0.05
 * with 5 rows of pad at the top */
TEST per_head_with_letterbox(void)
{
    static hal_yolo_t y, ref;
    rss_ivs_jzdl_param_t p;
    rss_ivs_detect_result_t r;
    const yv_head_t *hs[] = {&three8, &three16, &three32};
    size_t len;
    float *out = pack(hs, 3, &len);

    param(&p, 0.5f, 0.45f);
    hal_yolo_init(&y, &p, YV_THREE_W, YV_THREE_H);
    hal_yolo_begin(&y);
    for (int k = 0; k < 3; k++) {
        float *h = head(hs[k]->slots, hs[k]->ns, hs[k]->gh, hs[k]->gw);
        int ret = hal_yolo_decode_head(&y, h, hs[k]->gh, hs[k]->gw, YV_C);
        free(h);
        ASSERT_EQ_FMT(RSS_OK, ret, "%d");
    }
    int n = hal_yolo_finish(&y);
    CHECK_CALL(same_cand(&y, n, yv_three_lb, YV_THREE_LB_N));
    hal_yolo_emit(&y, n, 20.0f, 20.0f, 0.0f, 5.0f, 1920, 1080, &r);
    CHECK_CALL(same(&r, yv_three_lb, YV_THREE_LB_N));

    /* the same candidates as the packed path */
    hal_yolo_init(&ref, &p, YV_THREE_W, YV_THREE_H);
    ASSERT_EQ_FMT(RSS_OK, hal_yolo_run(&ref, out, len, 640, 360, &r), "%d");
    free(out);
    ASSERT_MEM_EQ(ref.cand, y.cand, sizeof(hal_yolo_box_t) * n);
    PASS();
}

TEST bad_heads_refused(void)
{
    static hal_yolo_t y;
    rss_ivs_jzdl_param_t p;
    float h[16 * 16 * 32];

    memset(h, 0, sizeof(h));
    param(&p, 0.5f, 0.45f);
    hal_yolo_init(&y, &p, YV_THREE_W, YV_THREE_H);
    hal_yolo_begin(&y);
    ASSERT_EQ(RSS_ERR, hal_yolo_decode_head(&y, h, 1, 1, YV_C));     /* stride 96 x 64 */
    ASSERT_EQ(RSS_ERR, hal_yolo_decode_head(&y, h, 4, 6, YV_C + 1)); /* not anchors x 8 */
    ASSERT_EQ(RSS_ERR, hal_yolo_decode_head(&y, h, 4, 6, 4 * 8));    /* 4 anchors */
    ASSERT_EQ(RSS_ERR, hal_yolo_decode_head(&y, h, 2, 6, YV_C));     /* 16 wide, 32 high */
    ASSERT_EQ(RSS_ERR, hal_yolo_decode_head(&y, h, 2, 2, YV_C));     /* stride 48 x 32 */
    ASSERT_EQ(RSS_OK, hal_yolo_decode_head(&y, h, 2, 3, 8));         /* one anchor is fine */
    PASS();
}

TEST bad_output_refused_logged_once(void)
{
    static hal_yolo_t y;
    rss_ivs_jzdl_param_t p;
    rss_ivs_detect_result_t r;
    size_t len;
    const yv_head_t *hs[] = {&two32, &two16};
    float *out = pack(hs, 2, &len);

    param(&p, 0.5f, 0.45f);
    hal_yolo_init(&y, &p, YV_TWO_W, YV_TWO_H);
    log_errors = 0;
    out[len - 1] = 3.0f; /* the trailer's head count off by one */
    ASSERT_EQ(RSS_ERR, hal_yolo_run(&y, out, len, 640, 360, &r));
    ASSERT_EQ_FMT(0, r.count, "%d");
    ASSERT_EQ(RSS_ERR, hal_yolo_run(&y, out, len, 640, 360, &r));
    ASSERT_EQ_FMT(1, log_errors, "%d");

    hal_yolo_init(&y, &p, YV_TWO_W, YV_TWO_H);
    out[len - 1] = 2.0f;
    out[len - 2] = 16.0f; /* the last head as 2 anchors: the sizes don't add up */
    ASSERT_EQ(RSS_ERR, hal_yolo_run(&y, out, len, 640, 360, &r));

    hal_yolo_init(&y, &p, YV_TWO_W, YV_TWO_H);
    ASSERT_EQ(RSS_ERR, hal_yolo_run(&y, out, 3, 640, 360, &r)); /* too short */
    free(out);
    PASS();
}

TEST results_capped(void)
{
    static hal_yolo_t y;
    rss_ivs_detect_result_t r;

    memset(&y, 0, sizeof(y));
    for (int i = 0; i < RSS_IVS_MAX_DETECTIONS + 5; i++)
        y.cand[i] = (hal_yolo_box_t){-4.0f, 1.0f, 50.0f, 70.0f, 0.9f, 0};
    hal_yolo_emit(&y, RSS_IVS_MAX_DETECTIONS + 5, 2.0f, 2.0f, 0.0f, 0.0f, 64, 128, &r);
    ASSERT_EQ_FMT(RSS_IVS_MAX_DETECTIONS, r.count, "%d");
    /* clamped to the frame */
    ASSERT_EQ_FMT(0, r.detections[0].box.p0_x, "%d");
    ASSERT_EQ_FMT(2, r.detections[0].box.p0_y, "%d");
    ASSERT_EQ_FMT(63, r.detections[0].box.p1_x, "%d");
    ASSERT_EQ_FMT(127, r.detections[0].box.p1_y, "%d");
    /* a box ending exactly on the frame's edge ends on its last pixel */
    y.cand[1] = (hal_yolo_box_t){0.0f, 0.0f, 32.0f, 64.0f, 0.8f, 0};
    hal_yolo_emit(&y, 2, 2.0f, 2.0f, 0.0f, 0.0f, 64, 128, &r);
    ASSERT_EQ_FMT(63, r.detections[1].box.p1_x, "%d");
    ASSERT_EQ_FMT(127, r.detections[1].box.p1_y, "%d");
    PASS();
}

SUITE(yolo_suite)
{
    RUN_TEST(two_heads_persondet_order);
    RUN_TEST(two_heads_either_order);
    RUN_TEST(two_heads_not_read_as_three);
    RUN_TEST(three_heads_yolov5_order);
    RUN_TEST(anchors_stock_unless_param_sets_a_stride);
    RUN_TEST(anchors_from_param_decode);
    RUN_TEST(per_class_thresholds);
    RUN_TEST(nms_per_class);
    RUN_TEST(per_head_with_letterbox);
    RUN_TEST(bad_heads_refused);
    RUN_TEST(bad_output_refused_logged_once);
    RUN_TEST(results_capped);
}
