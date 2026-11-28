/*
 * test_yolo.c -- the shared YOLO decode (src/hal_ivs_yolo.c)
 *
 * Fixed head tensors in, the expected boxes out. The heads and the boxes are
 * in yolo_vectors.h, written by yolo_ref.py, a decode of its own on the
 * same formulas. Two layouts: two heads at strides 32 and 16 (persondet's)
 * and three at 8, 16 and 32 (YOLOv5's). Every packed output is allocated at
 * its exact length, so a read past it is an ASan error.
 */

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "greatest.h"
#include "hal_ivs_yolo.h"
#include "log_capture.h"

#include "yolo_vectors.h"

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

    ASSERT_EQ_FMT(RSS_OK, hal_yolo_init(&y, p, iw, ih), "%d");
    log_capture_reset();
    int ret = hal_yolo_run(&y, out, len, 640, 360, &r);
    free(out);
    ASSERT_EQ_FMT(RSS_OK, ret, "%d");
    ASSERT_EQ_FMT(0, logged.errors, "%d");
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
    ASSERT_EQ_FMT(RSS_OK, hal_yolo_init(&y, &p, YV_TWO_W, YV_TWO_H), "%d");
    ASSERT_MEM_EQ(stock, y.anchors, sizeof(stock));
    memcpy(p.anchors[2], yv_param32, sizeof(yv_param32));
    ASSERT_EQ_FMT(RSS_OK, hal_yolo_init(&y, &p, YV_TWO_W, YV_TWO_H), "%d");
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
    ASSERT_EQ_FMT(RSS_OK, hal_yolo_init(&y, &p, YV_TWO_W, YV_TWO_H), "%d");
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
    ASSERT_EQ_FMT(RSS_OK, hal_yolo_init(&y, &p, YV_THREE_W, YV_THREE_H), "%d");
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
    ASSERT_EQ_FMT(RSS_OK, hal_yolo_init(&ref, &p, YV_THREE_W, YV_THREE_H), "%d");
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
    ASSERT_EQ_FMT(RSS_OK, hal_yolo_init(&y, &p, YV_THREE_W, YV_THREE_H), "%d");
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
    ASSERT_EQ_FMT(RSS_OK, hal_yolo_init(&y, &p, YV_TWO_W, YV_TWO_H), "%d");
    log_capture_reset();
    out[len - 1] = 3.0f; /* the trailer's head count off by one */
    ASSERT_EQ(RSS_ERR, hal_yolo_run(&y, out, len, 640, 360, &r));
    ASSERT_EQ_FMT(0, r.count, "%d");
    ASSERT_EQ(RSS_ERR, hal_yolo_run(&y, out, len, 640, 360, &r));
    ASSERT_EQ_FMT(1, logged.errors, "%d");

    ASSERT_EQ_FMT(RSS_OK, hal_yolo_init(&y, &p, YV_TWO_W, YV_TWO_H), "%d");
    out[len - 1] = 2.0f;
    out[len - 2] = 16.0f; /* the last head as 2 anchors: the sizes don't add up */
    ASSERT_EQ(RSS_ERR, hal_yolo_run(&y, out, len, 640, 360, &r));

    ASSERT_EQ_FMT(RSS_OK, hal_yolo_init(&y, &p, YV_TWO_W, YV_TWO_H), "%d");
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

TEST preprocess_must_match_the_backend(void)
{
    rss_ivs_jzdl_param_t p;

    memset(&p, 0, sizeof(p));
    strcpy(p.model_path, "m.bin");
    log_capture_reset();
    /* zeroed param = stretch, what the JZDL backends do */
    ASSERT(hal_yolo_preprocess_ok(&p, RSS_IVS_PREPROCESS_STRETCH));
    ASSERT_FALSE(hal_yolo_preprocess_ok(&p, RSS_IVS_PREPROCESS_LETTERBOX));
    p.preprocess = RSS_IVS_PREPROCESS_LETTERBOX;
    ASSERT_FALSE(hal_yolo_preprocess_ok(&p, RSS_IVS_PREPROCESS_STRETCH));
    ASSERT(hal_yolo_preprocess_ok(&p, RSS_IVS_PREPROCESS_LETTERBOX));
    p.preprocess = (rss_ivs_preprocess_t)7;
    ASSERT_FALSE(hal_yolo_preprocess_ok(&p, RSS_IVS_PREPROCESS_STRETCH));
    ASSERT_EQ_FMT(3, logged.errors, "%d"); /* one line per refusal */
    PASS();
}

/* num_classes and anchors come from config unchecked: 5 + num_classes = 0
 * divided by zero in the decode (num_classes <= -5), a NaN or huge anchor
 * gave a box no int holds. init refuses them, and a refused decoder decodes
 * nothing. */
TEST config_out_of_range_refused(void)
{
    static hal_yolo_t y;
    rss_ivs_jzdl_param_t p;
    rss_ivs_detect_result_t r;
    const yv_head_t *hs[] = {&two32, &two16};
    const int bad_nc[] = {-5, -6, -1000, 0, HAL_YOLO_MAX_CLASSES + 1};
    const float bad_anchor[] = {-1.0f, NAN, INFINITY, HAL_YOLO_MAX_ANCHOR + 1.0f};
    float h[16 * 16 * 32];
    size_t len;
    float *out = pack(hs, 2, &len);

    memset(h, 0, sizeof(h));
    for (size_t i = 0; i < sizeof(bad_nc) / sizeof(bad_nc[0]); i++) {
        param(&p, 0.5f, 0.45f);
        p.num_classes = bad_nc[i];
        log_capture_reset();
        ASSERT_EQ_FMT(RSS_ERR, hal_yolo_init(&y, &p, YV_TWO_W, YV_TWO_H), "%d");
        ASSERT_EQ_FMT(1, logged.errors, "%d");
        ASSERT_EQ(RSS_ERR, hal_yolo_run(&y, out, len, 640, 360, &r));
        ASSERT_EQ_FMT(0, r.count, "%d");
        hal_yolo_begin(&y);
        ASSERT_EQ(RSS_ERR, hal_yolo_decode_head(&y, h, 2, 3, YV_C));
        ASSERT_EQ(RSS_ERR, hal_yolo_decode_head(&y, h, 2, 3, 5)); /* the c % 0 case */
    }
    for (size_t i = 0; i < sizeof(bad_anchor) / sizeof(bad_anchor[0]); i++) {
        param(&p, 0.5f, 0.45f);
        p.anchors[1][3] = bad_anchor[i];
        log_capture_reset();
        ASSERT_EQ_FMT(RSS_ERR, hal_yolo_init(&y, &p, YV_TWO_W, YV_TWO_H), "%d");
        ASSERT_EQ_FMT(1, logged.errors, "%d");
        ASSERT_EQ(RSS_ERR, hal_yolo_run(&y, out, len, 640, 360, &r));
    }
    param(&p, 0.5f, 0.45f);
    ASSERT_EQ_FMT(RSS_ERR, hal_yolo_init(&y, &p, 0, YV_TWO_H), "%d");
    /* the edges are fine */
    p.num_classes = 1;
    ASSERT_EQ_FMT(RSS_OK, hal_yolo_init(&y, &p, YV_TWO_W, YV_TWO_H), "%d");
    p.num_classes = HAL_YOLO_MAX_CLASSES;
    p.anchors[2][0] = HAL_YOLO_MAX_ANCHOR;
    ASSERT_EQ_FMT(RSS_OK, hal_yolo_init(&y, &p, YV_TWO_W, YV_TWO_H), "%d");
    free(out);
    PASS();
}

/* NaN logits: a NaN score is false against any threshold, so `score < t`
 * let it through to a NaN box and an undefined float-to-int cast. Each
 * logit of one cell set to NaN in turn: never a detection. */
TEST nan_logits_dropped(void)
{
    static hal_yolo_t y;
    rss_ivs_jzdl_param_t p;
    rss_ivs_detect_result_t r;
    const int gh = YV_THREE_H / 32, gw = YV_THREE_W / 32;
    float *h = malloc(sizeof(float) * gh * gw * YV_C);

    param(&p, 0.25f, 0.45f);
    /* x, y, w, h, objectness and class 0 (the one every class ties with) */
    for (int k = 0; k < 6; k++) {
        ASSERT_EQ_FMT(RSS_OK, hal_yolo_init(&y, &p, YV_THREE_W, YV_THREE_H), "%d");
        for (int i = 0; i < gh * gw * YV_C; i++)
            h[i] = YV_BG;
        for (int i = 0; i < YV_PER; i++)
            h[i] = 4.0f; /* cell 0, anchor 0: a sure box */
        h[k] = NAN;
        hal_yolo_begin(&y);
        ASSERT_EQ_FMT(RSS_OK, hal_yolo_decode_head(&y, h, gh, gw, YV_C), "%d");
        int n = hal_yolo_finish(&y);
        ASSERT_EQ_FMT(0, n, "%d");
        hal_yolo_emit(&y, n, 1.0f, 1.0f, 0.0f, 0.0f, 640, 360, &r);
        ASSERT_EQ_FMT(0, r.count, "%d");
    }
    /* the same cell without the NaN is found */
    h[5] = 4.0f;
    hal_yolo_begin(&y);
    ASSERT_EQ_FMT(RSS_OK, hal_yolo_decode_head(&y, h, gh, gw, YV_C), "%d");
    ASSERT_EQ_FMT(1, hal_yolo_finish(&y), "%d");
    /* a NaN in a class that doesn't win leaves the winner's score */
    h[6] = NAN;
    hal_yolo_begin(&y);
    ASSERT_EQ_FMT(RSS_OK, hal_yolo_decode_head(&y, h, gh, gw, YV_C), "%d");
    ASSERT_EQ_FMT(1, hal_yolo_finish(&y), "%d");
    ASSERT_EQ_FMT(0, y.cand[0].cls, "%d");
    ASSERT(isfinite(y.cand[0].score));
    free(h);

    /* a NaN or infinite box that reached emit anyway is clamped, no UB cast */
    memset(&y, 0, sizeof(y));
    y.cand[0] = (hal_yolo_box_t){NAN, -INFINITY, INFINITY, 1e30f, 0.9f, 0};
    hal_yolo_emit(&y, 1, 1.0f, 1.0f, 0.0f, 0.0f, 640, 360, &r);
    ASSERT_EQ_FMT(0, r.detections[0].box.p0_x, "%d");
    ASSERT_EQ_FMT(0, r.detections[0].box.p0_y, "%d");
    ASSERT_EQ_FMT(639, r.detections[0].box.p1_x, "%d");
    ASSERT_EQ_FMT(359, r.detections[0].box.p1_y, "%d");
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
    RUN_TEST(preprocess_must_match_the_backend);
    RUN_TEST(config_out_of_range_refused);
    RUN_TEST(nan_logits_dropped);
}
