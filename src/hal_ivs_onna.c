/*
 * hal_ivs_onna.c -- Standalone detection on the T41 NNA through onna, an open runtime
 *
 * Same C API as hal_ivs_jzdl.cpp (hal_jzdl_create / _detect / _destroy), for the
 * T41's neural accelerator, without Ingenic's venus/Magik libraries, without an
 * nmem reservation and without changing the L2 cache split:
 *   - onna (src/onna/) drives the NNA with the MXU3 NN instructions directly.
 *     It runs venus .bin models of the QSUM kind (ReLU, the NNA requantizes)
 *     with output byte-identical to venus on the camera; model_path names the
 *     .bin; an onnaq text graph next to it (model.qgraph for model.bin, else
 *     model.bin.qgraph) is optional: without one onnaq reads the .bin's own op list;
 *   - the frame is resized as ojzdl_resize does (= libjzdl's jzdl::resize), so a
 *     model trained for both SoCs sees the same input on the T31 and the T41;
 *   - every output is a YOLOv5-style head (NHWC, 3 anchors x (5 + classes)); the
 *     stride comes from its grid, the anchors from the caller (param->anchors: rvd [motion]
 *     anchors_<stride>, e.g. from the model's model.conf), else the graph, else YOLOv5's defaults
 *     for strides 8/16/32 when it names none).
 *
 * The NN instructions only run on core 0: detect pins its thread (rvd's
 * inference thread) to core 0 for good, and create pins the ISP firmware thread
 * (isp_fw_process) to core 0 too. Without that, NN work on core 0 next to the
 * ISP firmware's MXU3 code on core 1 can deadlock both cores (watchdog reset).
 * One context at a time (the runtime keeps one model).
 *
 * Build with ONNA=1 (T41 only), instead of JZDL_INCLUDE or OJZDL=1.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: MIT
 */

#define _GNU_SOURCE
#include "hal_internal.h"
#include "ojzdl/ojzdl.h"
#include "onna/onnaq.h"

#include <math.h>
#include <sched.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

typedef struct {
    float x1, y1, x2, y2;
    float score;
    int classid;
} det_box_t;

typedef struct {
    det_box_t *v;
    size_t n, cap;
} box_vec_t;

static int box_push(box_vec_t *b, const det_box_t *d)
{
    if (b->n == b->cap) {
        size_t cap = b->cap ? b->cap * 2 : 64;
        det_box_t *nv = realloc(b->v, cap * sizeof(*nv));
        if (!nv)
            return -1;
        b->v = nv;
        b->cap = cap;
    }
    b->v[b->n++] = *d;
    return 0;
}

static inline float sigmoid(float x)
{
    return 1.0f / (1.0f + expf(-x));
}

/* one head: p = NHWC [gh][gw][3 * (5 + nc)], YOLOv5 v5+ box formula */
static void head_decode(const float *p, int gh, int gw, int stride, const float anc[6], int nc,
                        float conf_thresh, box_vec_t *boxes)
{
    int oc = 5 + nc;
    /* objectness and class score are at most 1: skip a cell whose raw
     * objectness is under the threshold's logit */
    float obj_min = logf(conf_thresh / (1.0f - conf_thresh));

    for (int y = 0; y < gh; y++)
        for (int x = 0; x < gw; x++)
            for (int a = 0; a < 3; a++) {
                const float *q = p + ((size_t)(y * gw + x) * 3 + a) * oc;
                if (q[4] < obj_min)
                    continue;
                int bc = 0;
                for (int c = 1; c < nc; c++)
                    if (q[5 + c] > q[5 + bc])
                        bc = c;
                float s = sigmoid(q[4]) * sigmoid(q[5 + bc]);
                if (s < conf_thresh)
                    continue;
                float cx = (sigmoid(q[0]) * 2.0f - 0.5f + x) * stride;
                float cy = (sigmoid(q[1]) * 2.0f - 0.5f + y) * stride;
                float bw = sigmoid(q[2]) * 2.0f, bh = sigmoid(q[3]) * 2.0f;
                bw = bw * bw * anc[2 * a];
                bh = bh * bh * anc[2 * a + 1];
                det_box_t d = {cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2, s, bc};
                if (box_push(boxes, &d))
                    return;
            }
}

static int score_desc(const void *a, const void *b)
{
    float sa = ((const det_box_t *)a)->score, sb = ((const det_box_t *)b)->score;
    return (sa < sb) - (sa > sb);
}

/* class-aware greedy NMS; input sorted in place */
static void nms(box_vec_t *in, box_vec_t *out, unsigned char *gone, float thr)
{
    qsort(in->v, in->n, sizeof(*in->v), score_desc);
    memset(gone, 0, in->n);
    for (size_t i = 0; i < in->n; i++) {
        if (gone[i])
            continue;
        const det_box_t *bi = &in->v[i];
        if (box_push(out, bi))
            return;
        float ai = (bi->x2 - bi->x1) * (bi->y2 - bi->y1);
        for (size_t j = i + 1; j < in->n; j++) {
            const det_box_t *bj = &in->v[j];
            if (gone[j] || bj->classid != bi->classid)
                continue;
            float iw = fmaxf(0.0f, fminf(bi->x2, bj->x2) - fmaxf(bi->x1, bj->x1));
            float ih = fmaxf(0.0f, fminf(bi->y2, bj->y2) - fmaxf(bi->y1, bj->y1));
            float inter = iw * ih;
            float aj = (bj->x2 - bj->x1) * (bj->y2 - bj->y1);
            if (inter / (ai + aj - inter) > thr)
                gone[j] = 1;
        }
    }
}

/* NV12 -> RGB one row at a time (the formula of hal_ivs_jzdl.cpp's nv12_to_rgb) */
static inline uint8_t clamp_u8(int v)
{
    return v < 0 ? 0 : (v > 255 ? 255 : (uint8_t)v);
}

static void nv12_row_to_rgb(const uint8_t *nv12, uint8_t *rgb, int w, int h, int row)
{
    const uint8_t *yp = nv12 + (size_t)row * w;
    const uint8_t *uvp = nv12 + (size_t)w * h + (size_t)(row / 2) * w;

    for (int col = 0; col < w; col++) {
        int y = yp[col], u = uvp[col & ~1] - 128, v = uvp[col | 1] - 128;
        rgb[col * 3 + 0] = clamp_u8(y + ((359 * v) >> 8));
        rgb[col * 3 + 1] = clamp_u8(y - ((88 * u + 183 * v) >> 8));
        rgb[col * 3 + 2] = clamp_u8(y + ((454 * u) >> 8));
    }
}

#define ONNA_MAX_HEADS 4

typedef struct {
    int input_w, input_h, frame_w, frame_h;
    float conf_thresh, nms_thresh;
    int nheads, nc, stride[ONNA_MAX_HEADS];
    float anc[ONNA_MAX_HEADS][6];
    float panc[3][6]; /* the caller's anchors for strides 8, 16, 32 (param->anchors; 0 = none) */
    uint8_t *rgb_row, *in_buf;
    ojzdl_resize *rs;
    const uint8_t *nv12;
    box_vec_t candidates, detections;
    unsigned char *gone;
    size_t gone_cap;
    int pinned;
    /* mean times, logged every 100 inferences */
    double t_pre, t_net, t_post;
    int nt;
} onna_ctx_t;

static int in_use;

static const uint8_t *nv12_get_row(void *ctx, int y)
{
    onna_ctx_t *c = ctx;
    nv12_row_to_rgb(c->nv12, c->rgb_row, c->frame_w, c->frame_h, y);
    return c->rgb_row;
}

static void ctx_free(onna_ctx_t *c)
{
    if (!c)
        return;
    free(c->rgb_row);
    free(c->in_buf);
    ojzdl_resize_free(c->rs);
    free(c->candidates.v);
    free(c->detections.v);
    free(c->gone);
    free(c);
}

static double now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1e3 + ts.tv_nsec / 1e6;
}

static int pin_cpu0(cpu_set_t *old)
{
    cpu_set_t cs;
    if (old && sched_getaffinity(0, sizeof(*old), old))
        return -1;
    CPU_ZERO(&cs);
    CPU_SET(0, &cs);
    return sched_setaffinity(0, sizeof(cs), &cs);
}

/* model.bin -> model.qgraph, else model.bin.qgraph, else "" (no graph file: onnaq builds the
 * graph from the .bin's own op list) */
static int graph_path(const char *model, char *out, size_t n)
{
    size_t l = strlen(model);
    if (l > 4 && !strcmp(model + l - 4, ".bin")) {
        if ((size_t)snprintf(out, n, "%.*s.qgraph", (int)(l - 4), model) < n && !access(out, R_OK))
            return 0;
    }
    if ((size_t)snprintf(out, n, "%s.qgraph", model) >= n)
        return -1;
    if (access(out, R_OK))
        out[0] = 0;
    return 0;
}

void *hal_jzdl_create(const rss_ivs_jzdl_param_t *param)
{
    char gpath[300];
    cpu_set_t old;
    onna_ctx_t *c;
    int rc;

    if (!param)
        return NULL;
    if (strnlen(param->model_path, sizeof(param->model_path)) >= sizeof(param->model_path)) {
        HAL_LOG_ERR("ONNA: model_path not null-terminated");
        return NULL;
    }
    if (param->width <= 1 || param->height <= 1 || param->width > 8192 || param->height > 8192) {
        HAL_LOG_ERR("ONNA: invalid frame dimensions %dx%d", param->width, param->height);
        return NULL;
    }
    if (in_use) {
        HAL_LOG_ERR("ONNA: one model at a time");
        return NULL;
    }
    if (graph_path(param->model_path, gpath, sizeof(gpath))) {
        HAL_LOG_ERR("ONNA: model path too long");
        return NULL;
    }
    c = calloc(1, sizeof(*c));
    if (!c)
        return NULL;

    /* loading does no NN work, but keep every runtime call on core 0 */
    int restore = !pin_cpu0(&old);
#ifdef ONNA_MXU3
    rc = onnaq_open(param->model_path, gpath[0] ? gpath : NULL, 1);
#else /* host test build: the same network on the CPU */
    rc = onnaq_open(param->model_path, gpath[0] ? gpath : NULL, 0);
#endif
    if (restore)
        sched_setaffinity(0, sizeof(old), &old);
    if (rc) {
        HAL_LOG_ERR("ONNA: failed to load %s + %s: %s", param->model_path, gpath[0] ? gpath : "(graph from the .bin)",
                    onnaq_error());
        free(c);
        return NULL;
    }
    in_use = 1;
    onnaq_input_size(&c->input_w, &c->input_h);
    c->nheads = onnaq_num_heads();
    if (c->nheads < 1 || c->nheads > ONNA_MAX_HEADS)
        goto bad_heads;
    /* head shapes are known after the first run (heads_setup) */

    memcpy(c->panc, param->anchors, sizeof(c->panc));
    c->frame_w = param->width;
    c->frame_h = param->height;
    c->conf_thresh = param->conf_threshold;
    c->nms_thresh = param->nms_threshold;
    c->rgb_row = malloc((size_t)c->frame_w * 3);
    c->in_buf = malloc((size_t)c->input_w * c->input_h * 3);
    c->rs = ojzdl_resize_create(c->frame_w, c->frame_h, c->input_w, c->input_h);
    if (!c->rgb_row || !c->in_buf || !c->rs) {
        onnaq_close();
        in_use = 0;
        ctx_free(c);
        return NULL;
    }
    HAL_LOG_INFO("ONNA: ready: %s (%dx%d -> %dx%d, %d heads, conf=%.2f, nms=%.2f)",
                 param->model_path, c->frame_w, c->frame_h, c->input_w, c->input_h, c->nheads,
                 (double)c->conf_thresh, (double)c->nms_thresh);
    return c;

bad_heads:
    HAL_LOG_ERR("ONNA: %s: %d outputs (1..%d supported)", param->model_path, c->nheads, ONNA_MAX_HEADS);
    onnaq_close();
    in_use = 0;
    free(c);
    return NULL;
}

/* after the first run: head shapes -> strides, classes, anchors */
static int heads_setup(onna_ctx_t *c)
{
    for (int k = 0; k < c->nheads; k++) {
        int h, w, ch;
        if (!onnaq_head(k, &h, &w, &ch) || h < 1 || w < 1 || ch % 3 || ch / 3 < 6)
            goto bad;
        int nc = ch / 3 - 5;
        if (k && nc != c->nc)
            goto bad;
        c->nc = nc;
        c->stride[k] = c->input_h / h;
        if (c->stride[k] * h != c->input_h || c->stride[k] * w != c->input_w)
            goto bad;
        /* anchors: the caller's ([motion] anchors_<stride>, e.g. from the model's model.conf),
         * else the graph's anchors line, else YOLOv5's (onnaq_anchors) */
        int si = c->stride[k] == 8 ? 0 : c->stride[k] == 16 ? 1 : c->stride[k] == 32 ? 2 : -1, set = 0;
        for (int j = 0; si >= 0 && j < 6; j++)
            set |= c->panc[si][j] != 0.0f;
        if (set) {
            memcpy(c->anc[k], c->panc[si], sizeof(c->anc[k]));
        } else if (onnaq_anchors(c->stride[k], c->anc[k])) {
            HAL_LOG_ERR("ONNA: no anchors for stride %d ([motion] anchors_%d or the graph)", c->stride[k],
                        c->stride[k]);
            return -1;
        }
    }
    HAL_LOG_INFO("ONNA: %d classes, %d heads, strides %d %d %d %d (0: none)", c->nc, c->nheads,
                 c->stride[0], c->stride[1], c->stride[2], c->stride[3]);
    return 0;
bad:
    HAL_LOG_ERR("ONNA: outputs are not YOLOv5-style heads (NHWC, 3 x (5 + classes))");
    return -1;
}

int hal_jzdl_detect(void *handle, const uint8_t *nv12_data, rss_ivs_detect_result_t *result)
{
    onna_ctx_t *c = handle;

    if (!handle || !nv12_data || !result)
        return RSS_ERR_INVAL;
    result->count = 0;
    if (!c->pinned) {
        if (pin_cpu0(NULL)) {
            HAL_LOG_ERR("ONNA: cannot pin the inference thread to core 0");
            return RSS_ERR;
        }
        c->pinned = 1;
    }

    double t0 = now_ms();
    c->nv12 = nv12_data;
    if (ojzdl_resize_rows(c->rs, nv12_get_row, c, c->in_buf))
        return RSS_ERR;
    double t1 = now_ms();
    if (onnaq_run(c->in_buf)) {
        HAL_LOG_ERR("ONNA: run failed: %s", onnaq_error());
        return RSS_ERR;
    }
    double t2 = now_ms();
    if (!c->nc && heads_setup(c)) {
        c->nc = -1;
        return RSS_ERR;
    }
    if (c->nc < 0)
        return RSS_ERR;

    c->candidates.n = 0;
    c->detections.n = 0;
    for (int k = 0; k < c->nheads; k++) {
        int h, w, ch;
        const float *p = onnaq_head(k, &h, &w, &ch);
        head_decode(p, h, w, c->stride[k], c->anc[k], c->nc, c->conf_thresh, &c->candidates);
    }
    if (c->candidates.n > c->gone_cap) {
        unsigned char *g = realloc(c->gone, c->candidates.n);
        if (!g)
            return RSS_ERR;
        c->gone = g;
        c->gone_cap = c->candidates.n;
    }
    nms(&c->candidates, &c->detections, c->gone, c->nms_thresh);

    float sx = (float)c->frame_w / c->input_w, sy = (float)c->frame_h / c->input_h;
    int n = (int)c->detections.n;
    if (n > RSS_IVS_MAX_DETECTIONS)
        n = RSS_IVS_MAX_DETECTIONS;
    for (int i = 0; i < n; i++) {
        const det_box_t *d = &c->detections.v[i];
        int x0 = (int)(d->x1 * sx), y0 = (int)(d->y1 * sy);
        int x1 = (int)(d->x2 * sx), y1 = (int)(d->y2 * sy);
        x0 = x0 < 0 ? 0 : x0;
        y0 = y0 < 0 ? 0 : y0;
        x1 = x1 >= c->frame_w ? c->frame_w - 1 : x1;
        y1 = y1 >= c->frame_h ? c->frame_h - 1 : y1;
        result->detections[i].box = (rss_rect_t){x0, y0, x1, y1};
        result->detections[i].confidence = d->score;
        result->detections[i].class_id = d->classid;
    }
    result->count = n;
    result->timestamp = 0; /* caller sets this */

    c->t_pre += t1 - t0;
    c->t_net += t2 - t1;
    c->t_post += now_ms() - t2;
    if (++c->nt == 100) {
        HAL_LOG_INFO("ONNA: mean of 100: resize %.1f ms, network %.1f ms, decode %.1f ms",
                     c->t_pre / 100, c->t_net / 100, c->t_post / 100);
        c->t_pre = c->t_net = c->t_post = 0;
        c->nt = 0;
    }
    return RSS_OK;
}

void hal_jzdl_destroy(void *handle)
{
    if (!handle)
        return;
    onnaq_close();
    in_use = 0;
    ctx_free(handle);
    HAL_LOG_INFO("ONNA: destroyed");
}
