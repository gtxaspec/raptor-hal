/*
 * hal_ivs_ojzdl.c -- Standalone JZDL inference on ojzdl, the open runtime
 *
 * Same C API and results as hal_ivs_jzdl.cpp (hal_jzdl_create / _detect /
 * _destroy), without Ingenic's libjzdl.m.so or libstdc++:
 *   - ojzdl runs JZDL v4/v5 models with output bit-identical to libjzdl
 *     (src/ojzdl/, MXU2 kernels on the T31);
 *   - ojzdl_resize_nv12 converts the NV12 frame with the C++ backend's formula
 *     and reproduces jzdl::resize exactly for the sizes raptor uses
 *     (640x360 -> 416x320 / 320x416);
 *   - YOLOv5 decode and NMS are the same code as the C++ backend.
 * Build with OJZDL=1 instead of JZDL_INCLUDE.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: MIT
 */

#include "hal_internal.h"
#include "ojzdl/ojzdl.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

/* ================================================================
 * YOLOv5 post-processing (same as hal_ivs_jzdl.cpp)
 * ================================================================ */

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

static void yolo_decode(const float *p, box_vec_t *boxes, int input_w, int input_h,
                        float conf_thresh, int num_classes)
{
    static const float anchors[] = {10, 13, 16,  30,  33, 23,  30,  61,  62,
                                    45, 59, 119, 116, 90, 156, 198, 373, 326};
    static const float strides[] = {8.0f, 16.0f, 32.0f};
    int box_num = 3;
    int onechannel = 5 + num_classes;

    for (int s = 0; s < 3; s++) {
        int gh = input_h / (int)strides[s];
        int gw = input_w / (int)strides[s];

        for (int h = 0; h < gh; h++) {
            for (int w = 0; w < gw; w++) {
                for (int n = 0; n < box_num; n++) {
                    int idx = h * (gw * box_num * onechannel) + w * (box_num * onechannel) +
                              n * onechannel;

                    float bx = 1.0f / (1.0f + expf(-p[idx + 0]));
                    float by = 1.0f / (1.0f + expf(-p[idx + 1]));
                    float bw = 1.0f / (1.0f + expf(-p[idx + 2]));
                    float bh = 1.0f / (1.0f + expf(-p[idx + 3]));
                    float obj = 1.0f / (1.0f + expf(-p[idx + 4]));

                    float aw = anchors[s * box_num * 2 + n * 2] / input_w;
                    float ah = anchors[s * box_num * 2 + n * 2 + 1] / input_h;

                    float box_w = (bw * 2.0f) * (bw * 2.0f) * aw;
                    float box_h = (bh * 2.0f) * (bh * 2.0f) * ah;
                    float cx = (bx * 2.0f - 0.5f + w) / gw;
                    float cy = (by * 2.0f - 0.5f + h) / gh;

                    int best_cls = 0;
                    float best_score = 0;
                    for (int c = 0; c < num_classes; c++) {
                        float cs = 1.0f / (1.0f + expf(-p[idx + 5 + c]));
                        if (cs > best_score) {
                            best_score = cs;
                            best_cls = c;
                        }
                    }

                    float prob = obj * best_score;
                    if (prob >= conf_thresh) {
                        det_box_t d;
                        d.x1 = (cx - box_w * 0.5f) * input_w;
                        d.y1 = (cy - box_h * 0.5f) * input_h;
                        d.x2 = (cx + box_w * 0.5f) * input_w;
                        d.y2 = (cy + box_h * 0.5f) * input_h;
                        d.score = prob;
                        d.classid = best_cls;
                        if (box_push(boxes, &d))
                            return;
                    }
                }
            }
        }
        p += gh * gw * box_num * onechannel;
    }
}

/* sort by score, descending: in-place quicksort with an insertion-sort
 * finish (qsort's per-compare callback is slow on the T31) */
static void box_sort_desc(det_box_t *v, size_t n)
{
    while (n > 16) {
        float pivot = v[n / 2].score;
        size_t i = 0, j = n - 1;
        for (;;) {
            while (v[i].score > pivot)
                i++;
            while (v[j].score < pivot)
                j--;
            if (i >= j)
                break;
            det_box_t t = v[i];
            v[i] = v[j];
            v[j] = t;
            i++;
            j--;
        }
        /* recurse into the smaller part, loop on the larger */
        if (j + 1 < n - j - 1) {
            box_sort_desc(v, j + 1);
            v += j + 1;
            n -= j + 1;
        } else {
            box_sort_desc(v + j + 1, n - j - 1);
            n = j + 1;
        }
    }
    for (size_t i = 1; i < n; i++) {
        det_box_t t = v[i];
        size_t k = i;
        while (k > 0 && v[k - 1].score < t.score) {
            v[k] = v[k - 1];
            k--;
        }
        v[k] = t;
    }
}

/* input is sorted in place; equal scores may order differently from std::sort,
 * which only matters for exact score ties */
static void yolo_nms(box_vec_t *input, box_vec_t *output, unsigned char *merged, float nms_thresh)
{
    box_sort_desc(input->v, input->n);
    memset(merged, 0, input->n);
    for (size_t i = 0; i < input->n; i++) {
        if (merged[i])
            continue;
        const det_box_t *bi = &input->v[i];
        if (box_push(output, bi))
            return;
        float area_i = (bi->x2 - bi->x1) * (bi->y2 - bi->y1);

        for (size_t j = i + 1; j < input->n; j++) {
            const det_box_t *bj = &input->v[j];
            if (merged[j] || bj->classid != bi->classid)
                continue;
            float ix0 = fmaxf(bi->x1, bj->x1);
            float iy0 = fmaxf(bi->y1, bj->y1);
            float ix1 = fminf(bi->x2, bj->x2);
            float iy1 = fminf(bi->y2, bj->y2);
            float iw = fmaxf(0.0f, ix1 - ix0);
            float ih = fmaxf(0.0f, iy1 - iy0);
            float inter = iw * ih;
            float area_j = (bj->x2 - bj->x1) * (bj->y2 - bj->y1);
            if (inter / (area_i + area_j - inter) > nms_thresh)
                merged[j] = 1;
        }
    }
}

/* ================================================================
 * Context
 * ================================================================ */

typedef struct {
    ojzdl_net *net;
    int input_w, input_h;
    int frame_w, frame_h;
    float conf_thresh, nms_thresh;
    int num_classes;
    uint8_t *in_buf;  /* model input, input_w x input_h x 3 */
    ojzdl_resize *rs; /* = jzdl::resize, exact for 640x360 -> model input */
    box_vec_t candidates, detections;
    unsigned char *merged;
    size_t merged_cap;
    size_t need; /* output values yolo_decode reads */
    int warned;
} ojzdl_ctx_t;

static void ctx_free(ojzdl_ctx_t *c)
{
    if (!c)
        return;
    ojzdl_free(c->net);
    free(c->in_buf);
    ojzdl_resize_free(c->rs);
    free(c->candidates.v);
    free(c->detections.v);
    free(c->merged);
    free(c);
}

/* ================================================================
 * C API -- create, run, destroy
 * ================================================================ */

void *hal_jzdl_create(const rss_ivs_jzdl_param_t *param)
{
    const char *err = NULL;
    ojzdl_ctx_t *c;

    if (!param)
        return NULL;
    if (strnlen(param->model_path, sizeof(param->model_path)) >= sizeof(param->model_path)) {
        HAL_LOG_ERR("JZDL: model_path not null-terminated");
        return NULL;
    }
    if (param->width <= 1 || param->height <= 1 || param->width > 8192 || param->height > 8192) {
        HAL_LOG_ERR("JZDL: invalid frame dimensions %dx%d", param->width, param->height);
        return NULL;
    }
    c = calloc(1, sizeof(*c));
    if (!c)
        return NULL;
    c->net = ojzdl_load_file(param->model_path, &err);
    if (!c->net) {
        HAL_LOG_ERR("JZDL: failed to load model: %s (%s)", param->model_path, err ? err : "?");
        ctx_free(c);
        return NULL;
    }
    c->input_w = ojzdl_in_w(c->net);
    c->input_h = ojzdl_in_h(c->net);
    if (c->input_w < 2 || c->input_h < 2) {
        HAL_LOG_ERR("JZDL: model input %dx%d too small", c->input_w, c->input_h);
        ctx_free(c);
        return NULL;
    }
    HAL_LOG_INFO("JZDL: model loaded: %s (%dx%dx%d, ojzdl)", param->model_path, c->input_w,
                 c->input_h, ojzdl_in_c(c->net));

    c->frame_w = param->width;
    c->frame_h = param->height;
    c->conf_thresh = param->conf_threshold;
    c->nms_thresh = param->nms_threshold;
    c->num_classes = param->num_classes;
    c->in_buf = malloc((size_t)c->input_w * c->input_h * 3);
    c->rs = ojzdl_resize_create(c->frame_w, c->frame_h, c->input_w, c->input_h);
    if (!c->in_buf || !c->rs) {
        ctx_free(c);
        return NULL;
    }

    HAL_LOG_INFO("JZDL: ready (%dx%d -> %dx%d, %d classes, conf=%.2f, nms=%.2f)", c->frame_w,
                 c->frame_h, c->input_w, c->input_h, c->num_classes, (double)c->conf_thresh,
                 (double)c->nms_thresh);
    return c;
}

int hal_jzdl_detect(void *handle, const uint8_t *nv12_data, rss_ivs_detect_result_t *result)
{
    ojzdl_ctx_t *c = handle;
    size_t count;
    const float *out;

    if (!handle || !nv12_data || !result)
        return RSS_ERR_INVAL;

    if (ojzdl_resize_nv12(c->rs, nv12_data, c->in_buf) || ojzdl_input_rgb(c->net, c->in_buf) ||
        ojzdl_forward(c->net)) {
        result->count = 0;
        return RSS_ERR;
    }
    out = ojzdl_output(c->net, &count);
    if (!out) {
        result->count = 0;
        return RSS_OK;
    }

    /* yolo_decode reads 3 YOLOv5 heads (strides 8/16/32); a model with fewer
     * outputs (e.g. persondet, 2 heads) would be read out of bounds */
    if (count < c->need) {
        if (!c->warned)
            HAL_LOG_ERR("JZDL: model output has %zu values, the YOLOv5 decode needs %zu", count,
                        c->need);
        c->warned = 1;
        result->count = 0;
        return RSS_ERR;
    }

    c->candidates.n = 0;
    c->detections.n = 0;
    yolo_decode(out, &c->candidates, c->input_w, c->input_h, c->conf_thresh, c->num_classes);
    if (c->candidates.n > c->merged_cap) {
        unsigned char *m = realloc(c->merged, c->candidates.n);
        if (!m) {
            result->count = 0;
            return RSS_ERR;
        }
        c->merged = m;
        c->merged_cap = c->candidates.n;
    }
    yolo_nms(&c->candidates, &c->detections, c->merged, c->nms_thresh);

    float sx = (float)c->frame_w / c->input_w;
    float sy = (float)c->frame_h / c->input_h;

    result->count = (int)c->detections.n;
    if (result->count > RSS_IVS_MAX_DETECTIONS)
        result->count = RSS_IVS_MAX_DETECTIONS;
    result->timestamp = 0; /* caller sets this */

    for (int i = 0; i < result->count; i++) {
        const det_box_t *d = &c->detections.v[i];
        int x0 = (int)(d->x1 * sx);
        int y0 = (int)(d->y1 * sy);
        int x1 = (int)(d->x2 * sx);
        int y1 = (int)(d->y2 * sy);
        if (x0 < 0)
            x0 = 0;
        if (y0 < 0)
            y0 = 0;
        if (x1 >= c->frame_w)
            x1 = c->frame_w - 1;
        if (y1 >= c->frame_h)
            y1 = c->frame_h - 1;

        result->detections[i].box = (rss_rect_t){x0, y0, x1, y1};
        result->detections[i].confidence = d->score;
        result->detections[i].class_id = d->classid;
    }
    return RSS_OK;
}

void hal_jzdl_destroy(void *handle)
{
    if (!handle)
        return;
    ctx_free(handle);
    HAL_LOG_INFO("JZDL: destroyed");
}
