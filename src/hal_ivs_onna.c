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
 *   - every output is a YOLO head (NHWC, anchors x (5 + classes)), decoded by
 *     hal_ivs_yolo.c (per-class thresholds, per-class NMS); the stride comes from its grid,
 *     the anchors from the caller (param->anchors: rvd [motion] anchors_<stride>, e.g. from
 *     the model's model.conf), else the graph, else YOLOv5's defaults.
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
#include "hal_ivs_yolo.h"
#include "ojzdl/ojzdl.h"
#include "onna/onnaq.h"

#include <sched.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

typedef struct {
    int input_w, input_h, frame_w, frame_h;
    int nheads;
    int bad; /* the outputs don't fit the configured model: logged once, detect refuses */
    hal_yolo_t yolo;
    uint8_t *in_buf;
    ojzdl_resize *rs;
    int pinned;
    /* mean times, logged every 100 inferences */
    double t_pre, t_net, t_post;
    int nt;
} onna_ctx_t;

static int in_use;

static void ctx_free(onna_ctx_t *c)
{
    if (!c)
        return;
    free(c->in_buf);
    ojzdl_resize_free(c->rs);
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
    if (!hal_yolo_preprocess_ok(param, RSS_IVS_PREPROCESS_STRETCH))
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
        HAL_LOG_ERR("ONNA: failed to load %s + %s: %s", param->model_path,
                    gpath[0] ? gpath : "(graph from the .bin)", onnaq_error());
        free(c);
        return NULL;
    }
    in_use = 1;
    onnaq_input_size(&c->input_w, &c->input_h);
    c->nheads = onnaq_num_heads();
    if (c->nheads < 1 || c->nheads > HAL_YOLO_MAX_HEADS)
        goto bad_heads;
    /* thresholds (per class), NMS and the caller's anchors ([motion] anchors_<stride>, e.g.
     * from the model's model.conf); the head shapes are checked on the first run */
    if (hal_yolo_init(&c->yolo, param, c->input_w, c->input_h) != RSS_OK)
        goto fail;
    /* strides the caller left without anchors: the graph's anchors line, else YOLOv5's
     * (onnaq_anchors gives those too; hal_yolo_init already did) */
    for (int si = 0; si < HAL_YOLO_MAX_HEADS; si++) {
        int set = 0;
        for (int j = 0; j < HAL_YOLO_MAX_ANCHORS * 2; j++)
            set |= param->anchors[si][j] > 0.0f;
        if (!set)
            onnaq_anchors(8 << si, c->yolo.anchors[si]);
    }
    c->frame_w = param->width;
    c->frame_h = param->height;
    c->in_buf = malloc((size_t)c->input_w * c->input_h * 3);
    c->rs = ojzdl_resize_create(c->frame_w, c->frame_h, c->input_w, c->input_h);
    if (!c->in_buf || !c->rs)
        goto fail;
    HAL_LOG_INFO("ONNA: ready: %s (%dx%d -> %dx%d, %d heads, %d classes, conf=%.2f, nms=%.2f)",
                 param->model_path, c->frame_w, c->frame_h, c->input_w, c->input_h, c->nheads,
                 param->num_classes, (double)param->conf_threshold, (double)param->nms_threshold);
    return c;

fail:
    onnaq_close();
    in_use = 0;
    ctx_free(c);
    return NULL;

bad_heads:
    HAL_LOG_ERR("ONNA: %s: %d outputs (1..%d supported)", param->model_path, c->nheads,
                HAL_YOLO_MAX_HEADS);
    onnaq_close();
    in_use = 0;
    free(c);
    return NULL;
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
    if (ojzdl_resize_nv12(c->rs, nv12_data, c->in_buf))
        return RSS_ERR;
    double t1 = now_ms();
    if (onnaq_run(c->in_buf)) {
        HAL_LOG_ERR("ONNA: run failed: %s", onnaq_error());
        return RSS_ERR;
    }
    double t2 = now_ms();
    if (c->bad)
        return RSS_ERR;

    hal_yolo_begin(&c->yolo);
    for (int k = 0; k < c->nheads; k++) {
        int h = 0, w = 0, ch = 0;
        const float *p = onnaq_head(k, &h, &w, &ch);
        if (!p || hal_yolo_decode_head(&c->yolo, p, h, w, ch) != RSS_OK) {
            HAL_LOG_ERR("ONNA: output %d (%dx%dx%d) is not a YOLO head with %d classes at stride "
                        "8/16/32 for a %dx%d input",
                        k, h, w, ch, c->yolo.num_classes, c->input_w, c->input_h);
            c->bad = 1;
            return RSS_ERR;
        }
    }
    hal_yolo_emit(&c->yolo, hal_yolo_finish(&c->yolo), (float)c->frame_w / c->input_w,
                  (float)c->frame_h / c->input_h, 0.0f, 0.0f, c->frame_w, c->frame_h, result);
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
