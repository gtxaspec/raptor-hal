/*
 * hal_ivs_jzdl.cpp -- Standalone JZDL inference API
 *
 * Provides C-callable functions to load a JZDL model, run inference
 * on NV12 frames, and return detection results. Used by RVD's JZDL
 * inference thread — reads frames directly from FrameSource, not
 * through the IVS pipeline (the IVS interface approach was abandoned
 * as vendor IVS internals are incompatible with external models).
 *
 * Requires: libjzdl.m.so (standalone JZDL with BaseNet API)
 */

#ifndef JZ_MXU
#define JZ_MXU 0
#endif

extern "C" {
#include "hal_internal.h"
}

#include "net.h"
#include "utils.h"

#include "hal_ivs_yolo.h"

#include <cstring>
#include <vector>

/* ================================================================
 * NV12 → RGB conversion
 * ================================================================ */

static inline uint8_t clamp_u8(int v)
{
    return v < 0 ? 0 : (v > 255 ? 255 : (uint8_t)v);
}

static void nv12_to_rgb(const uint8_t *nv12, uint8_t *rgb, int w, int h)
{
    const uint8_t *y_plane = nv12;
    const uint8_t *uv_plane = nv12 + w * h;

    for (int row = 0; row < h; row++) {
        for (int col = 0; col < w; col++) {
            int y = y_plane[row * w + col];
            int u = uv_plane[(row / 2) * w + (col & ~1)] - 128;
            int v = uv_plane[(row / 2) * w + (col | 1)] - 128;
            int idx = (row * w + col) * 3;
            rgb[idx + 0] = clamp_u8(y + ((359 * v) >> 8));
            rgb[idx + 1] = clamp_u8(y - ((88 * u + 183 * v) >> 8));
            rgb[idx + 2] = clamp_u8(y + ((454 * u) >> 8));
        }
    }
}

/* ================================================================
 * JZDL context (opaque handle)
 * ================================================================ */

struct jzdl_ctx {
    jzdl::BaseNet *net;
    int input_w, input_h, input_c;
    int frame_w, frame_h;
    float conf_thresh;
    float nms_thresh;
    int num_classes;
    uint8_t *rgb_buf;
    std::vector<DetBox> candidates;
    std::vector<DetBox> detections;
};

/* ================================================================
 * C API — create, run, destroy
 * ================================================================ */

extern "C" void *hal_jzdl_create(const rss_ivs_jzdl_param_t *param)
{
    if (!param)
        return NULL;

    if (strnlen(param->model_path, sizeof(param->model_path)) >= sizeof(param->model_path)) {
        HAL_LOG_ERR("JZDL: model_path not null-terminated");
        return NULL;
    }

    jzdl::BaseNet *net = jzdl::net_create();
    if (!net) {
        HAL_LOG_ERR("JZDL: net_create failed");
        return NULL;
    }

    if (net->load_model(param->model_path) != 0) {
        HAL_LOG_ERR("JZDL: failed to load model: %s", param->model_path);
        jzdl::net_destory(net); /* sic — vendor typo in JZDL API */
        return NULL;
    }

    std::vector<uint32_t> shape = net->get_input_shape();
    if (shape.size() < 3) {
        HAL_LOG_ERR("JZDL: model input shape has %zu dims, expected 3", shape.size());
        jzdl::net_destory(net);
        return NULL;
    }
    HAL_LOG_INFO("JZDL: model loaded: %s (%ux%ux%u)", param->model_path, shape[0], shape[1],
                 shape[2]);

    jzdl_ctx *ctx = new (std::nothrow) jzdl_ctx();
    if (!ctx) {
        jzdl::net_destory(net);
        return NULL;
    }

    if (param->width <= 0 || param->height <= 0 || param->width > 8192 || param->height > 8192) {
        HAL_LOG_ERR("JZDL: invalid frame dimensions %dx%d", param->width, param->height);
        jzdl::net_destory(net);
        delete ctx;
        return NULL;
    }

    ctx->net = net;
    ctx->input_w = (int)shape[0];
    ctx->input_h = (int)shape[1];
    ctx->input_c = (int)shape[2];
    ctx->frame_w = param->width;
    ctx->frame_h = param->height;
    ctx->conf_thresh = param->conf_threshold;
    ctx->nms_thresh = param->nms_threshold;
    ctx->num_classes = param->num_classes;
    ctx->rgb_buf = new (std::nothrow) uint8_t[(size_t)param->width * param->height * 3];
    if (!ctx->rgb_buf) {
        jzdl::net_destory(net);
        delete ctx;
        return NULL;
    }

    HAL_LOG_INFO("JZDL: ready (%dx%d -> %dx%d, %d classes, conf=%.2f, nms=%.2f)", param->width,
                 param->height, ctx->input_w, ctx->input_h, param->num_classes,
                 (double)param->conf_threshold, (double)param->nms_threshold);

    return ctx;
}

extern "C" int hal_jzdl_detect(void *handle, const uint8_t *nv12_data,
                               rss_ivs_detect_result_t *result)
{
    if (!handle || !nv12_data || !result)
        return RSS_ERR_INVAL;

    jzdl_ctx *ctx = (jzdl_ctx *)handle;

    /* NV12 → RGB */
    nv12_to_rgb(nv12_data, ctx->rgb_buf, ctx->frame_w, ctx->frame_h);

    /* Resize + normalize */
    jzdl::Mat<uint8_t> src(ctx->frame_w, ctx->frame_h, 3, ctx->rgb_buf);
    jzdl::Mat<uint8_t> dst(ctx->input_w, ctx->input_h, ctx->input_c);
    jzdl::resize(src, dst);
    jzdl::image_sub(dst, 128);

    /* Inference */
    jzdl::Mat<int8_t> input(ctx->input_w, ctx->input_h, ctx->input_c, (int8_t *)dst.data);
    jzdl::Mat<float> output;
    ctx->net->input(input);
    ctx->net->run(output);

    if (!output.data) {
        result->count = 0;
        return RSS_OK;
    }

    /* Post-process (reuse per-context vectors to avoid alloc per frame) */
    ctx->candidates.clear();
    ctx->detections.clear();
    /* One buffer, the three head levels (strides 8, 16, 32) in a row. */
    const float *p = output.data;
    for (int level = 0; level < 3; level++) {
        int gw = ctx->input_w >> (3 + level);
        int gh = ctx->input_h >> (3 + level);
        yolo_decode_level(p, level, gw, gh, ctx->input_w, ctx->input_h, ctx->conf_thresh,
                          ctx->num_classes, ctx->candidates);
        p += gw * gh * 3 * (5 + ctx->num_classes);
    }
    yolo_nms(ctx->candidates, ctx->detections, ctx->nms_thresh);

    /* Scale to frame coordinates and fill result */
    float sx = (float)ctx->frame_w / ctx->input_w;
    float sy = (float)ctx->frame_h / ctx->input_h;

    result->count = (int)ctx->detections.size();
    if (result->count > RSS_IVS_MAX_DETECTIONS)
        result->count = RSS_IVS_MAX_DETECTIONS;
    result->timestamp = 0; /* caller sets this */

    for (int i = 0; i < result->count; i++) {
        DetBox &d = ctx->detections[i];
        int x0 = (int)(d.x1 * sx);
        int y0 = (int)(d.y1 * sy);
        int x1 = (int)(d.x2 * sx);
        int y1 = (int)(d.y2 * sy);
        if (x0 < 0)
            x0 = 0;
        if (y0 < 0)
            y0 = 0;
        if (x1 >= ctx->frame_w)
            x1 = ctx->frame_w - 1;
        if (y1 >= ctx->frame_h)
            y1 = ctx->frame_h - 1;

        result->detections[i].box = (rss_rect_t){x0, y0, x1, y1};
        result->detections[i].confidence = d.score;
        result->detections[i].class_id = d.classid;
    }

    return RSS_OK;
}

extern "C" void hal_jzdl_destroy(void *handle)
{
    if (!handle)
        return;

    jzdl_ctx *ctx = (jzdl_ctx *)handle;
    if (ctx->net)
        jzdl::net_destory(ctx->net);
    delete[] ctx->rgb_buf;
    delete ctx;

    HAL_LOG_INFO("JZDL: destroyed");
}
