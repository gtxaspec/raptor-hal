/*
 * hal_ivs_yolo.h -- YOLO post-processing shared by the detection backends
 *
 * Decodes YOLOv5-style heads (HWC, c = anchors x (5 + classes)), applies
 * per-class thresholds and per-class NMS.
 *
 * Two ways in:
 * - hal_yolo_run(): a JZDL model's packed output (layer type 76: every head
 *   flattened HWC, then h, w, c per head and the head count), whatever the
 *   number and order of heads; fills an rss_ivs_detect_result_t in frame
 *   coordinates, the frame being the model input stretched.
 * - hal_yolo_begin(), hal_yolo_decode_head() per head, hal_yolo_finish():
 *   for runtimes with one output tensor per head. The boxes stay in model
 *   input pixels; hal_yolo_emit() maps them to the frame with any scale and
 *   offset (a letterbox, for instance).
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: MIT
 */

#ifndef HAL_IVS_YOLO_H
#define HAL_IVS_YOLO_H

#include "hal_internal.h"

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HAL_YOLO_MAX_HEADS 3     /* strides 8, 16, 32 */
#define HAL_YOLO_MAX_ANCHORS 3   /* per head */
#define HAL_YOLO_MAX_CAND 256    /* candidates kept before NMS (the best ones) */
#define HAL_YOLO_MAX_CLASSES 255 /* num_classes accepted from config */
#define HAL_YOLO_MAX_ANCHOR 4096 /* anchor w/h, model input pixels */

typedef struct {
    float x1, y1, x2, y2; /* model input pixels */
    float score;
    int cls;
} hal_yolo_box_t;

typedef struct {
    int gh, gw, na; /* grid and anchors per cell */
    float stride;
    const float *anchors; /* na x (w, h), model input pixels */
} hal_yolo_head_t;

typedef struct {
    int input_w, input_h, num_classes;
    float conf[RSS_IVS_MAX_CLASSES]; /* threshold per class (num_classes used) */
    float conf_default;              /* classes past RSS_IVS_MAX_CLASSES */
    float min_logit;                 /* objectness logit below which a cell is skipped */
    float nms;
    float anchors[HAL_YOLO_MAX_HEADS][HAL_YOLO_MAX_ANCHORS * 2]; /* strides 8, 16, 32 */
    int nheads; /* 0 until the first output has been parsed */
    hal_yolo_head_t head[HAL_YOLO_MAX_HEADS];
    size_t out_len; /* output values incl. the trailer */
    int ncand;
    hal_yolo_box_t cand[HAL_YOLO_MAX_CAND];
} hal_yolo_t;

/* The model's preprocess (param->preprocess) against the one the backend
 * does. false, logged, if they differ or the value is unknown: the backend
 * then refuses the model at load instead of feeding it the wrong input. */
bool hal_yolo_preprocess_ok(const rss_ivs_jzdl_param_t *param, rss_ivs_preprocess_t backend);

/* thresholds, anchors (param's, or YOLOv5's for strides left at 0).
 * RSS_ERR, logged, if num_classes is outside 1..HAL_YOLO_MAX_CLASSES, the
 * input size is not positive or an anchor is outside 0..HAL_YOLO_MAX_ANCHOR. */
int hal_yolo_init(hal_yolo_t *y, const rss_ivs_jzdl_param_t *param, int input_w, int input_h);

/* JZDL packed output: decode + NMS + scale to the frame. RSS_ERR if the
 * output doesn't fit the configured model (logged once). */
int hal_yolo_run(hal_yolo_t *y, const float *out, size_t n, int frame_w, int frame_h,
                 rss_ivs_detect_result_t *result);

/* Per-head use. begin() clears the candidates. decode_head() takes one head,
 * gh x gw x c floats HWC (c = anchors x (5 + num_classes), anchor-major,
 * values before the sigmoid); the stride is input_w / gw and must be 8, 16
 * or 32. RSS_ERR (nothing decoded) if the head doesn't fit the model.
 * finish() runs NMS and returns the number of boxes kept, best first, in
 * y->cand[] in model input pixels. */
void hal_yolo_begin(hal_yolo_t *y);
int hal_yolo_decode_head(hal_yolo_t *y, const float *p, int gh, int gw, int c);
int hal_yolo_finish(hal_yolo_t *y);

/* The first n boxes of y->cand[] into result (at most RSS_IVS_MAX_DETECTIONS),
 * frame = (model - off) * scale, clamped to frame_w x frame_h. */
void hal_yolo_emit(const hal_yolo_t *y, int n, float scale_x, float scale_y, float off_x,
                   float off_y, int frame_w, int frame_h, rss_ivs_detect_result_t *result);

#ifdef __cplusplus
}
#endif

#endif /* HAL_IVS_YOLO_H */
