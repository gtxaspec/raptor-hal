/*
 * hal_ivs_yolo.h -- YOLO post-processing shared by the JZDL backends
 *
 * Decodes a JZDL model's packed output (layer type 76: every head flattened
 * HWC, then h, w, c per head and the head count), whatever the number and
 * order of heads, applies per-class thresholds and per-class NMS, and fills
 * an rss_ivs_detect_result_t in frame coordinates.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: MIT
 */

#ifndef HAL_IVS_YOLO_H
#define HAL_IVS_YOLO_H

#include "hal_internal.h"

#include <stddef.h>

#define HAL_YOLO_MAX_HEADS   3   /* strides 8, 16, 32 */
#define HAL_YOLO_MAX_ANCHORS 3   /* per head */
#define HAL_YOLO_MAX_CAND    256 /* candidates kept before NMS (the best ones) */

typedef struct {
	float x1, y1, x2, y2; /* model input pixels */
	float score;
	int cls;
} hal_yolo_box_t;

typedef struct {
	int gh, gw, na;        /* grid and anchors per cell */
	float stride;
	const float *anchors;  /* na x (w, h), model input pixels */
} hal_yolo_head_t;

typedef struct {
	int input_w, input_h, num_classes;
	float conf[RSS_IVS_MAX_CLASSES]; /* threshold per class (num_classes used) */
	float conf_default;              /* classes past RSS_IVS_MAX_CLASSES */
	float min_logit;                 /* objectness logit below which a cell is skipped */
	float nms;
	float anchors[HAL_YOLO_MAX_HEADS][HAL_YOLO_MAX_ANCHORS * 2]; /* strides 8, 16, 32 */
	int nheads;                      /* 0 until the first output has been parsed */
	hal_yolo_head_t head[HAL_YOLO_MAX_HEADS];
	size_t out_len;                  /* output values incl. the trailer */
	int ncand;
	hal_yolo_box_t cand[HAL_YOLO_MAX_CAND];
} hal_yolo_t;

/* thresholds, anchors (param's, or YOLOv5's for strides left at 0) */
void hal_yolo_init(hal_yolo_t *y, const rss_ivs_jzdl_param_t *param, int input_w, int input_h);

/* decode + NMS + scale to the frame. RSS_ERR if the output doesn't fit the
 * configured model (logged once). */
int hal_yolo_run(hal_yolo_t *y, const float *out, size_t n, int frame_w, int frame_h,
		 rss_ivs_detect_result_t *result);

#endif /* HAL_IVS_YOLO_H */
