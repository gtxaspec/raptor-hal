/*
 * hal_ivs_ojzdl.c -- Standalone JZDL inference on ojzdl, the open runtime
 *
 * Same C API and results as hal_ivs_jzdl.cpp (hal_jzdl_create / _detect /
 * _destroy), without Ingenic's libjzdl.m.so or libstdc++:
 *   - ojzdl runs JZDL v4/v5 models with output bit-identical to libjzdl
 *     (src/ojzdl/, MXU2 kernels on the T31);
 *   - ojzdl_resize reproduces jzdl::resize exactly for the sizes raptor uses
 *     (640x360 -> 416x320 / 320x416), from NV12 rows converted on demand;
 *   - NV12 -> RGB is the same code as the C++ backend; decode + NMS are shared
 *     (hal_ivs_yolo.c).
 * Build with OJZDL=1 instead of JZDL_INCLUDE.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: MIT
 */

#include "hal_internal.h"
#include "hal_ivs_yolo.h"
#include "ojzdl/ojzdl.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

/* ================================================================
 * NV12 -> RGB, one row at a time (same formula as hal_ivs_jzdl.cpp's
 * nv12_to_rgb), fed to ojzdl_resize: no full-frame RGB buffer
 * ================================================================ */

static inline uint8_t clamp_u8(int v)
{
	return v < 0 ? 0 : (v > 255 ? 255 : (uint8_t)v);
}

static void nv12_row_to_rgb(const uint8_t *nv12, uint8_t *rgb, int w, int h, int row)
{
	const uint8_t *y_plane = nv12 + (size_t)row * w;
	const uint8_t *uv_plane = nv12 + (size_t)w * h + (size_t)(row / 2) * w;

	for (int col = 0; col < w; col++) {
		int y = y_plane[col];
		int u = uv_plane[col & ~1] - 128;
		int v = uv_plane[col | 1] - 128;
		rgb[col * 3 + 0] = clamp_u8(y + ((359 * v) >> 8));
		rgb[col * 3 + 1] = clamp_u8(y - ((88 * u + 183 * v) >> 8));
		rgb[col * 3 + 2] = clamp_u8(y + ((454 * u) >> 8));
	}
}

/* ================================================================
 * Context
 * ================================================================ */

typedef struct {
	ojzdl_net *net;
	int input_w, input_h;
	int frame_w, frame_h;
	int num_classes;
	uint8_t *rgb_row;          /* one frame row, frame_w x 3 */
	uint8_t *in_buf;           /* model input, input_w x input_h x 3 */
	ojzdl_resize *rs;          /* = jzdl::resize, exact for 640x360 -> model input */
	const uint8_t *nv12;       /* frame being converted */
	hal_yolo_t yolo;
} ojzdl_ctx_t;

static const uint8_t *nv12_get_row(void *ctx, int y)
{
	ojzdl_ctx_t *c = ctx;
	nv12_row_to_rgb(c->nv12, c->rgb_row, c->frame_w, c->frame_h, y);
	return c->rgb_row;
}

static void ctx_free(ojzdl_ctx_t *c)
{
	if (!c)
		return;
	ojzdl_free(c->net);
	free(c->rgb_row);
	free(c->in_buf);
	ojzdl_resize_free(c->rs);
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
	c->num_classes = param->num_classes;
	hal_yolo_init(&c->yolo, param, c->input_w, c->input_h);
	c->rgb_row = malloc((size_t)c->frame_w * 3);
	c->in_buf = malloc((size_t)c->input_w * c->input_h * 3);
	c->rs = ojzdl_resize_create(c->frame_w, c->frame_h, c->input_w, c->input_h);
	if (!c->rgb_row || !c->in_buf || !c->rs) {
		ctx_free(c);
		return NULL;
	}

	HAL_LOG_INFO("JZDL: ready (%dx%d -> %dx%d, %d classes, conf=%.2f, nms=%.2f)", c->frame_w,
		     c->frame_h, c->input_w, c->input_h, c->num_classes, (double)param->conf_threshold,
		     (double)param->nms_threshold);
	return c;
}

int hal_jzdl_detect(void *handle, const uint8_t *nv12_data, rss_ivs_detect_result_t *result)
{
	ojzdl_ctx_t *c = handle;
	size_t count;
	const float *out;

	if (!handle || !nv12_data || !result)
		return RSS_ERR_INVAL;

	c->nv12 = nv12_data;
	if (ojzdl_resize_rows(c->rs, nv12_get_row, c, c->in_buf) || ojzdl_input_rgb(c->net, c->in_buf) || ojzdl_forward(c->net)) {
		result->count = 0;
		return RSS_ERR;
	}
	out = ojzdl_output(c->net, &count);
	if (!out) {
		result->count = 0;
		return RSS_OK;
	}

	return hal_yolo_run(&c->yolo, out, count, c->frame_w, c->frame_h, result);
}

void hal_jzdl_destroy(void *handle)
{
	if (!handle)
		return;
	ctx_free(handle);
	HAL_LOG_INFO("JZDL: destroyed");
}
