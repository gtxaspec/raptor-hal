/*
 * hal_ivs_yolo.c -- YOLO post-processing shared by the detection backends
 *
 * Output layout (JZDL type 76, see thingino-ai docs/jzdl-format.md): head k is
 * h x w x c floats (HWC, c = anchors x (5 + classes), anchor-major), the heads
 * one after another, then the trailer h0 w0 c0 h1 w1 c1 ... nheads. A head's
 * stride is input_w / w; its anchors come from the stride, so models with
 * persondet's 2 heads (32, 16) and YOLOv5's 3 (8, 16, 32) both decode.
 * Box decode is YOLOv5's: sigmoid everything, xy = (2s - 0.5 + cell) * stride,
 * wh = (2s)^2 * anchor, score = obj * best class. Runtimes with one tensor
 * per head feed the heads one by one (hal_yolo_decode_head).
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: MIT
 */

#include "hal_ivs_yolo.h"

#include <math.h>
#include <string.h>

/* YOLOv5 COCO anchors (yolov5s.yaml), per stride 8 / 16 / 32 */
static const float yolov5_anchors[HAL_YOLO_MAX_HEADS][HAL_YOLO_MAX_ANCHORS * 2] = {
	{10, 13, 16, 30, 33, 23},
	{30, 61, 62, 45, 59, 119},
	{116, 90, 156, 198, 373, 326},
};

static inline float sigmoid(float x)
{
	return 1.0f / (1.0f + expf(-x));
}

static int stride_index(int s)
{
	return s == 8 ? 0 : s == 16 ? 1 : s == 32 ? 2 : -1;
}

void hal_yolo_init(hal_yolo_t *y, const rss_ivs_jzdl_param_t *p, int input_w, int input_h)
{
	float lo = 1.0f;

	memset(y, 0, sizeof(*y));
	y->input_w = input_w;
	y->input_h = input_h;
	y->num_classes = p->num_classes;
	y->nms = p->nms_threshold;
	y->conf_default = p->conf_threshold;
	for (int c = 0; c < RSS_IVS_MAX_CLASSES; c++)
		y->conf[c] = p->class_threshold[c] > 0.0f ? p->class_threshold[c] : p->conf_threshold;
	for (int c = 0; c < p->num_classes; c++) {
		float t = c < RSS_IVS_MAX_CLASSES ? y->conf[c] : y->conf_default;
		if (t < lo)
			lo = t;
	}
	/* score = obj * cls <= obj, so obj < the lowest threshold can't pass */
	lo = lo < 1e-6f ? 1e-6f : lo > 0.999999f ? 0.999999f : lo;
	y->min_logit = logf(lo / (1.0f - lo));
	for (int s = 0; s < HAL_YOLO_MAX_HEADS; s++) {
		const float *a = p->anchors[s];
		int set = 0;
		for (int k = 0; k < HAL_YOLO_MAX_ANCHORS * 2; k++)
			set |= a[k] > 0.0f;
		memcpy(y->anchors[s], set ? a : yolov5_anchors[s], sizeof(y->anchors[s]));
	}
}

/* one head's grid and channels against the model: stride from the grid */
static int head_setup(const hal_yolo_t *y, hal_yolo_head_t *h, int gh, int gw, int c)
{
	int per = 5 + y->num_classes, s, si;

	if (gh < 1 || gw < 1 || c < per || c % per)
		return -1;
	s = y->input_w / gw;
	si = stride_index(s);
	if (si < 0 || s * gw != y->input_w || s * gh != y->input_h ||
	    c / per > HAL_YOLO_MAX_ANCHORS)
		return -1;
	h->gh = gh;
	h->gw = gw;
	h->na = c / per;
	h->stride = (float)s;
	h->anchors = y->anchors[si];
	return 0;
}

/* read the trailer once; the layout is fixed for a model */
static int parse_heads(hal_yolo_t *y, const float *out, size_t n)
{
	int nh;
	size_t t, sum = 0;

	if (n < 4)
		return -1;
	nh = (int)out[n - 1];
	if (nh < 1 || nh > HAL_YOLO_MAX_HEADS || n < (size_t)(3 * nh + 1))
		return -1;
	t = n - 1 - 3 * (size_t)nh;
	for (int k = 0; k < nh; k++) {
		int gh = (int)out[t + 3 * k], gw = (int)out[t + 3 * k + 1], c = (int)out[t + 3 * k + 2];
		if (head_setup(y, &y->head[k], gh, gw, c))
			return -1;
		sum += (size_t)gh * gw * c;
	}
	if (sum != t)
		return -1;
	y->nheads = nh;
	y->out_len = n;
	return 0;
}

/* keep the best HAL_YOLO_MAX_CAND: when full, a better box replaces the worst */
static void cand_add(hal_yolo_t *y, const hal_yolo_box_t *b)
{
	int w = 0;

	if (y->ncand < HAL_YOLO_MAX_CAND) {
		y->cand[y->ncand++] = *b;
		return;
	}
	for (int i = 1; i < HAL_YOLO_MAX_CAND; i++)
		if (y->cand[i].score < y->cand[w].score)
			w = i;
	if (b->score > y->cand[w].score)
		y->cand[w] = *b;
}

/* one head's cells into the candidates; returns the end of its values */
static const float *decode_head(hal_yolo_t *y, const hal_yolo_head_t *h, const float *p)
{
	int per = 5 + y->num_classes;

	for (int gy = 0; gy < h->gh; gy++) {
		for (int gx = 0; gx < h->gw; gx++) {
			for (int a = 0; a < h->na; a++, p += per) {
				hal_yolo_box_t b;
				float obj, t, sw, sh;
				int best = 0;

				if (p[4] < y->min_logit)
					continue;
				for (int c = 1; c < y->num_classes; c++)
					if (p[5 + c] > p[5 + best])
						best = c;
				obj = sigmoid(p[4]);
				b.score = obj * sigmoid(p[5 + best]);
				t = best < RSS_IVS_MAX_CLASSES ? y->conf[best] : y->conf_default;
				if (b.score < t)
					continue;
				sw = sigmoid(p[2]) * 2.0f;
				sh = sigmoid(p[3]) * 2.0f;
				sw *= sw * h->anchors[2 * a];
				sh *= sh * h->anchors[2 * a + 1];
				float cx = (sigmoid(p[0]) * 2.0f - 0.5f + gx) * h->stride;
				float cy = (sigmoid(p[1]) * 2.0f - 0.5f + gy) * h->stride;
				b.x1 = cx - sw * 0.5f;
				b.y1 = cy - sh * 0.5f;
				b.x2 = cx + sw * 0.5f;
				b.y2 = cy + sh * 0.5f;
				b.cls = best;
				cand_add(y, &b);
			}
		}
	}
	return p;
}

static void decode(hal_yolo_t *y, const float *p)
{
	y->ncand = 0;
	for (int k = 0; k < y->nheads; k++)
		p = decode_head(y, &y->head[k], p);
}

/* in place: sort by score (insertion sort, n <= HAL_YOLO_MAX_CAND), then
 * per-class greedy NMS; returns the number kept at the front */
static int nms(hal_yolo_box_t *v, int n, float thresh)
{
	int kept = 0;

	for (int i = 1; i < n; i++) {
		hal_yolo_box_t b = v[i];
		int j = i;
		while (j > 0 && v[j - 1].score < b.score) {
			v[j] = v[j - 1];
			j--;
		}
		v[j] = b;
	}
	for (int i = 0; i < n; i++) {
		const hal_yolo_box_t *b = &v[i];
		float area = (b->x2 - b->x1) * (b->y2 - b->y1);
		int drop = 0;
		for (int k = 0; k < kept && !drop; k++) {
			const hal_yolo_box_t *o = &v[k];
			if (o->cls != b->cls)
				continue;
			float iw = fminf(b->x2, o->x2) - fmaxf(b->x1, o->x1);
			float ih = fminf(b->y2, o->y2) - fmaxf(b->y1, o->y1);
			if (iw <= 0.0f || ih <= 0.0f)
				continue;
			float inter = iw * ih;
			float uni = area + (o->x2 - o->x1) * (o->y2 - o->y1) - inter;
			drop = inter > thresh * uni;
		}
		if (!drop)
			v[kept++] = *b;
	}
	return kept;
}

void hal_yolo_begin(hal_yolo_t *y)
{
	y->ncand = 0;
}

int hal_yolo_decode_head(hal_yolo_t *y, const float *p, int gh, int gw, int c)
{
	hal_yolo_head_t h;

	if (head_setup(y, &h, gh, gw, c))
		return RSS_ERR;
	decode_head(y, &h, p);
	return RSS_OK;
}

int hal_yolo_finish(hal_yolo_t *y)
{
	return nms(y->cand, y->ncand, y->nms);
}

void hal_yolo_emit(const hal_yolo_t *y, int n, float scale_x, float scale_y, float off_x,
		   float off_y, int frame_w, int frame_h, rss_ivs_detect_result_t *result)
{
	if (n > RSS_IVS_MAX_DETECTIONS)
		n = RSS_IVS_MAX_DETECTIONS;
	for (int i = 0; i < n; i++) {
		const hal_yolo_box_t *b = &y->cand[i];
		int x0 = (int)((b->x1 - off_x) * scale_x), y0 = (int)((b->y1 - off_y) * scale_y);
		int x1 = (int)((b->x2 - off_x) * scale_x), y1 = (int)((b->y2 - off_y) * scale_y);
		if (x0 < 0)
			x0 = 0;
		if (y0 < 0)
			y0 = 0;
		if (x1 >= frame_w)
			x1 = frame_w - 1;
		if (y1 >= frame_h)
			y1 = frame_h - 1;
		result->detections[i].box = (rss_rect_t){x0, y0, x1, y1};
		result->detections[i].confidence = b->score;
		result->detections[i].class_id = b->cls;
	}
	result->count = n;
}

int hal_yolo_run(hal_yolo_t *y, const float *out, size_t n, int frame_w, int frame_h,
		 rss_ivs_detect_result_t *result)
{
	result->count = 0;
	result->timestamp = 0; /* caller sets this */
	if (y->nheads == 0 || n != y->out_len) {
		if (y->nheads >= 0 && parse_heads(y, out, n)) {
			HAL_LOG_ERR("JZDL: model output (%zu values) is not %d-class YOLO heads at "
				    "strides 8/16/32 for a %dx%d input",
				    n, y->num_classes, y->input_w, y->input_h);
			y->nheads = -1; /* log once */
		}
		if (y->nheads <= 0)
			return RSS_ERR;
		HAL_LOG_INFO("JZDL: %d head(s), %zu outputs", y->nheads, n);
	}
	decode(y, out);
	hal_yolo_emit(y, hal_yolo_finish(y), (float)frame_w / y->input_w,
		      (float)frame_h / y->input_h, 0.0f, 0.0f, frame_w, frame_h, result);
	return RSS_OK;
}
