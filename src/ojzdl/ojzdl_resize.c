/* ojzdl_resize: bilinear RGB resize identical to libjzdl 5.4.0's
 * jzdl::resize (INTER_LINEAR), for model preprocessing.
 *
 * Found by experiment against libjzdl under QEMU (../../jzdl-runtime.md,
 * "Preprocessing"). Exact from 640x360 to 416x320, 320x416, 416x234,
 * 448x256, 384x224, 320x192 and same-size, and for 700x400 and 999x361 ->
 * 416x320. 1280x720 -> 416x320 / 448x256 and 213x120 -> 416x320 are within 1
 * on < 0.1 % of bytes.
 *   - 11-bit coefficients (2048 = 1.0);
 *   - horizontal: source x in float ((d + 0.5f) * (float)src/dst - 0.5f, each
 *     step rounded to float) and both coefficients rounded half to even
 *     (verified 2026-11-25: at 448 wide, double with half up is off by one in
 *     every 7th column past x = 360), edge coefficient 2048;
 *   - vertical: fraction in float, b0 rounded, b1 = 2048 - b0, edge 2047;
 *   - rows: (p0 * a0 + p1 * a1) >> 4 as int16; out = ((b0*r0) >> 16) +
 *     ((b1*r1) >> 16), then (+2) >> 2.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: MIT
 */
#include "ojzdl.h"
#include "ojzdl_mxu2.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define RS_ONE 2048 /* coefficient scale, 11 bits */

struct ojzdl_resize {
	int sw, sh, dw, dh;
	int *xofs, *yofs;
	int16_t *alpha, *beta;
	int16_t *rows[2];      /* horizontally resized source rows */
	int have[2];           /* source row index held in rows[k], or -1 */
};

static void tab_x(int src, int dst, int *ofs, int16_t *a)
{
	float scale = (float)src / (float)dst;
	int d;
	for (d = 0; d < dst; d++) {
		/* separate statements: no fused multiply-add, every step rounds to float */
		float t = (float)d + 0.5f;
		float m = t * scale;
		float f = m - 0.5f;
		int s = (int)floorf(f);
		float fr = f - (float)s;
		float c0 = (1.0f - fr) * (float)RS_ONE, c1 = fr * (float)RS_ONE;
		int a0 = (int)lrintf(c0), a1 = (int)lrintf(c1);
		if (s < 0) {
			s = 0;
			a0 = RS_ONE;
			a1 = 0;
		}
		if (s >= src - 1) {
			s = src - 2;
			a0 = 0;
			a1 = RS_ONE;
		}
		ofs[d] = s;
		a[2 * d] = (int16_t)a0;
		a[2 * d + 1] = (int16_t)a1;
	}
}

static void tab_y(int src, int dst, int *ofs, int16_t *b)
{
	double scale = (double)src / dst;
	int d;
	for (d = 0; d < dst; d++) {
		float f = (float)((d + 0.5) * scale - 0.5);
		int s = (int)floorf(f);
		float fr = f - (float)s;
		int b0 = (int)floor((1.0 - (double)fr) * RS_ONE + 0.5), b1 = RS_ONE - b0;
		if (s < 0) {
			s = 0;
			b0 = RS_ONE - 1;
			b1 = 0;
		}
		if (s >= src - 1) {
			s = src - 2;
			b0 = 0;
			b1 = RS_ONE - 1;
		}
		ofs[d] = s;
		b[2 * d] = (int16_t)b0;
		b[2 * d + 1] = (int16_t)b1;
	}
}

ojzdl_resize *ojzdl_resize_create(int src_w, int src_h, int dst_w, int dst_h)
{
	ojzdl_resize *r;
	if (src_w < 2 || src_h < 2 || dst_w < 1 || dst_h < 1 || src_w > 8192 || src_h > 8192 ||
	    dst_w > 8192 || dst_h > 8192)
		return NULL;
	r = calloc(1, sizeof(*r));
	if (!r)
		return NULL;
	r->sw = src_w; r->sh = src_h; r->dw = dst_w; r->dh = dst_h;
	r->xofs = malloc((size_t)dst_w * sizeof(int));
	r->yofs = malloc((size_t)dst_h * sizeof(int));
	r->alpha = malloc((size_t)dst_w * 2 * sizeof(int16_t));
	r->beta = malloc((size_t)dst_h * 2 * sizeof(int16_t));
	/* +16 bytes: the vector pass reads whole 16-byte blocks */
	r->rows[0] = malloc((size_t)dst_w * 3 * sizeof(int16_t) + 32);
	r->rows[1] = malloc((size_t)dst_w * 3 * sizeof(int16_t) + 32);
	if (!r->xofs || !r->yofs || !r->alpha || !r->beta || !r->rows[0] || !r->rows[1]) {
		ojzdl_resize_free(r);
		return NULL;
	}
	tab_x(src_w, dst_w, r->xofs, r->alpha);
	tab_y(src_h, dst_h, r->yofs, r->beta);
	return r;
}

void ojzdl_resize_free(ojzdl_resize *r)
{
	if (!r)
		return;
	free(r->xofs);
	free(r->yofs);
	free(r->alpha);
	free(r->beta);
	free(r->rows[0]);
	free(r->rows[1]);
	free(r);
}

static void hrow(const ojzdl_resize *r, const uint8_t *src, int16_t *dst)
{
	int dx, k;
	for (dx = 0; dx < r->dw; dx++) {
		const uint8_t *p = src + r->xofs[dx] * 3;
		int a0 = r->alpha[2 * dx], a1 = r->alpha[2 * dx + 1];
		for (k = 0; k < 3; k++)
			dst[dx * 3 + k] = (int16_t)((p[k] * a0 + p[3 + k] * a1) >> 4);
	}
}

/* rows[0] = source row y, rows[1] = row y + 1; reuse what is already held */
static int load_pair(ojzdl_resize *r, int y, const uint8_t *(*get_row)(void *, int), void *ctx)
{
	int k;
	if (r->have[0] == y && r->have[1] == y + 1)
		return 0;
	if (r->have[1] == y) { /* advance by one: the old lower row becomes the upper */
		int16_t *t = r->rows[0];
		r->rows[0] = r->rows[1];
		r->rows[1] = t;
		r->have[0] = y;
		r->have[1] = -1;
	}
	for (k = 0; k < 2; k++)
		if (r->have[k] != y + k) {
			const uint8_t *row = get_row(ctx, y + k);
			if (!row)
				return -1;
			hrow(r, row, r->rows[k]);
			r->have[k] = y + k;
		}
	return 0;
}

int ojzdl_resize_rows(ojzdl_resize *r, const uint8_t *(*get_row)(void *ctx, int y), void *ctx,
                      uint8_t *dst)
{
	size_t n = (size_t)r->dw * 3, i;
	int dy;
	r->have[0] = r->have[1] = -1;
	for (dy = 0; dy < r->dh; dy++) {
		int b0 = r->beta[2 * dy], b1 = r->beta[2 * dy + 1];
		const int16_t *r0, *r1;
		uint8_t *d = dst + (size_t)dy * n;
		if (load_pair(r, r->yofs[dy], get_row, ctx))
			return -1;
		r0 = r->rows[0];
		r1 = r->rows[1];
		i = 0;
#if OJZDL_HAVE_MXU2
		i = ojzdl_vresize_mxu2(r0, r1, b0, b1, d, n);
#endif
		for (; i < n; i++) {
			int t0 = (int16_t)((b0 * r0[i]) >> 16);
			int t1 = (int16_t)((b1 * r1[i]) >> 16);
			d[i] = (uint8_t)((t0 + t1 + 2) >> 2);
		}
	}
	return 0;
}

struct rgb_src {
	const uint8_t *p;
	size_t stride;
};

static const uint8_t *rgb_row(void *ctx, int y)
{
	const struct rgb_src *s = ctx;
	return s->p + (size_t)y * s->stride;
}

int ojzdl_resize_rgb(ojzdl_resize *r, const uint8_t *src, uint8_t *dst)
{
	struct rgb_src s = { src, (size_t)r->sw * 3 };
	return ojzdl_resize_rows(r, rgb_row, &s, dst);
}
