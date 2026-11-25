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

#if defined(OJZDL_MXU3) && OJZDL_HAVE_MXU2
#error "OJZDL_MXU3 (T41) and OJZDL_MXU2 (T31) are exclusive"
#endif

/* Horizontally resized rows: int16 (as the T31's MXU2 vertical pass reads them), or int32 for
 * the T41's MXU3 pass, whose lanes are words. The values are the same (0 .. 32640). */
#ifdef OJZDL_MXU3
typedef int32_t rs_row_t;
#else
typedef int16_t rs_row_t;
#endif

struct ojzdl_resize {
	int sw, sh, dw, dh;
	int *xofs, *yofs;
	int16_t *alpha, *beta;
	rs_row_t *rows[2];     /* horizontally resized source rows (64-byte aligned) */
	int have[2];           /* source row index held in rows[k], or -1 */
	uint8_t *rgb;          /* one converted NV12 row (ojzdl_resize_nv12) */
	uint8_t clip[1024];    /* clip[v + CLIP_OFS] = v clamped to 0..255 */
#ifdef OJZDL_MXU3
	int32_t *mx;           /* 64-byte aligned: 3 x 16 words (b0, b1, 2), 4 x 64 selector
	                        * bytes, 64 output bytes */
#endif
};

#define CLIP_OFS 384 /* y + chroma term spans -227 .. 480 */

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
	/* aligned, + 64 bytes: the vector passes read whole 16- or 64-byte blocks */
	for (int k = 0; k < 2; k++) {
		void *m = NULL;
		if (posix_memalign(&m, 64, (size_t)dst_w * 3 * sizeof(rs_row_t) + 64))
			m = NULL;
		r->rows[k] = m;
	}
	r->rgb = malloc((size_t)src_w * 3);
#ifdef OJZDL_MXU3
	{
		void *m = NULL;
		if (posix_memalign(&m, 64, 3 * 64 + 4 * 64 + 64))
			m = NULL;
		r->mx = m;
	}
	if (!r->mx) {
		ojzdl_resize_free(r);
		return NULL;
	}
#endif
	if (!r->xofs || !r->yofs || !r->alpha || !r->beta || !r->rows[0] || !r->rows[1] || !r->rgb) {
		ojzdl_resize_free(r);
		return NULL;
	}
	tab_x(src_w, dst_w, r->xofs, r->alpha);
	tab_y(src_h, dst_h, r->yofs, r->beta);
	for (int v = 0; v < (int)sizeof(r->clip); v++) {
		int c = v - CLIP_OFS;
		r->clip[v] = (uint8_t)(c < 0 ? 0 : (c > 255 ? 255 : c));
	}
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
	free(r->rgb);
#ifdef OJZDL_MXU3
	free(r->mx);
#endif
	free(r);
}

static void hrow(const ojzdl_resize *r, const uint8_t *src, rs_row_t *dst)
{
	int dx, k;
	for (dx = 0; dx < r->dw; dx++) {
		const uint8_t *p = src + r->xofs[dx] * 3;
		int a0 = r->alpha[2 * dx], a1 = r->alpha[2 * dx + 1];
		for (k = 0; k < 3; k++)
			dst[dx * 3 + k] = (rs_row_t)((p[k] * a0 + p[3 + k] * a1) >> 4);
	}
}

#ifdef OJZDL_MXU3
/* The vertical pass on the T41's MXU3 (XBurst2): 16 word lanes per 64-byte VPR, the scalar
 * loop's arithmetic exactly (every product and sum is non-negative and fits 32 bits). The
 * MXU3, like the NN unit, is used on core 0 only: define OJZDL_MXU3 only where the caller runs
 * there (raptor-hal's onna backend pins its thread). Encodings: tools/onna/mxu3.h
 * (gtxaspec/qemu target/mips/tcg/mxu3.decode); pointer in $9, VPRs 1..12 used. */
#define RS_S_(x) #x
#define RS_S(x) RS_S_(x)
#define RS_W(w) ".word " RS_S(w) "\n\t"
#define RS_LAO(base, off, n, vr) (0x70000000 | ((base) << 21) | (((off) & 31) << 16) | ((n) << 14) | (3 << 11) | ((vr) << 6) | 0x11)
#define RS_SAO(base, off, n, vr) (0x70000000 | ((base) << 21) | (((off) & 31) << 16) | ((vr) << 11) | ((n) << 9) | (3 << 6) | 0x15)
#define RS_3R(minor, fn, vd, vs, vp) (0x48000000 | ((minor) << 21) | ((vs) << 16) | ((vp) << 11) | ((vd) << 6) | (fn))
#define RS_ADDW(vd, vs, vp) RS_3R(0x14, 0x02, vd, vs, vp)
#define RS_MULW(vd, vs, vp) RS_3R(0x13, (0x8 << 2) | 2, vd, vs, vp) /* low 32 bits */
#define RS_SRAIW(vd, vs, imm) RS_3R(0x15, (0xe << 2) | 2, vd, vs, imm)
#define RS_ORV(vd, vs, vp) RS_3R(0x13, 0x04, vd, vs, vp)
/* byte i of vd = byte (vs byte i) & 63 of vp, 0 where the selector byte has bit 7 set */
#define RS_GSHUFVB(vd, vs, vp) (0x70000000 | (0x1c << 21) | ((vs) << 16) | ((vp) << 11) | ((vd) << 6) | 0x2e)
#define RS_VLOAD(vr, p) do { register const void *_p __asm__("$9") = (p); \
	__asm__ volatile(RS_W(RS_LAO(9, 0, 0, vr)) RS_W(RS_LAO(9, 1, 1, vr)) :: "r"(_p) : "memory"); } while (0)
#define RS_VSTORE(vr, p) do { register void *_p __asm__("$9") = (p); \
	__asm__ volatile(RS_W(RS_SAO(9, 0, 0, vr)) RS_W(RS_SAO(9, 1, 1, vr)) :: "r"(_p) : "memory"); } while (0)
/* 16 outputs from VPRs 8 (row 0) and 9 (row 1) with b0, b1, 2 in VPRs 1..3, then their low bytes
 * into quarter q of VPR 12 (selector q in VPR 4 + q) */
#define RS_V16(q) __asm__ volatile(RS_W(RS_MULW(8, 8, 1)) RS_W(RS_SRAIW(8, 8, 16)) \
	RS_W(RS_MULW(9, 9, 2)) RS_W(RS_SRAIW(9, 9, 16)) RS_W(RS_ADDW(8, 8, 9)) RS_W(RS_ADDW(8, 8, 3)) \
	RS_W(RS_SRAIW(8, 8, 2)) RS_W(RS_GSHUFVB(10, 4 + (q), 8)) ::: "memory")

static size_t vresize_mxu3(const ojzdl_resize *r, const int32_t *r0, const int32_t *r1, int b0,
                           int b1, uint8_t *d, size_t n)
{
	int32_t *k = r->mx;
	uint8_t *sel = (uint8_t *)(k + 48), *out = sel + 4 * 64;
	size_t i;
	int j, q;

	for (j = 0; j < 16; j++) {
		k[j] = b0;
		k[16 + j] = b1;
		k[32 + j] = 2;
	}
	/* selector q: output byte 16q + j <- byte 0 of word j; the other quarters 0 */
	for (q = 0; q < 4; q++)
		for (j = 0; j < 64; j++)
			sel[64 * q + j] = (j >> 4) == q ? (uint8_t)(4 * (j & 15)) : 0x80;
	RS_VLOAD(1, k);
	RS_VLOAD(2, k + 16);
	RS_VLOAD(3, k + 32);
	RS_VLOAD(4, sel);
	RS_VLOAD(5, sel + 64);
	RS_VLOAD(6, sel + 128);
	RS_VLOAD(7, sel + 192);
	for (i = 0; i + 64 <= n; i += 64) {
		RS_VLOAD(8, r0 + i);
		RS_VLOAD(9, r1 + i);
		RS_V16(0);
		__asm__ volatile(RS_W(RS_ORV(12, 10, 10)) ::: "memory");
		RS_VLOAD(8, r0 + i + 16);
		RS_VLOAD(9, r1 + i + 16);
		RS_V16(1);
		__asm__ volatile(RS_W(RS_ORV(12, 12, 10)) ::: "memory");
		RS_VLOAD(8, r0 + i + 32);
		RS_VLOAD(9, r1 + i + 32);
		RS_V16(2);
		__asm__ volatile(RS_W(RS_ORV(12, 12, 10)) ::: "memory");
		RS_VLOAD(8, r0 + i + 48);
		RS_VLOAD(9, r1 + i + 48);
		RS_V16(3);
		__asm__ volatile(RS_W(RS_ORV(12, 12, 10)) ::: "memory");
		RS_VSTORE(12, out);
		memcpy(d + i, out, 64);
	}
	return i;
}
#endif

/* rows[0] = source row y, rows[1] = row y + 1; reuse what is already held */
static int load_pair(ojzdl_resize *r, int y, const uint8_t *(*get_row)(void *, int), void *ctx)
{
	int k;
	if (r->have[0] == y && r->have[1] == y + 1)
		return 0;
	if (r->have[1] == y) { /* advance by one: the old lower row becomes the upper */
		rs_row_t *t = r->rows[0];
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
		const rs_row_t *r0, *r1;
		uint8_t *d = dst + (size_t)dy * n;
		if (load_pair(r, r->yofs[dy], get_row, ctx))
			return -1;
		r0 = r->rows[0];
		r1 = r->rows[1];
		i = 0;
#if OJZDL_HAVE_MXU2
		i = ojzdl_vresize_mxu2(r0, r1, b0, b1, d, n);
#elif defined(OJZDL_MXU3)
		i = vresize_mxu3(r, r0, r1, b0, b1, d, n);
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

struct nv12_src {
	ojzdl_resize *r;
	const uint8_t *p;
};

/* One NV12 row to RGB: the chroma terms once per pixel pair, clamping by table.
 * Same integer arithmetic as the per-pixel formula, so the same bytes (an odd
 * last column reads uv[col | 1] past the row, as the per-pixel form does). */
static const uint8_t *nv12_row(void *ctx, int y)
{
	const struct nv12_src *s = ctx;
	const ojzdl_resize *r = s->r;
	const int w = r->sw;
	const uint8_t *yp = s->p + (size_t)y * w;
	const uint8_t *uvp = s->p + (size_t)w * r->sh + (size_t)(y / 2) * w;
	const uint8_t *clip = r->clip + CLIP_OFS;
	uint8_t *o = r->rgb;
	int col;

	for (col = 0; col < w; col += 2) {
		int u = uvp[col] - 128, v = uvp[col + 1] - 128;
		int dr = (359 * v) >> 8, dg = -((88 * u + 183 * v) >> 8), db = (454 * u) >> 8;
		int y0 = yp[col];
		o[0] = clip[y0 + dr];
		o[1] = clip[y0 + dg];
		o[2] = clip[y0 + db];
		if (col + 1 < w) {
			int y1 = yp[col + 1];
			o[3] = clip[y1 + dr];
			o[4] = clip[y1 + dg];
			o[5] = clip[y1 + db];
		}
		o += 6;
	}
	return r->rgb;
}

int ojzdl_resize_nv12(ojzdl_resize *r, const uint8_t *nv12, uint8_t *dst)
{
	struct nv12_src s = { r, nv12 };
	return ojzdl_resize_rows(r, nv12_row, &s, dst);
}
