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
 * the T41's MXU3 passes, whose lanes are words. The values are the same (0 .. 32640). With MXU3
 * everything between the source row and the output is planar: the source row as three byte
 * planes (R, G, B), the resized rows as three int32 planes; the vertical pass interleaves. */
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
	int32_t *mx;           /* 64-byte aligned: 3 x 16 words (b0, b1, 2), 3 x 64 interleave
	                        * selectors, 64 output bytes */
	int nblk, rstride;     /* 16-pixel blocks per output row; words per row plane (16 x nblk) */
	uint8_t *plane;        /* the source row: 3 byte planes of pstride (64-byte aligned) */
	size_t pstride;
	int hsimd;             /* every block's source window fits 64 bytes: the MXU3 pass */
	int *hbase;            /* block b reads plane bytes hbase[b] .. + 63 (32-byte aligned) */
	uint8_t *hsel;         /* block b: 2 x 64 selectors (pixel s, pixel s + 1), 64-byte aligned */
	int32_t *ha;           /* block b: a0[16], a1[16], 64-byte aligned */
	int planar_src;        /* the row callback filled plane itself (ojzdl_resize_nv12) */
	uint8_t *csel;         /* NV12 conversion: per window offset 0..31, selectors for y, u, v
	                        * (3 x 64 bytes), then 5 x 16 words (128, 359, 88, 183, 454) and
	                        * 4 x 64 pack selectors; 64-byte aligned */
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

#ifdef OJZDL_MXU3
static void mxu3_tables(ojzdl_resize *r);
#endif

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
#ifdef OJZDL_MXU3
	r->nblk = (dst_w + 15) / 16;
	r->rstride = 16 * r->nblk;
#endif
	for (int k = 0; k < 2; k++) {
		void *m = NULL;
#ifdef OJZDL_MXU3
		size_t rb = (size_t)r->rstride * 3 * sizeof(rs_row_t) + 64;
#else
		size_t rb = (size_t)dst_w * 3 * sizeof(rs_row_t) + 64;
#endif
		if (posix_memalign(&m, 64, rb))
			m = NULL;
		r->rows[k] = m;
	}
	r->rgb = malloc((size_t)src_w * 3);
#ifdef OJZDL_MXU3
	{
		void *m[4] = { NULL, NULL, NULL, NULL };
		r->pstride = ((size_t)src_w + 2 + 31) / 32 * 32 + 64;
		if (posix_memalign(&m[0], 64, 3 * 64 + 3 * 64 + 64))
			m[0] = NULL;
		if (posix_memalign(&m[1], 64, 3 * r->pstride))
			m[1] = NULL;
		if (posix_memalign(&m[2], 64, (size_t)r->nblk * 128))
			m[2] = NULL;
		if (posix_memalign(&m[3], 64, (size_t)r->nblk * 32 * sizeof(int32_t)))
			m[3] = NULL;
		if (posix_memalign((void **)&r->csel, 64, 32 * 3 * 64 + 5 * 64 + 4 * 64))
			r->csel = NULL;
		r->mx = m[0];
		r->plane = m[1];
		r->hsel = m[2];
		r->ha = m[3];
		r->hbase = malloc((size_t)r->nblk * sizeof(int));
	}
	if (!r->mx || !r->plane || !r->hsel || !r->ha || !r->hbase || !r->csel) {
		ojzdl_resize_free(r);
		return NULL;
	}
	memset(r->plane, 0, 3 * r->pstride);
#endif
	if (!r->xofs || !r->yofs || !r->alpha || !r->beta || !r->rows[0] || !r->rows[1] || !r->rgb) {
		ojzdl_resize_free(r);
		return NULL;
	}
	tab_x(src_w, dst_w, r->xofs, r->alpha);
	tab_y(src_h, dst_h, r->yofs, r->beta);
#ifdef OJZDL_MXU3
	mxu3_tables(r);
#endif
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
	free(r->plane);
	free(r->hsel);
	free(r->ha);
	free(r->hbase);
	free(r->csel);
#endif
	free(r);
}

#ifndef OJZDL_MXU3
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
#endif

#ifdef OJZDL_MXU3
/* The passes on the T41's MXU3 (XBurst2): 16 word lanes per 64-byte VPR, the scalar loops'
 * arithmetic exactly (every product and sum is non-negative and fits 32 bits). The MXU3, like
 * the NN unit, is used on core 0 only: define OJZDL_MXU3 only where the caller runs there
 * (raptor-hal's onna backend pins its thread). Encodings: tools/onna/mxu3.h (gtxaspec/qemu
 * target/mips/tcg/mxu3.decode); pointer in $9, VPRs 1..12 used. */
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
#define RS_SUBW(vd, vs, vp) RS_3R(0x14, (0x2 << 2) | 2, vd, vs, vp) /* vs - vp */
/* signed words saturated to 0..255 */
#define RS_SATSU8W(vd, vs) (0x70000000 | (1 << 25) | (2 << 23) | (2 << 21) | ((vs) << 16) | ((vd) << 6) | 0x2f)
/* byte i of vd = byte (vs byte i) & 63 of vp, 0 where the selector byte has bit 7 set */
#define RS_GSHUFVB(vd, vs, vp) (0x70000000 | (0x1c << 21) | ((vs) << 16) | ((vp) << 11) | ((vd) << 6) | 0x2e)
/* 64 bytes at p (32-byte aligned) into / from VPR vr */
#define RS_VLOAD(vr, p) do { register const void *_p __asm__("$9") = (p); \
	__asm__ volatile(RS_W(RS_LAO(9, 0, 0, vr)) RS_W(RS_LAO(9, 1, 1, vr)) :: "r"(_p) : "memory"); } while (0)
#define RS_VSTORE(vr, p) do { register void *_p __asm__("$9") = (p); \
	__asm__ volatile(RS_W(RS_SAO(9, 0, 0, vr)) RS_W(RS_SAO(9, 1, 1, vr)) :: "r"(_p) : "memory"); } while (0)

/* horizontal: plane window in VPR 5, selectors (pixel s, s + 1) in VPRs 3, 4, a0 and a1 in
 * VPRs 1, 2: the gathers zero-extend each byte to a word lane, then (p0 a0 + p1 a1) >> 4 */
#define RS_H16() __asm__ volatile(RS_W(RS_GSHUFVB(6, 3, 5)) RS_W(RS_GSHUFVB(7, 4, 5)) \
	RS_W(RS_MULW(6, 6, 1)) RS_W(RS_MULW(7, 7, 2)) RS_W(RS_ADDW(6, 6, 7)) RS_W(RS_SRAIW(6, 6, 4)) ::: "memory")
/* vertical: 16 outputs of plane c from VPRs 8 (row 0) and 9 (row 1) with b0, b1, 2 in VPRs 1..3,
 * their low bytes to bytes 3j + c of VPR 10 (selector in VPR 4 + c) */
#define RS_V16(c) __asm__ volatile(RS_W(RS_MULW(8, 8, 1)) RS_W(RS_SRAIW(8, 8, 16)) \
	RS_W(RS_MULW(9, 9, 2)) RS_W(RS_SRAIW(9, 9, 16)) RS_W(RS_ADDW(8, 8, 9)) RS_W(RS_ADDW(8, 8, 3)) \
	RS_W(RS_SRAIW(8, 8, 2)) RS_W(RS_GSHUFVB(10, 4 + (c), 8)) ::: "memory")

/* NV12 -> RGB for 16 pixels: windows in VPRs 20 (y) and 21 (uv), their selectors in 22..24,
 * constants 128, 359, 88, 183, 454 in VPRs 1..5, pack selector q in VPR 6 + q; R, G, B words
 * clamped to 0..255, their bytes ORed into quarter q of VPRs 17, 18, 19 */
#define RS_C16(q) __asm__ volatile(RS_W(RS_GSHUFVB(25, 22, 20)) \
	RS_W(RS_GSHUFVB(26, 23, 21)) RS_W(RS_SUBW(26, 26, 1)) RS_W(RS_GSHUFVB(27, 24, 21)) RS_W(RS_SUBW(27, 27, 1)) \
	RS_W(RS_MULW(28, 27, 2)) RS_W(RS_SRAIW(28, 28, 8)) RS_W(RS_ADDW(28, 28, 25)) RS_W(RS_SATSU8W(28, 28)) \
	RS_W(RS_MULW(29, 26, 5)) RS_W(RS_SRAIW(29, 29, 8)) RS_W(RS_ADDW(29, 29, 25)) RS_W(RS_SATSU8W(29, 29)) \
	RS_W(RS_MULW(30, 26, 3)) RS_W(RS_MULW(31, 27, 4)) RS_W(RS_ADDW(30, 30, 31)) RS_W(RS_SRAIW(30, 30, 8)) \
	RS_W(RS_SUBW(30, 25, 30)) RS_W(RS_SATSU8W(30, 30)) \
	RS_W(RS_GSHUFVB(10, 6 + (q), 28)) RS_W(RS_GSHUFVB(11, 6 + (q), 30)) RS_W(RS_GSHUFVB(12, 6 + (q), 29)) \
	RS_W(RS_ORV(17, 17, 10)) RS_W(RS_ORV(18, 18, 11)) RS_W(RS_ORV(19, 19, 12)) ::: "memory")
#define RS_ZERO(vr) __asm__ volatile(RS_W(RS_SUBW(vr, vr, vr)) ::: "memory")

/* per-block windows, selectors and coefficients for the horizontal pass, and the vertical pass's
 * interleave selectors */
static void mxu3_tables(ojzdl_resize *r)
{
	uint8_t *ilv = (uint8_t *)(r->mx + 48);
	int b, j, c;

	r->hsimd = 1;
	for (b = 0; b < r->nblk; b++) {
		int dx0 = 16 * b, last = dx0 + 15 < r->dw - 1 ? dx0 + 15 : r->dw - 1;
		int base = r->xofs[dx0] & ~31;
		uint8_t *s0 = r->hsel + 128 * b, *s1 = s0 + 64;
		int32_t *a = r->ha + 32 * b;
		r->hbase[b] = base;
		if (r->xofs[last] + 1 - base > 63)
			r->hsimd = 0;
		memset(s0, 0x80, 128);
		for (j = 0; j < 16; j++) {
			int dx = dx0 + j;
			a[j] = a[16 + j] = 0;
			if (dx >= r->dw)
				continue;
			s0[4 * j] = (uint8_t)((r->xofs[dx] - base) & 63);
			s1[4 * j] = (uint8_t)((r->xofs[dx] + 1 - base) & 63);
			a[j] = r->alpha[2 * dx];
			a[16 + j] = r->alpha[2 * dx + 1];
		}
	}
	/* interleave selector c: output byte 3j + c <- byte 0 of word j */
	for (c = 0; c < 3; c++)
		for (j = 0; j < 64; j++)
			ilv[64 * c + j] = j < 48 && j % 3 == c ? (uint8_t)(4 * (j / 3)) : 0x80;
	/* NV12 conversion: for a window that starts o bytes before the pixel, word j <- y byte o + j,
	 * u byte o + 2 (j / 2), v byte o + 2 (j / 2) + 1; constants; pack quarter q: byte 16q + j <- word j */
	{
		uint8_t *cs = r->csel;
		int32_t *k = (int32_t *)(cs + 32 * 3 * 64);
		uint8_t *pk = (uint8_t *)(k + 5 * 16);
		static const int32_t kv[5] = { 128, 359, 88, 183, 454 };
		int o;
		memset(cs, 0x80, 32 * 3 * 64);
		for (o = 0; o < 32; o++)
			for (j = 0; j < 16; j++) {
				cs[o * 192 + 4 * j] = (uint8_t)(o + j);
				cs[o * 192 + 64 + 4 * j] = (uint8_t)(o + 2 * (j / 2));
				cs[o * 192 + 128 + 4 * j] = (uint8_t)(o + 2 * (j / 2) + 1);
			}
		for (c = 0; c < 5; c++)
			for (j = 0; j < 16; j++)
				k[16 * c + j] = kv[c];
		for (c = 0; c < 4; c++)
			for (j = 0; j < 64; j++)
				pk[64 * c + j] = (j >> 4) == c ? (uint8_t)(4 * (j & 15)) : 0x80;
	}
}

/* the source row in r->plane -> one resized row, three int32 planes of rstride */
static void hrow_mxu3(const ojzdl_resize *r, int32_t *dst)
{
	const size_t ps = r->pstride;
	const int rs = r->rstride;
	int b, c, dx;

	if (!r->hsimd) { /* a window wider than 64 bytes (downscale beyond ~2x): the same in C */
		for (c = 0; c < 3; c++) {
			const uint8_t *pl = r->plane + c * ps;
			for (dx = 0; dx < r->dw; dx++) {
				const uint8_t *p = pl + r->xofs[dx];
				dst[c * rs + dx] = (p[0] * r->alpha[2 * dx] + p[1] * r->alpha[2 * dx + 1]) >> 4;
			}
		}
		return;
	}
	for (b = 0; b < r->nblk; b++) {
		const uint8_t *w = r->plane + r->hbase[b];
		RS_VLOAD(1, r->ha + 32 * b);
		RS_VLOAD(2, r->ha + 32 * b + 16);
		RS_VLOAD(3, r->hsel + 128 * b);
		RS_VLOAD(4, r->hsel + 128 * b + 64);
		for (c = 0; c < 3; c++) {
			RS_VLOAD(5, w + c * ps);
			RS_H16();
			RS_VSTORE(6, dst + c * rs + 16 * b);
		}
	}
}

/* two planar resized rows -> one interleaved output row */
static void vresize_mxu3(const ojzdl_resize *r, const int32_t *r0, const int32_t *r1, int b0,
                         int b1, uint8_t *d)
{
	int32_t *k = r->mx;
	uint8_t *ilv = (uint8_t *)(k + 48), *out = ilv + 3 * 64;
	const int rs = r->rstride;
	int b, j;

	for (j = 0; j < 16; j++) {
		k[j] = b0;
		k[16 + j] = b1;
		k[32 + j] = 2;
	}
	RS_VLOAD(1, k);
	RS_VLOAD(2, k + 16);
	RS_VLOAD(3, k + 32);
	RS_VLOAD(4, ilv);
	RS_VLOAD(5, ilv + 64);
	RS_VLOAD(6, ilv + 128);
	for (b = 0; b < r->nblk; b++) {
		int px = r->dw - 16 * b < 16 ? r->dw - 16 * b : 16;
		RS_VLOAD(8, r0 + 16 * b);
		RS_VLOAD(9, r1 + 16 * b);
		RS_V16(0);
		__asm__ volatile(RS_W(RS_ORV(12, 10, 10)) ::: "memory");
		RS_VLOAD(8, r0 + rs + 16 * b);
		RS_VLOAD(9, r1 + rs + 16 * b);
		RS_V16(1);
		__asm__ volatile(RS_W(RS_ORV(12, 12, 10)) ::: "memory");
		RS_VLOAD(8, r0 + 2 * rs + 16 * b);
		RS_VLOAD(9, r1 + 2 * rs + 16 * b);
		RS_V16(2);
		__asm__ volatile(RS_W(RS_ORV(12, 12, 10)) ::: "memory");
		RS_VSTORE(12, out);
		memcpy(d + 48 * b, out, (size_t)px * 3);
	}
}

/* an interleaved RGB source row into the three planes */
static void deinterleave(const ojzdl_resize *r, const uint8_t *src)
{
	uint8_t *p0 = r->plane, *p1 = p0 + r->pstride, *p2 = p1 + r->pstride;
	int x;
	for (x = 0; x < r->sw; x++) {
		p0[x] = src[3 * x];
		p1[x] = src[3 * x + 1];
		p2[x] = src[3 * x + 2];
	}
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
#ifdef OJZDL_MXU3
			if (!r->planar_src)
				deinterleave(r, row);
			hrow_mxu3(r, r->rows[k]);
#else
			hrow(r, row, r->rows[k]);
#endif
			r->have[k] = y + k;
		}
	return 0;
}

static const uint8_t *nv12_row(void *ctx, int y);

int ojzdl_resize_rows(ojzdl_resize *r, const uint8_t *(*get_row)(void *ctx, int y), void *ctx,
                      uint8_t *dst)
{
	size_t n = (size_t)r->dw * 3, i;
	int dy;
	r->have[0] = r->have[1] = -1;
#ifdef OJZDL_MXU3
	r->planar_src = get_row == nv12_row;
#endif
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
		vresize_mxu3(r, r0, r1, b0, b1, d);
		i = n;
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
	int col;
#ifdef OJZDL_MXU3
	/* planar, straight into the planes the horizontal pass reads; 64 pixels at a time on the
	 * MXU3 where both 64-byte windows of each 16 lie inside the frame, the rest in C */
	uint8_t *pr = r->plane, *pg = pr + r->pstride, *pb = pg + r->pstride;
	const uintptr_t lo = (uintptr_t)s->p, hi = lo + (size_t)w * r->sh + (size_t)w * ((r->sh + 1) / 2);
	const uint8_t *cs = r->csel;
	const int32_t *k = (const int32_t *)(cs + 32 * 3 * 64);
	const uint8_t *pk = (const uint8_t *)(k + 5 * 16);
	int g, q, ok, loaded = 0;

	for (g = 0; g + 64 <= w; g += 64) {
		ok = 1;
		for (q = 0; q < 4 && ok; q++) {
			uintptr_t ya = (uintptr_t)(yp + g + 16 * q) & ~(uintptr_t)31;
			uintptr_t ua = (uintptr_t)(uvp + g + 16 * q) & ~(uintptr_t)31;
			ok = ya >= lo && ya + 64 <= hi && ua >= lo && ua + 64 <= hi;
		}
		if (!ok)
			break;
		if (!loaded) {
			RS_VLOAD(1, k);
			RS_VLOAD(2, k + 16);
			RS_VLOAD(3, k + 32);
			RS_VLOAD(4, k + 48);
			RS_VLOAD(5, k + 64);
			RS_VLOAD(6, pk);
			RS_VLOAD(7, pk + 64);
			RS_VLOAD(8, pk + 128);
			RS_VLOAD(9, pk + 192);
			loaded = 1;
		}
		RS_ZERO(17);
		RS_ZERO(18);
		RS_ZERO(19);
#define RS_CQ(q) do { \
		const uint8_t *yq = yp + g + 16 * (q), *uq = uvp + g + 16 * (q); \
		int yo = (int)((uintptr_t)yq & 31), uo = (int)((uintptr_t)uq & 31); \
		RS_VLOAD(20, yq - yo); \
		RS_VLOAD(21, uq - uo); \
		RS_VLOAD(22, cs + yo * 192); \
		RS_VLOAD(23, cs + uo * 192 + 64); \
		RS_VLOAD(24, cs + uo * 192 + 128); \
		RS_C16(q); \
	} while (0)
		RS_CQ(0);
		RS_CQ(1);
		RS_CQ(2);
		RS_CQ(3);
#undef RS_CQ
		RS_VSTORE(17, pr + g);
		RS_VSTORE(18, pg + g);
		RS_VSTORE(19, pb + g);
	}
	for (col = g; col < w; col += 2) {
		int u = uvp[col] - 128, v = uvp[col + 1] - 128;
		int dr = (359 * v) >> 8, dg = -((88 * u + 183 * v) >> 8), db = (454 * u) >> 8;
		int y0 = yp[col], y1 = yp[col + 1]; /* col + 1 == w: lands in the plane's padding */
		pr[col] = clip[y0 + dr];
		pg[col] = clip[y0 + dg];
		pb[col] = clip[y0 + db];
		pr[col + 1] = clip[y1 + dr];
		pg[col + 1] = clip[y1 + dg];
		pb[col + 1] = clip[y1 + db];
	}
	return r->plane;
#else
	uint8_t *o = r->rgb;

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
#endif
}

int ojzdl_resize_nv12(ojzdl_resize *r, const uint8_t *nv12, uint8_t *dst)
{
	struct nv12_src s = { r, nv12 };
	return ojzdl_resize_rows(r, nv12_row, &s, dst);
}
