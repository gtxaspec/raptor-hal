/* ojzdl MXU2 kernels (T31). Same results as the C path in ojzdl.c, bit for bit;
 * the gate (gate.sh, and the v3 run) is what proves it.
 *
 * Register map, 1x1 conv (vector registers by number):
 *   v0..v7   accumulators, pixel p: v(2p) = outputs 0..7, v(2p+1) = outputs 8..15
 *   v8, v9   weight block halves (16 outputs x 2 inputs = 32 bytes)
 *   v10..v13 16 input bytes (8 pairs) of pixels 0..3
 *   v14..v17 broadcast input pair per pixel; reused as temps in requant
 *   v18,v19  bias  (int16, outputs 0..7 / 8..15)
 *   v20,v21  mult  (int16)
 *   v22,v23  shift (int16)
 *   v24      2^(bits-1)      v25  narrowing selector
 *   v26      zero            v27  2^(bits-1) - 1
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: MIT
 */
#include "ojzdl_mxu2.h"

#if OJZDL_HAVE_MXU2

#include <string.h>

#include "mxu2.h"

/* SHUFV selector: low bytes of lo (t) halfwords, then of hi (s) [mxu2.h SHUFV] */
static const uint8_t narrow_sel[16] __attribute__((aligned(16))) = {
	0, 4, 8, 12, 16, 20, 24, 28, 1, 5, 9, 13, 17, 21, 25, 29
};

typedef struct {
	int16_t v[8];
} h8;

/* one input pair k (0..7) of the 16-byte chunk, 4 pixels */
#define PW_STEP(k)                                                   \
	LU1Q(8, (k) * 32, GP4) LU1Q(9, (k) * 32 + 16, GP4)           \
	REPI(14, 10, k, DF_H) REPI(15, 11, k, DF_H)                  \
	REPI(16, 12, k, DF_H) REPI(17, 13, k, DF_H)                  \
	DADDS(0, 8, 14, DF_H) DADDS(1, 9, 14, DF_H)                  \
	DADDS(2, 8, 15, DF_H) DADDS(3, 9, 15, DF_H)                  \
	DADDS(4, 8, 16, DF_H) DADDS(5, 9, 16, DF_H)                  \
	DADDS(6, 8, 17, DF_H) DADDS(7, 9, 17, DF_H)

/* requant accumulators (lo, hi) of one pixel and store 16 bytes at GPR base */
#define PW_OUT(lo, hi, base) REQ_OUT(lo, hi, 14, base)
#define REQ_OUT(lo, hi, tmp, base)                                   \
	ADD(lo, lo, 18, DF_H) ADD(hi, hi, 19, DF_H)                  \
	MULQ_H(lo, lo, 20) MULQ_H(hi, hi, 21)                        \
	SRAR(lo, lo, 22, DF_H) SRAR(hi, hi, 23, DF_H)                \
	MAXS(lo, lo, 26, DF_H) MAXS(hi, hi, 26, DF_H)                \
	SUB(lo, lo, 24, DF_H) SUB(hi, hi, 24, DF_H)                  \
	MINS(lo, lo, 27, DF_H) MINS(hi, hi, 27, DF_H)                \
	SHUFV(tmp, hi, lo, 25) SU1Q(tmp, 0, base)

/* v24 = 2^(bits-1), v27 = 2^(bits-1) - 1, v25 = narrowing selector, v26 = 0 */
static void load_out_consts(int bits)
{
	h8 off, hi;
	int k;
	for (k = 0; k < 8; k++) {
		off.v[k] = (int16_t)(1 << (bits - 1));
		hi.v[k] = (int16_t)((1 << (bits - 1)) - 1);
	}
	{
		register const void *p0 __asm__("$8") = &off;
		register const void *p1 __asm__("$9") = &hi;
		register const void *p2 __asm__("$10") = narrow_sel;
		__asm__ volatile(LU1Q(24, 0, GP0) LU1Q(27, 0, GP1) LU1Q(25, 0, GP2) LI(26, 0, DF_H)
		                 : : "r"(p0), "r"(p1), "r"(p2), "m"(off), "m"(hi) : "memory");
	}
}

/* v18..v23 = bias, mult, shift of output group og (int16 x 16 each) */
static void load_requant(const uint8_t *bias, const uint8_t *mult, const uint8_t *shift, int og)
{
	register const uint8_t *pb __asm__("$8") = bias + 32 * og;
	register const uint8_t *pm __asm__("$9") = mult + 32 * og;
	register const uint8_t *ps __asm__("$10") = shift + 32 * og;
	__asm__ volatile(LU1Q(18, 0, GP0) LU1Q(19, 16, GP0)
	                 LU1Q(20, 0, GP1) LU1Q(21, 16, GP1)
	                 LU1Q(22, 0, GP2) LU1Q(23, 16, GP2)
	                 : : "r"(pb), "r"(pm), "r"(ps) : "memory");
}

size_t ojzdl_pw_mxu2(const int8_t *x, size_t npx, int ci, const int8_t *wp, int co,
                     const uint8_t *bias, const uint8_t *mult, const uint8_t *shift,
                     int bits, int8_t *dst)
{
	size_t nblk = npx / 4, b;
	int og, c0, ncp = ci / 2;

	load_out_consts(bits);
	for (og = 0; og < co / 16; og++) {
		load_requant(bias, mult, shift, og);
		for (b = 0; b < nblk; b++) {
			const int8_t *xb = x + b * 4 * (size_t)ci;
			__asm__ volatile(LI(0, 0, DF_H) LI(1, 0, DF_H) LI(2, 0, DF_H) LI(3, 0, DF_H)
			                 LI(4, 0, DF_H) LI(5, 0, DF_H) LI(6, 0, DF_H) LI(7, 0, DF_H) ::: "memory");
			for (c0 = 0; c0 < ci; c0 += 16) {
				register const int8_t *i0 __asm__("$8") = xb + c0;
				register const int8_t *i1 __asm__("$9") = xb + ci + c0;
				register const int8_t *i2 __asm__("$10") = xb + 2 * ci + c0;
				register const int8_t *i3 __asm__("$11") = xb + 3 * ci + c0;
				register const int8_t *pw __asm__("$12") = wp + ((size_t)og * ncp + c0 / 2) * 32;
				__asm__ volatile(LU1Q(10, 0, GP0) LU1Q(11, 0, GP1) LU1Q(12, 0, GP2) LU1Q(13, 0, GP3)
				                 PW_STEP(0) PW_STEP(1) PW_STEP(2) PW_STEP(3)
				                 PW_STEP(4) PW_STEP(5) PW_STEP(6) PW_STEP(7)
				                 : : "r"(i0), "r"(i1), "r"(i2), "r"(i3), "r"(pw) : "memory");
			}
			{
				int8_t *d0 = dst + b * 4 * (size_t)co + og * 16;
				register int8_t *o0 __asm__("$8") = d0;
				register int8_t *o1 __asm__("$9") = d0 + co;
				register int8_t *o2 __asm__("$10") = d0 + 2 * co;
				register int8_t *o3 __asm__("$11") = d0 + 3 * co;
				__asm__ volatile(PW_OUT(0, 1, GP0) PW_OUT(2, 3, GP1)
				                 PW_OUT(4, 5, GP2) PW_OUT(6, 7, GP3)
				                 : : "r"(o0), "r"(o1), "r"(o2), "r"(o3) : "memory");
			}
		}
	}
	return nblk * 4;
}

/* ---------------------------------------------------------------- depthwise
 * Register map: v0,v1 acc; v2..v7 temps; v8..v17 weights (5 pairs x lo/hi);
 * v18..v27 requant constants as for 1x1; v28/v29 interleave selectors.
 * Weight pair p, half h is at 32p + 16h of the group's 160 bytes [fmt: Weight layouts].
 */
static const uint8_t ilv_lo[16] __attribute__((aligned(16))) = {
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15
};
static const uint8_t ilv_hi[16] __attribute__((aligned(16))) = {
	16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31
};

/* Tap (row R, col K): K = 0 at the row base, K = 1/2 via index GP3 = c, GP4 = 2c
 * (2c exceeds LU1Q's s10 offset for c >= 256). Rows: GP0..GP2. */
#define TAP0(v, R) LU1Q(v, 0, R)
#define TAP1(v, R) LU1QX(v, R, GP3)
#define TAP2(v, R) LU1QX(v, R, GP4)
/* interleave taps in v2 (slot 0) and v3 (slot 1), accumulate with weight pair p */
#define DW_ACC(p, OP)                                                   \
	SHUFV(4, 3, 2, 28) SHUFV(5, 3, 2, 29)                           \
	OP(0, 8 + 2 * (p), 4, DF_H) OP(1, 9 + 2 * (p), 5, DF_H)

void ojzdl_dw_mxu2(const int8_t *xp, int wp_, int ho, int wo, int c, int stride, const int8_t *wt,
                   const uint8_t *bias, const uint8_t *mult, const uint8_t *shift,
                   int bits, int8_t *dst)
{
	int g, oy, ox;
	size_t rs = (size_t)wp_ * c; /* padded row stride, bytes */
	load_out_consts(bits);
	{
		register const void *a __asm__("$8") = ilv_lo;
		register const void *b __asm__("$9") = ilv_hi;
		__asm__ volatile(LU1Q(28, 0, GP0) LU1Q(29, 0, GP1) : : "r"(a), "r"(b) : "memory");
	}
	for (g = 0; g < c / 16; g++) {
		load_requant(bias, mult, shift, g);
		{
			register const int8_t *w __asm__("$8") = wt + 160 * g;
			__asm__ volatile(LU1Q(8, 0, GP0) LU1Q(9, 16, GP0) LU1Q(10, 32, GP0) LU1Q(11, 48, GP0)
			                 LU1Q(12, 64, GP0) LU1Q(13, 80, GP0) LU1Q(14, 96, GP0) LU1Q(15, 112, GP0)
			                 LU1Q(16, 128, GP0) LU1Q(17, 144, GP0)
			                 : : "r"(w) : "memory");
		}
		for (oy = 0; oy < ho; oy++)
			for (ox = 0; ox < wo; ox++) {
				register const int8_t *r0 __asm__("$8") = xp + (size_t)oy * stride * rs + (size_t)ox * stride * c + 16 * g;
				register const int8_t *r1 __asm__("$9") = r0 + rs;
				register const int8_t *r2 __asm__("$10") = r0 + 2 * rs;
				register size_t c1 __asm__("$11") = (size_t)c;
				register size_t c2 __asm__("$12") = 2 * (size_t)c;
				register int8_t *o __asm__("$13") = dst + ((size_t)oy * wo + ox) * c + 16 * g;
				__asm__ volatile(
					TAP0(2, GP0) TAP1(3, GP0) DW_ACC(0, DOTPS)   /* t0 t1 */
					TAP2(2, GP0) TAP0(3, GP1) DW_ACC(1, DADDS)   /* t2 t3 */
					TAP1(2, GP1) TAP2(3, GP1) DW_ACC(2, DADDS)   /* t4 t5 */
					TAP0(2, GP2) TAP1(3, GP2) DW_ACC(3, DADDS)   /* t6 t7 */
					TAP2(3, GP2) SHUFV(2, 3, 3, 28) SHUFV(4, 3, 3, 29) /* pad t8: pad weight is 0 */
					DADDS(0, 16, 2, DF_H) DADDS(1, 17, 4, DF_H)
					REQ_OUT(0, 1, 6, GP5)
					: : "r"(r0), "r"(r1), "r"(r2), "r"(c1), "r"(c2), "r"(o) : "memory");
			}
	}
}

/* ---------------------------------------------------------------- first conv
 * Each kernel row is kw*cin = 9 contiguous bytes of the padded HWC image; the
 * weight blocks pair slots (2p, 2p+1) of each 10-slot row [fmt: Weight layouts].
 * Same scheme as 1x1: v10..v13 = one row of 4 output pixels, REPI.h lane p.
 * Writes int16 accumulators [pixel][co]; the Q31 requant is done by the caller.
 */
#define FC_STEP(p, wofs)                                              \
	LU1Q(8, wofs, GP4) LU1Q(9, (wofs) + 16, GP4)                  \
	REPI(14, 10, p, DF_H) REPI(15, 11, p, DF_H)                   \
	REPI(16, 12, p, DF_H) REPI(17, 13, p, DF_H)                   \
	DADDS(0, 8, 14, DF_H) DADDS(1, 9, 14, DF_H)                   \
	DADDS(2, 8, 15, DF_H) DADDS(3, 9, 15, DF_H)                   \
	DADDS(4, 8, 16, DF_H) DADDS(5, 9, 16, DF_H)                   \
	DADDS(6, 8, 17, DF_H) DADDS(7, 9, 17, DF_H)
#define FC_ROW(ky)                                                    \
	LU1Q(10, 0, GP0) LU1Q(11, 0, GP1) LU1Q(12, 0, GP2) LU1Q(13, 0, GP3) \
	FC_STEP(0, ((ky) * 5 + 0) * 32) FC_STEP(1, ((ky) * 5 + 1) * 32) \
	FC_STEP(2, ((ky) * 5 + 2) * 32) FC_STEP(3, ((ky) * 5 + 3) * 32) \
	FC_STEP(4, ((ky) * 5 + 4) * 32)

/* Q31 requant of one pixel's halfword accumulators (lo, hi) [fmt: Arithmetic]:
 * widen even/odd outputs to words (DOTPS.w with 1,0 / 0,1 masks), ADD bias,
 * MULQ.w (sat32((a*b) >> 31) = the floor multiply), SRAR.w, clamp, narrow.
 * v8..v19 even/odd constants (see first_qtab), v20 off, v21 max, v22 zero,
 * v23/v24 masks, v25/v26 narrowing selectors; temps v27..v30. */
#define FQ_HALF(acc, bE, bO, mE, mO, sE, sO, nsel, out)               \
	DOTPS(27, acc, 23, DF_W) DOTPS(28, acc, 24, DF_W)              \
	ADD(27, 27, bE, DF_W) ADD(28, 28, bO, DF_W)                    \
	MULQ_W(27, 27, mE) MULQ_W(28, 28, mO)                          \
	SRAR(27, 27, sE, DF_W) SRAR(28, 28, sO, DF_W)                  \
	MAXS(27, 27, 22, DF_W) MAXS(28, 28, 22, DF_W)                  \
	SUB(27, 27, 20, DF_W) SUB(28, 28, 20, DF_W)                    \
	MINS(27, 27, 21, DF_W) MINS(28, 28, 21, DF_W)                  \
	SHUFV(out, 28, 27, nsel)
#define FQ_PIX(lo, hi, base)                                          \
	FQ_HALF(lo, 8, 9, 12, 13, 16, 17, 25, 29)                     \
	FQ_HALF(hi, 10, 11, 14, 15, 18, 19, 26, 30)                   \
	ORV(29, 29, 30) SU1Q(29, 0, base)

static const int16_t mask_even[8] __attribute__((aligned(16))) = { 1, 0, 1, 0, 1, 0, 1, 0 };
static const int16_t mask_odd[8] __attribute__((aligned(16))) = { 0, 1, 0, 1, 0, 1, 0, 1 };
static const uint8_t nsel_lo[16] __attribute__((aligned(16))) = {
	0, 1, 8, 9, 16, 17, 24, 25, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80
};
static const uint8_t nsel_hi[16] __attribute__((aligned(16))) = {
	0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0, 1, 8, 9, 16, 17, 24, 25
};

size_t ojzdl_first_mxu2(const int8_t *row0, size_t rs, int wo, int pstride,
                        const int8_t *wt, int per_group, int co, const int32_t *qtab,
                        int bits, int8_t *dst)
{
	int og, b, nblk = wo / 4, ky, k;
	int32_t off[4] __attribute__((aligned(16))), hi[4] __attribute__((aligned(16)));
	for (k = 0; k < 4; k++) {
		off[k] = 1 << (bits - 1);
		hi[k] = (1 << (bits - 1)) - 1;
	}
	{
		register const void *a0 __asm__("$8") = off;
		register const void *a1 __asm__("$9") = hi;
		register const void *a2 __asm__("$10") = mask_even;
		register const void *a3 __asm__("$11") = mask_odd;
		register const void *a4 __asm__("$12") = nsel_lo;
		register const void *a5 __asm__("$13") = nsel_hi;
		__asm__ volatile(LU1Q(20, 0, GP0) LU1Q(21, 0, GP1) LI(22, 0, DF_W)
		                 LU1Q(23, 0, GP2) LU1Q(24, 0, GP3) LU1Q(25, 0, GP4) LU1Q(26, 0, GP5)
		                 : : "r"(a0), "r"(a1), "r"(a2), "r"(a3), "r"(a4), "r"(a5), "m"(off), "m"(hi)
		                 : "memory");
	}
	for (og = 0; og < co / 16; og++)
		for (b = 0; b < nblk; b++) {
			const int8_t *px = row0 + (size_t)b * 4 * pstride;
			__asm__ volatile(LI(0, 0, DF_H) LI(1, 0, DF_H) LI(2, 0, DF_H) LI(3, 0, DF_H)
			                 LI(4, 0, DF_H) LI(5, 0, DF_H) LI(6, 0, DF_H) LI(7, 0, DF_H) ::: "memory");
			for (ky = 0; ky < 3; ky++) {
				register const int8_t *a0 __asm__("$8") = px + ky * rs;
				register const int8_t *a1 __asm__("$9") = a0 + pstride;
				register const int8_t *a2 __asm__("$10") = a0 + 2 * pstride;
				register const int8_t *a3 __asm__("$11") = a0 + 3 * pstride;
				register const int8_t *w __asm__("$12") = wt + (size_t)og * per_group;
				if (ky == 0)
					__asm__ volatile(FC_ROW(0) : : "r"(a0), "r"(a1), "r"(a2), "r"(a3), "r"(w) : "memory");
				else if (ky == 1)
					__asm__ volatile(FC_ROW(1) : : "r"(a0), "r"(a1), "r"(a2), "r"(a3), "r"(w) : "memory");
				else
					__asm__ volatile(FC_ROW(2) : : "r"(a0), "r"(a1), "r"(a2), "r"(a3), "r"(w) : "memory");
			}
			{
				register const int32_t *q __asm__("$12") = qtab + (size_t)og * 48;
				int8_t *d = dst + (size_t)b * 4 * co + og * 16;
				register int8_t *o0 __asm__("$8") = d;
				register int8_t *o1 __asm__("$9") = d + co;
				register int8_t *o2 __asm__("$10") = d + 2 * co;
				register int8_t *o3 __asm__("$11") = d + 3 * co;
				__asm__ volatile(LU1Q(8, 0, GP4) LU1Q(9, 16, GP4) LU1Q(10, 32, GP4) LU1Q(11, 48, GP4)
				                 LU1Q(12, 64, GP4) LU1Q(13, 80, GP4) LU1Q(14, 96, GP4) LU1Q(15, 112, GP4)
				                 LU1Q(16, 128, GP4) LU1Q(17, 144, GP4) LU1Q(18, 160, GP4) LU1Q(19, 176, GP4)
				                 FQ_PIX(0, 1, GP0) FQ_PIX(2, 3, GP1) FQ_PIX(4, 5, GP2) FQ_PIX(6, 7, GP3)
				                 : : "r"(q), "r"(o0), "r"(o1), "r"(o2), "r"(o3) : "memory");
			}
		}
	return (size_t)nblk * 4;
}

/* ---------------------------------------------------------------- 2x2/2 max-pool
 * One output row from two input rows a, b (w pixels x c bytes each). c % 16 == 0. */
void ojzdl_pool2_mxu2(const int8_t *a, const int8_t *b, int w, int c, int8_t *dst)
{
	int ox, g;
	for (ox = 0; ox < w / 2; ox++)
		for (g = 0; g < c; g += 16) {
			register const int8_t *p0 __asm__("$8") = a + (size_t)2 * ox * c + g;
			register const int8_t *p1 __asm__("$9") = p0 + c;
			register const int8_t *p2 __asm__("$10") = b + (size_t)2 * ox * c + g;
			register const int8_t *p3 __asm__("$11") = p2 + c;
			register int8_t *o __asm__("$12") = dst + (size_t)ox * c + g;
			__asm__ volatile(LU1Q(0, 0, GP0) LU1Q(1, 0, GP1) LU1Q(2, 0, GP2) LU1Q(3, 0, GP3)
			                 MAXS(0, 0, 1, DF_B) MAXS(2, 2, 3, DF_B) MAXS(0, 0, 2, DF_B)
			                 SU1Q(0, 0, GP4)
			                 : : "r"(p0), "r"(p1), "r"(p2), "r"(p3), "r"(o) : "memory");
		}
}

/* ---------------------------------------------------------------- head dot products
 * acc[px][o] = sum_c x[px][c] * w[o][c], exact int32: DOTPS.h gives pair sums
 * (caller proves they fit int16), DADDS.w with ones widens and accumulates.
 * ci % 16 == 0. v0 acc (4 words), v1 x, v2 w, v3 pair sums, v4 ones. */
void ojzdl_head_mxu2(const int8_t *x, size_t npx, int ci, const int8_t *w, int co, int32_t *acc)
{
	size_t px;
	int o, c0;
	int32_t part[4] __attribute__((aligned(16)));
	__asm__ volatile(LI(4, 1, DF_H) ::: "memory");
	for (px = 0; px < npx; px++)
		for (o = 0; o < co; o++) {
			__asm__ volatile(LI(0, 0, DF_W) ::: "memory");
			for (c0 = 0; c0 < ci; c0 += 32) {
				register const int8_t *xp __asm__("$8") = x + px * ci + c0;
				register const int8_t *wp __asm__("$9") = w + (size_t)o * ci + c0;
				if (c0 + 32 <= ci)
					__asm__ volatile(LU1Q(1, 0, GP0) LU1Q(2, 0, GP1) DOTPS(3, 1, 2, DF_H) DADDS(0, 3, 4, DF_W)
					                 LU1Q(1, 16, GP0) LU1Q(2, 16, GP1) DOTPS(3, 1, 2, DF_H) DADDS(0, 3, 4, DF_W)
					                 : : "r"(xp), "r"(wp) : "memory");
				else
					__asm__ volatile(LU1Q(1, 0, GP0) LU1Q(2, 0, GP1) DOTPS(3, 1, 2, DF_H) DADDS(0, 3, 4, DF_W)
					                 : : "r"(xp), "r"(wp) : "memory");
			}
			{
				register int32_t *pp __asm__("$8") = part;
				__asm__ volatile(SU1Q(0, 0, GP0) : : "r"(pp) : "memory");
			}
			acc[px * co + o] = part[0] + part[1] + part[2] + part[3];
		}
}
/* uint8 rgb - 128 as int8 is rgb ^ 0x80 [fmt: Arithmetic, input]; n % 16 == 0 part */
size_t ojzdl_input_mxu2(const uint8_t *rgb, size_t n, int8_t *dst)
{
	size_t i;
	for (i = 0; i + 16 <= n; i += 16) {
		register const uint8_t *s __asm__("$8") = rgb + i;
		register int8_t *d __asm__("$9") = dst + i;
		__asm__ volatile(LU1Q(0, 0, GP0) XORIB(0, 0, 0x80) SU1Q(0, 0, GP1) : : "r"(s), "r"(d) : "memory");
	}
	return i;
}
/* resize vertical pass [ojzdl_resize.c]: d = ((b0*r0) >> 16) + ((b1*r1) >> 16),
 * then (+2) >> 2. (b*r) >> 16 = (MULQ.h: sat16((b*r) >> 15)) >> 1, no saturation
 * since |b| <= 2048; (x + 2) >> 2 = SRARI 2. 16 bytes per step; returns bytes done. */
size_t ojzdl_vresize_mxu2(const int16_t *r0, const int16_t *r1, int b0, int b1, uint8_t *dst, size_t n)
{
	h8 vb0, vb1;
	size_t i;
	int k;
	for (k = 0; k < 8; k++) {
		vb0.v[k] = (int16_t)b0;
		vb1.v[k] = (int16_t)b1;
	}
	{
		register const void *a0 __asm__("$8") = &vb0;
		register const void *a1 __asm__("$9") = &vb1;
		register const void *a2 __asm__("$10") = narrow_sel;
		__asm__ volatile(LU1Q(10, 0, GP0) LU1Q(11, 0, GP1) LU1Q(12, 0, GP2)
		                 : : "r"(a0), "r"(a1), "r"(a2), "m"(vb0), "m"(vb1) : "memory");
	}
	for (i = 0; i + 16 <= n; i += 16) {
		register const int16_t *p0 __asm__("$8") = r0 + i;
		register const int16_t *p1 __asm__("$9") = r1 + i;
		register uint8_t *o __asm__("$10") = dst + i;
		__asm__ volatile(LU1Q(1, 0, GP0) LU1Q(2, 16, GP0) LU1Q(3, 0, GP1) LU1Q(4, 16, GP1)
		                 MULQ_H(1, 1, 10) MULQ_H(2, 2, 10) MULQ_H(3, 3, 11) MULQ_H(4, 4, 11)
		                 SRAI(1, 1, 1, DF_H) SRAI(2, 2, 1, DF_H) SRAI(3, 3, 1, DF_H) SRAI(4, 4, 1, DF_H)
		                 ADD(1, 1, 3, DF_H) ADD(2, 2, 4, DF_H)
		                 SRARI(1, 1, 2, DF_H) SRARI(2, 2, 2, DF_H)
		                 SHUFV(5, 2, 1, 12) SU1Q(5, 0, GP2)
		                 : : "r"(p0), "r"(p1), "r"(o) : "memory");
	}
	return i;
}
/* ---------------------------------------------------------------- head, 4 outputs per pass
 * As ojzdl_head_mxu2, for ci % 16 == 0 and ci <= 256: one pixel's input stays in
 * v8..v23 (16 bytes each) while every output reads it; four outputs accumulate at
 * once in v0..v3 (DOTPS.h pair sums, DADDS.w with ones in v31). Exact int32. */
#define HW(j, k, gp, a) LU1Q(4 + (j), (k) * 16, gp) DOTPS(24 + (j), 8 + (k), 4 + (j), DF_H) DADDS(a, 24 + (j), 31, DF_W)
/* four independent chains, interleaved so the in-order core doesn't wait on each result */
#define HEAD4(k)                                                                         \
	LU1Q(4, (k) * 16, GP0) LU1Q(5, (k) * 16, GP1) LU1Q(6, (k) * 16, GP2) LU1Q(7, (k) * 16, GP3) \
	DOTPS(24, 8 + (k), 4, DF_H) DOTPS(25, 8 + (k), 5, DF_H)                          \
	DOTPS(26, 8 + (k), 6, DF_H) DOTPS(27, 8 + (k), 7, DF_H)                          \
	DADDS(0, 24, 31, DF_W) DADDS(1, 25, 31, DF_W) DADDS(2, 26, 31, DF_W) DADDS(3, 27, 31, DF_W)
#define HEAD1(k) HW(0, k, GP0, 0)
#define HX(k) LU1Q(8 + (k), (k) * 16, GP0)

static void head_load_x(const int8_t *xp_, int nc)
{
	register const int8_t *xp __asm__("$8") = xp_;
	switch (nc) { /* fall through: chunks nc-1 .. 0 */
	case 16: __asm__ volatile(HX(15) : : "r"(xp) : "memory"); /* fall through */
	case 15: __asm__ volatile(HX(14) : : "r"(xp) : "memory"); /* fall through */
	case 14: __asm__ volatile(HX(13) : : "r"(xp) : "memory"); /* fall through */
	case 13: __asm__ volatile(HX(12) : : "r"(xp) : "memory"); /* fall through */
	case 12: __asm__ volatile(HX(11) : : "r"(xp) : "memory"); /* fall through */
	case 11: __asm__ volatile(HX(10) : : "r"(xp) : "memory"); /* fall through */
	case 10: __asm__ volatile(HX(9) : : "r"(xp) : "memory"); /* fall through */
	case 9: __asm__ volatile(HX(8) : : "r"(xp) : "memory"); /* fall through */
	case 8: __asm__ volatile(HX(7) : : "r"(xp) : "memory"); /* fall through */
	case 7: __asm__ volatile(HX(6) : : "r"(xp) : "memory"); /* fall through */
	case 6: __asm__ volatile(HX(5) : : "r"(xp) : "memory"); /* fall through */
	case 5: __asm__ volatile(HX(4) : : "r"(xp) : "memory"); /* fall through */
	case 4: __asm__ volatile(HX(3) : : "r"(xp) : "memory"); /* fall through */
	case 3: __asm__ volatile(HX(2) : : "r"(xp) : "memory"); /* fall through */
	case 2: __asm__ volatile(HX(1) : : "r"(xp) : "memory"); /* fall through */
	case 1: __asm__ volatile(HX(0) : : "r"(xp) : "memory"); /* fall through */
	default: break;
	}
}

#define HCASE(k, M)                                                                    \
	case k:                                                                        \
		__asm__ volatile(M(k) : : "r"(w0), "r"(w1), "r"(w2), "r"(w3) : "memory"); \
		break;
#define HSWITCH(k, M)                                                                  \
	switch (k) {                                                                   \
		HCASE(0, M) HCASE(1, M) HCASE(2, M) HCASE(3, M) HCASE(4, M) HCASE(5, M)   \
		HCASE(6, M) HCASE(7, M) HCASE(8, M) HCASE(9, M) HCASE(10, M) HCASE(11, M) \
		HCASE(12, M) HCASE(13, M) HCASE(14, M) HCASE(15, M) default: break;       \
	}

void ojzdl_head4_mxu2(const int8_t *x, size_t npx, int ci, const int8_t *w, int co, int32_t *acc)
{
	size_t px;
	int o, k, j, nc = ci / 16;
	int32_t part[16] __attribute__((aligned(16)));
	__asm__ volatile(LI(31, 1, DF_H) ::: "memory");
	for (px = 0; px < npx; px++) {
		head_load_x(x + px * ci, nc);
		for (o = 0; o + 4 <= co; o += 4) {
			register const int8_t *w0 __asm__("$8") = w + (size_t)o * ci;
			register const int8_t *w1 __asm__("$9") = w0 + ci;
			register const int8_t *w2 __asm__("$10") = w1 + ci;
			register const int8_t *w3 __asm__("$11") = w2 + ci;
			__asm__ volatile(LI(0, 0, DF_W) LI(1, 0, DF_W) LI(2, 0, DF_W) LI(3, 0, DF_W) ::: "memory");
			for (k = 0; k < nc; k++)
				HSWITCH(k, HEAD4)
			{
				register int32_t *pp __asm__("$12") = part;
				__asm__ volatile(SU1Q(0, 0, GP4) SU1Q(1, 16, GP4) SU1Q(2, 32, GP4) SU1Q(3, 48, GP4)
				                 : : "r"(pp) : "memory");
			}
			for (j = 0; j < 4; j++)
				acc[px * co + o + j] = part[4 * j] + part[4 * j + 1] + part[4 * j + 2] + part[4 * j + 3];
		}
		for (; o < co; o++) {
			register const int8_t *w0 __asm__("$8") = w + (size_t)o * ci;
			register const int8_t *w1 __asm__("$9") = w0;
			register const int8_t *w2 __asm__("$10") = w0;
			register const int8_t *w3 __asm__("$11") = w0;
			__asm__ volatile(LI(0, 0, DF_W) ::: "memory");
			for (k = 0; k < nc; k++)
				HSWITCH(k, HEAD1)
			{
				register int32_t *pp __asm__("$12") = part;
				__asm__ volatile(SU1Q(0, 0, GP4) : : "r"(pp) : "memory");
			}
			acc[px * co + o] = part[0] + part[1] + part[2] + part[3];
		}
	}
}

/* ---------------------------------------------------------------- eltwise (type 78)
 * [fmt: Layer types] u = round_half_even((ua + ub) / 2), ua = a + 2^(bits_in-1),
 * clamp to 2^bits_out - 1, stored u - 2^(bits_out-1). Byte lanes: needs bits_in <= 6
 * (ua + ub <= 126). rhe(s/2) = (s + ((s >> 1) & 1)) >> 1, and (k & 1) = k - 2 (k >> 1).
 * v20 = 2^bits_in, v21 = hi, v22 = 2^(bits_out-1). Returns bytes done (n & ~15). */
size_t ojzdl_add_mxu2(const int8_t *a, const int8_t *b, size_t n, int bits_in, int bits_out, int8_t *dst)
{
	int8_t c2[16] __attribute__((aligned(16))), hi[16] __attribute__((aligned(16))),
	       yo[16] __attribute__((aligned(16)));
	size_t i;
	int k;
	for (k = 0; k < 16; k++) {
		c2[k] = (int8_t)(1 << bits_in);
		hi[k] = (int8_t)((1 << bits_out) - 1 > 127 ? 127 : (1 << bits_out) - 1);
		yo[k] = (int8_t)(1 << (bits_out - 1));
	}
	{
		register const void *p0 __asm__("$8") = c2;
		register const void *p1 __asm__("$9") = hi;
		register const void *p2 __asm__("$10") = yo;
		__asm__ volatile(LU1Q(20, 0, GP0) LU1Q(21, 0, GP1) LU1Q(22, 0, GP2)
		                 : : "r"(p0), "r"(p1), "r"(p2), "m"(c2), "m"(hi), "m"(yo) : "memory");
	}
	for (i = 0; i + 16 <= n; i += 16) {
		register const int8_t *pa __asm__("$8") = a + i;
		register const int8_t *pb __asm__("$9") = b + i;
		register int8_t *o __asm__("$10") = dst + i;
		__asm__ volatile(LU1Q(0, 0, GP0) LU1Q(1, 0, GP1)
		                 ADD(0, 0, 1, DF_B) ADD(0, 0, 20, DF_B)  /* s = ua + ub */
		                 SRAI(1, 0, 1, DF_B) SRAI(2, 1, 1, DF_B) /* k = s >> 1, k >> 1 */
		                 ADD(2, 2, 2, DF_B) SUB(1, 1, 2, DF_B)   /* k & 1 */
		                 ADD(0, 0, 1, DF_B) SRAI(0, 0, 1, DF_B)  /* rhe(s / 2) */
		                 MINS(0, 0, 21, DF_B) SUB(0, 0, 22, DF_B)
		                 SU1Q(0, 0, GP2)
		                 : : "r"(pa), "r"(pb), "r"(o) : "memory");
	}
	return i;
}

#endif
