/* nnacpu: the CPU half of a venus-compatible T41 NNA runtime. See nnacpu.h.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: MIT
 */
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "nnacpu.h"
#ifdef ONNA_MXU3
#include "mxu3.h"
#endif

/* rint (ties to even) without a libm call: for |x| < 2^22, adding and subtracting 1.5 * 2^23 in
 * float32 rounds to the nearest integer, ties to even (the FPU's default mode), exactly as rint */
static inline long frint(float x)
{
	if (x > -4194304.0f && x < 4194304.0f) {
		const float m = 12582912.0f;		/* not folded without -ffast-math */

		return (long)((x + m) - m);
	}
	return lrintf(x);
}

static inline uint8_t clip8(long v)
{
	return v < 0 ? 0 : v > 255 ? 255 : (uint8_t)v;
}

void nna_silu_load(struct nna_silu *t, const int16_t raw[64])
{
	for (int i = 0; i < 16; i++) {
		t->dy[i] = raw[2 * i];
		t->dy[16 + i] = raw[2 * i + 1];
		t->y[i] = raw[32 + 2 * i];
		t->y[16 + i] = raw[32 + 2 * i + 1];
	}
}

void nna_requant_silu(const int32_t *S, int npx, int noc, const int32_t *B, const float *Fl,
		      const struct nna_silu *tb, uint8_t *q)
{
	for (int p = 0; p < npx; p++) {
		for (int c = 0; c < noc; c++) {
			float v = (float)(S[(long)p * noc + c] + B[c]) * Fl[c];
			long t = frint(v);
			long seg = t >> 10;			/* floor, also for t < 0 */
			float y;

			if (seg < 0) {
				y = (float)tb->y[0];
			} else {
				long sc = seg > 31 ? 31 : seg;
				long tc = t > 32767 ? 32767 : t;
				float fr = (float)(tc - sc * 1024);
				float inc = (float)tb->dy[sc] * fr;

				inc = inc / 1024.0f;
				y = (float)tb->y[sc] + inc;
			}
			q[(long)p * noc + c] = clip8(frint(y / 128.0f));
		}
	}
}

void nna_head(const int32_t *S, int npx, int noc, const int32_t *B, const float *Fl, float *out)
{
	for (long p = 0; p < npx; p++, S += noc, out += noc)
		for (int c = 0; c < noc; c++)
			out[c] = (float)(S[c] + B[c]) * Fl[c];
}

void nna_rescale(const uint8_t *x, int n, float a, uint8_t *out)
{
	for (int i = 0; i < n; i++)
		out[i] = clip8(frint(((float)x[i] - 128.0f) * a) + 128);
}

void nna_add_ref(const uint8_t *a, const uint8_t *b, int n, int wa, int wb, int32_t B, int32_t T,
		 uint8_t *out)
{
	for (int i = 0; i < n; i++) {
		long long num = 64LL * (wa * a[i] + wb * b[i]) + B;
		long long qq = num / T;

		if ((num % T) && ((num < 0) != (T < 0)))
			qq--;				/* floor division */
		out[i] = clip8((long)qq + 128);
	}
}

#ifdef ONNA_MXU3
/* out[i] = max of the n 64-byte vectors at src[k] + i * 64, for nv vectors (all 64-byte aligned) */
static void vmax_rows(uint8_t *out, const uint8_t *const *src, int n, int nv)
{
	for (int v = 0; v < nv; v++) {
		VLOAD(1, src[0] + 64 * v);
		for (int k = 1; k < n; k++) {
			VLOAD(2, src[k] + 64 * v);
			__asm__ volatile(MXU3_WORD(MAXUB_W(1, 1, 2)) ::: "memory");
		}
		VSTORE(1, out + 64 * v);
	}
}
#endif

void nna_maxpool5(const uint8_t *x, int h, int w, int c, uint8_t *out)
{
#ifdef ONNA_MXU3
	/* 64 channels per vector op when every pixel starts 64-byte aligned */
	if (c % 64 == 0 && !((uintptr_t)x & 63) && !((uintptr_t)out & 63)) {
		void *rm = NULL;
		uint8_t *rowmax;
		const uint8_t *src[5];

		if (posix_memalign(&rm, 64, (size_t)h * w * c))
			abort();
		rowmax = rm;
		for (int y = 0; y < h; y++)
			for (int xx = 0; xx < w; xx++) {
				int a = xx - 2 < 0 ? 0 : xx - 2, b = xx + 2 >= w ? w - 1 : xx + 2;

				for (int x2 = a; x2 <= b; x2++)
					src[x2 - a] = &x[((long)y * w + x2) * c];
				vmax_rows(&rowmax[((long)y * w + xx) * c], src, b - a + 1, c / 64);
			}
		for (int y = 0; y < h; y++) {
			int a = y - 2 < 0 ? 0 : y - 2, b = y + 2 >= h ? h - 1 : y + 2;

			for (int xx = 0; xx < w; xx++) {
				for (int y2 = a; y2 <= b; y2++)
					src[y2 - a] = &rowmax[((long)y2 * w + xx) * c];
				vmax_rows(&out[((long)y * w + xx) * c], src, b - a + 1, c / 64);
			}
		}
		free(rowmax);
		return;
	}
#endif
	/* separable: max over 5 columns, then over 5 rows (the same result as 5x5, 10 compares) */
	uint8_t *rowmax = malloc((size_t)h * w * c);

	for (int y = 0; y < h; y++)
		for (int xx = 0; xx < w; xx++) {
			int a = xx - 2 < 0 ? 0 : xx - 2, b = xx + 2 >= w ? w - 1 : xx + 2;
			uint8_t *o = &rowmax[((long)y * w + xx) * c];

			memcpy(o, &x[((long)y * w + a) * c], (size_t)c);
			for (int x2 = a + 1; x2 <= b; x2++) {
				const uint8_t *s = &x[((long)y * w + x2) * c];

				for (int k = 0; k < c; k++)
					o[k] = s[k] > o[k] ? s[k] : o[k];
			}
		}
	for (int y = 0; y < h; y++) {
		int a = y - 2 < 0 ? 0 : y - 2, b = y + 2 >= h ? h - 1 : y + 2;

		for (int xx = 0; xx < w; xx++) {
			uint8_t *o = &out[((long)y * w + xx) * c];

			memcpy(o, &rowmax[((long)a * w + xx) * c], (size_t)c);
			for (int y2 = a + 1; y2 <= b; y2++) {
				const uint8_t *s = &rowmax[((long)y2 * w + xx) * c];

				for (int k = 0; k < c; k++)
					o[k] = s[k] > o[k] ? s[k] : o[k];
			}
		}
	}
	free(rowmax);
}

void nna_upsample2(const uint8_t *x, int h, int w, int c, uint8_t *out)
{
	for (int y = 0; y < 2 * h; y++)
		for (int xx = 0; xx < 2 * w; xx++)
			memcpy(&out[((long)y * 2 * w + xx) * c], &x[((long)(y / 2) * w + xx / 2) * c], c);
}

void nna_silu_lut(const struct nna_silu *tb, uint8_t lut[NNA_SILU_LUT])
{
	/* the same float32 code as nna_requant_silu, for every t in -1 (= any t < 0) .. 32767 */
	for (long t = -1; t <= 32767; t++) {
		float y;

		if (t < 0) {
			y = (float)tb->y[0];
		} else {
			long sc = (t >> 10) > 31 ? 31 : t >> 10;
			float inc = (float)tb->dy[sc] * (float)(t - sc * 1024);

			inc = inc / 1024.0f;
			y = (float)tb->y[sc] + inc;
		}
		lut[t + 1] = clip8(frint(y / 128.0f));
	}
}


void nna_requant_lut(const int32_t *S, int npx, int noc, const int32_t *B, const float *Fl,
		     const uint8_t lut[NNA_SILU_LUT], uint8_t *q)
{
#ifdef ONNA_MXU3
	/* MXU3: t for 16 channels at a time (ADDW, FCVT int->f, FMULW, FCVT f->int nearest even),
	 * bit-identical to the scalar path below (rqvtest: 2M cases on silicon) */
	if (noc % 16 == 0) {
		int32_t t16[16] __attribute__((aligned(64)));

		for (int p = 0; p < npx; p++) {
			const int32_t *s = S + (long)p * noc;
			uint8_t *o = q + (long)p * noc;

			for (int c = 0; c < noc; c += 16) {
				VREQUANT16(s + c, B + c, Fl + c, t16);
				for (int i = 0; i < 16; i++) {
					long t = t16[i];

					t = t < 0 ? -1 : t > 32767 ? 32767 : t;
					o[c + i] = lut[t + 1];
				}
			}
		}
		return;
	}
#endif
	for (int p = 0; p < npx; p++) {
		const int32_t *s = S + (long)p * noc;
		uint8_t *o = q + (long)p * noc;

		for (int c = 0; c < noc; c++) {
			long t = frint((float)(s[c] + B[c]) * Fl[c]);

			t = t < 0 ? -1 : t > 32767 ? 32767 : t;
			o[c] = lut[t + 1];
		}
	}
}

void nna_add_lut(int wa, int wb, int32_t B, int32_t T, uint8_t lut[65536])
{
	static uint8_t av[65536], bv[65536];

	for (int i = 0; i < 65536; i++)
		av[i] = (uint8_t)(i >> 8), bv[i] = (uint8_t)i;
	nna_add_ref(av, bv, 65536, wa, wb, B, T, lut);
}

void nna_add_apply(const uint8_t *a, const uint8_t *b, int n, const uint8_t lut[65536], uint8_t *out)
{
	for (int i = 0; i < n; i++)
		out[i] = lut[(unsigned)a[i] << 8 | b[i]];
}

void nna_requant_t(const int32_t *S, int nvec, const int32_t *B16, const float *Fl16, int32_t *t)
{
#ifdef ONNA_MXU3
	VRQ_SETUP(B16, Fl16);
	for (int i = 0; i < nvec; i++)
		VRQ_ONE(S + 16 * i, t + 16 * i);
#else
	for (int i = 0; i < nvec; i++)
		for (int c = 0; c < 16; c++)
			t[16 * i + c] = (int32_t)frint((float)(S[16 * i + c] + B16[c]) * Fl16[c]);
#endif
}

#ifdef ONNA_MXU3
/* constants of the vector SiLU requant, loaded into VPRs by nna_vrq_load */
static const struct {
	int32_t zero[16], max_t[16], m1023[16], selk[16], sel[16];
	float f2m10[16], f2m7[16];
	uint8_t pack0[64], pack1[64], pack2[64];
} vk __attribute__((aligned(64))) = {
#define X16(v) { v, v, v, v, v, v, v, v, v, v, v, v, v, v, v, v }
	X16(0), X16(32767), X16(1023), X16(0x02020000),	/* s * this = (s << 17) + (s << 25) */
	X16(0x01008080),	/* bytes 2s, 2s + 1 into the high half of each word (s added twice) */
	X16(1.0f / 1024), X16(1.0f / 8388608),	/* 2^-10; 2^-23 = 2^-7 for the table's 2^16 */
#undef X16
	/* pack: word j's low byte to byte j (half 0) or byte 16 + j (half 1), 0 elsewhere */
	{ 0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56, 60,
	  128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128,
	  128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128,
	  128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128 },
	{ 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128,
	  0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56, 60,
	  128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128,
	  128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128 },
	/* pack at byte 32 (the second pixel of a wide 16-channel pair) */
	{ 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128,
	  128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128,
	  0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56, 60,
	  128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128 },
};

/*
 * VPRs: 2/3 B/Fl half 0, 12/13 half 1; 5 y table, 6 dy table; 7 sel, 8 32767, 9 0, 10 2^-10,
 * 11 2^-23, 14 1023, 15 0x02020000, 16/17 pack; 4 and 1 t; 18..30 scratch. (0 = NN data, 31 = VWR: untouched)
 */
void nna_vrq_load_bfl(const struct nna_vrq *v)
{
	VLOAD(2, v->B[0]); VLOAD(3, v->Fl[0]); VLOAD(12, v->B[1]); VLOAD(13, v->Fl[1]);
}

void nna_vrq_load_const(const struct nna_vrq *v)
{
	VLOAD(6, v->tb.dy); VLOAD(5, v->tb.y);
	VLOAD(7, vk.sel); VLOAD(8, vk.max_t); VLOAD(9, vk.zero); VLOAD(10, vk.f2m10);
	VLOAD(11, vk.f2m7); VLOAD(14, vk.m1023); VLOAD(15, vk.selk); VLOAD(16, vk.pack0);
	VLOAD(17, v->wide ? vk.pack2 : vk.pack1);
}

void nna_vrq_load(const struct nna_vrq *v)
{
	nna_vrq_load_bfl(v);
	nna_vrq_load_const(v);
}

/* per half: t = rint(f32(S + B) * Fl), then the SiLU segment table, as nna_requant_silu:
 * tc = clip(t, 0, 32767); s = tc >> 10; inc = f32(dy[s]) * f32(tc & 1023) / 1024;
 * q = clip(rint((f32(y[s]) + inc) / 128), 0, 255) (t < 0 gives y[0], as the scalar path).
 * The gather leaves dy and y times 2^16 (high halves); float32 rounding doesn't change under
 * a power-of-two scale, so they're used as they are and the last multiply is 2^-23, not 2^-7;
 * the bytes of both halves packed into VPR 23 (32 channels) */

/* both halves interleaved (independent chains, so the vector unit's latencies overlap):
 * half 0 in t 4, s 18, fr 19, sel 20, dy 21, y 22; half 1 in t 1, s 25, fr 26, sel 27, dy 28, y 29;
 * same operations and order per half as VRQ_SILU */
#define I2(a0, a1) MXU3_WORD(a0) MXU3_WORD(a1)
#define VRQ_SILU2 \
	I2(ADDW_W(4, 4, 2), ADDW_W(1, 1, 12)) I2(FCVT_W(2, 4, 4), FCVT_W(2, 1, 1)) \
	I2(FMULW_W(4, 4, 3), FMULW_W(1, 1, 13)) I2(FCVT_W(7, 4, 4), FCVT_W(7, 1, 1)) \
	I2(MAXSW_W(4, 4, 9), MAXSW_W(1, 1, 9)) I2(MINSW_W(4, 4, 8), MINSW_W(1, 1, 8)) \
	I2(SRAIW_W(18, 4, 10), SRAIW_W(25, 1, 10)) I2(ANDV_W(19, 4, 14), ANDV_W(26, 1, 14)) \
	I2(MULW_W(20, 18, 15), MULW_W(27, 25, 15)) I2(ADDW_W(20, 20, 7), ADDW_W(27, 27, 7)) \
	I2(GSHUFVB_W(21, 20, 6), GSHUFVB_W(28, 27, 6)) I2(GSHUFVB_W(22, 20, 5), GSHUFVB_W(29, 27, 5)) \
	I2(FCVT_W(2, 21, 21), FCVT_W(2, 28, 28)) I2(FCVT_W(2, 19, 19), FCVT_W(2, 26, 26)) \
	I2(FMULW_W(21, 21, 19), FMULW_W(28, 28, 26)) I2(FMULW_W(21, 21, 10), FMULW_W(28, 28, 10)) \
	I2(FCVT_W(2, 22, 22), FCVT_W(2, 29, 29)) I2(FADDW_W(22, 22, 21), FADDW_W(29, 29, 28)) \
	I2(FMULW_W(22, 22, 11), FMULW_W(29, 29, 11)) I2(FCVT_W(7, 22, 22), FCVT_W(7, 29, 29)) \
	I2(SATSU8W_W(22, 22), SATSU8W_W(29, 29)) \
	I2(GSHUFVB_W(23, 16, 22), GSHUFVB_W(24, 17, 29)) MXU3_WORD(ORV_W(23, 23, 24))
#define VRQ_DRD_PX(p) case p: \
	__asm__ volatile(MXU3_WORD(NNR_W(3, 2 * (p), 4)) MXU3_WORD(NNR_W(3, 2 * (p) + 1, 1)) VRQ_SILU2 \
			 ::: "memory"); break;
#define VRQ_DRD_DROP(p) case p: \
	__asm__ volatile(MXU3_WORD(NNR_W(3, 2 * (p), 4)) MXU3_WORD(NNR_W(3, 2 * (p) + 1, 1)) ::: "memory"); \
	return;

#define VRQ_DRD_PAIR16(p) case p: \
	__asm__ volatile(MXU3_WORD(NNR_W(3, 2 * (p), 4)) MXU3_WORD(NNR_W(3, 2 * (p) + 1, 30)) \
			 MXU3_WORD(NNR_W(3, 2 * (p) + 2, 1)) MXU3_WORD(NNR_W(3, 2 * (p) + 3, 30)) \
			 VRQ_SILU2 ::: "memory"); break;

void nna_vrq_drd_pair16(int p, uint8_t *q, int nbytes)
{
	switch (p) {
	VRQ_DRD_PAIR16(0) VRQ_DRD_PAIR16(2) VRQ_DRD_PAIR16(4) VRQ_DRD_PAIR16(6)
	default: return;
	}
	if (nbytes == 64) {
		VSTORE(23, q);
		return;
	}
	{
		register void *_p __asm__("$9") = q;

		__asm__ volatile(MXU3_WORD(SAO_W(9, 0, 0, 23)) :: "r"(_p) : "memory");
	}
}

void nna_vrq_drd_px(int p, uint8_t *q)
{
	if (!q) {
		switch (p) {
		VRQ_DRD_DROP(0) VRQ_DRD_DROP(1) VRQ_DRD_DROP(2) VRQ_DRD_DROP(3)
		VRQ_DRD_DROP(4) VRQ_DRD_DROP(5) VRQ_DRD_DROP(6) VRQ_DRD_DROP(7)
		default: return;
		}
	}
	switch (p) {
	VRQ_DRD_PX(0) VRQ_DRD_PX(1) VRQ_DRD_PX(2) VRQ_DRD_PX(3)
	VRQ_DRD_PX(4) VRQ_DRD_PX(5) VRQ_DRD_PX(6) VRQ_DRD_PX(7)
	default: return;
	}
	{
		register void *_p __asm__("$9") = q;

		__asm__ volatile(MXU3_WORD(SAO_W(9, 0, 0, 23)) :: "r"(_p) : "memory");
	}
}

#define VHEAD_DRD_PX(p) case p: \
	__asm__ volatile(MXU3_WORD(NNR_W(3, 2 * (p), 4)) MXU3_WORD(NNR_W(3, 2 * (p) + 1, 1)) \
			 I2(ADDW_W(4, 4, 2), ADDW_W(1, 1, 12)) I2(FCVT_W(2, 4, 4), FCVT_W(2, 1, 1)) \
			 I2(FMULW_W(4, 4, 3), FMULW_W(1, 1, 13)) ::: "memory"); break;

void nna_vhead_drd_px(int p, float *f0, float *f1)
{
	switch (p) {
	VHEAD_DRD_PX(0) VHEAD_DRD_PX(1) VHEAD_DRD_PX(2) VHEAD_DRD_PX(3)
	VHEAD_DRD_PX(4) VHEAD_DRD_PX(5) VHEAD_DRD_PX(6) VHEAD_DRD_PX(7)
	default: return;
	}
	VSTORE(4, f0);
	VSTORE(1, f1);
}

void nna_vrq_px(const int32_t *S0, const int32_t *S1, uint8_t *q)
{
	VLOAD(4, S0);
	VLOAD(1, S1);
	__asm__ volatile(VRQ_SILU2 ::: "memory");
	{
		register void *_p __asm__("$9") = q;

		__asm__ volatile(MXU3_WORD(SAO_W(9, 0, 0, 23)) :: "r"(_p) : "memory");
	}
}
#endif

#ifdef ONNA_MXU3
/* bytes <-> words: word lane i of group g = byte 16g + i (unpack); byte 16g + i = word lane i's
 * low byte (pack); 0x80 = zero */
static const struct {
	uint8_t un[4][64], pk[4][64];
	int32_t p128[16], m128[16], zero[16], c255[16];
} vb __attribute__((aligned(64))) = {
#define U(g) { 16*g+0,128,128,128, 16*g+1,128,128,128, 16*g+2,128,128,128, 16*g+3,128,128,128, \
	16*g+4,128,128,128, 16*g+5,128,128,128, 16*g+6,128,128,128, 16*g+7,128,128,128, \
	16*g+8,128,128,128, 16*g+9,128,128,128, 16*g+10,128,128,128, 16*g+11,128,128,128, \
	16*g+12,128,128,128, 16*g+13,128,128,128, 16*g+14,128,128,128, 16*g+15,128,128,128 }
	{ U(0), U(1), U(2), U(3) },
#undef U
#define Z16 128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128
#define P16 0,4,8,12,16,20,24,28,32,36,40,44,48,52,56,60
	{ { P16, Z16, Z16, Z16 }, { Z16, P16, Z16, Z16 }, { Z16, Z16, P16, Z16 }, { Z16, Z16, Z16, P16 } },
#undef Z16
#undef P16
#define X16(v) { v, v, v, v, v, v, v, v, v, v, v, v, v, v, v, v }
	X16(128), X16(-128), X16(0), X16(255),
#undef X16
};

/* VPRs: 10..13 unpack, 14..17 pack, 18 +128, 19 -128, 20 0, 21 255; work 1..9, 22..27 */
static void vb_load(void)
{
	VLOAD(10, vb.un[0]); VLOAD(11, vb.un[1]); VLOAD(12, vb.un[2]); VLOAD(13, vb.un[3]);
	VLOAD(14, vb.pk[0]); VLOAD(15, vb.pk[1]); VLOAD(16, vb.pk[2]); VLOAD(17, vb.pk[3]);
	VLOAD(18, vb.p128); VLOAD(19, vb.m128); VLOAD(20, vb.zero); VLOAD(21, vb.c255);
}

/* group g of V1 -> rint((x - 128) * a) + 128, clamped, packed into V4 (V22 = a) */
#define VRS_G(g, d) \
	MXU3_WORD(GSHUFVB_W(2, 10 + (g), 1)) MXU3_WORD(ADDW_W(2, 2, 19)) MXU3_WORD(FCVT_W(2, 2, 2)) \
	MXU3_WORD(FMULW_W(2, 2, 22)) MXU3_WORD(FCVT_W(7, 2, 2)) MXU3_WORD(ADDW_W(2, 2, 18)) \
	MXU3_WORD(MAXSW_W(2, 2, 20)) MXU3_WORD(MINSW_W(2, 2, 21)) \
	MXU3_WORD(GSHUFVB_W(d, 14 + (g), 2)) MXU3_WORD(ORV_W(4, 4, d))

void nna_vrescale(const uint8_t *x, int n, float a, uint8_t *out)
{
	float av[16] __attribute__((aligned(64)));
	int i = 0;

	for (int k = 0; k < 16; k++)
		av[k] = a;
	vb_load();
	VLOAD(22, av);
	for (; i + 64 <= n; i += 64) {
		VLOAD(1, x + i);
		/* group 0 packs into V4 (ORed with itself), the others into V3, ORed into V4 */
		__asm__ volatile(VRS_G(0, 4) VRS_G(1, 3) VRS_G(2, 3) VRS_G(3, 3) ::: "memory");
		VSTORE(4, out + i);
	}
	if (i < n)
		nna_rescale(x + i, n - i, a, out + i);
}

/* group g of V1 (a) and V5 (b): num = 64 (wa a + wb b) + B; q = rint(num / T) corrected to the
 * floor with r = num - q T; + 128, clamped, packed into V4. V22 wa, V23 wb, V24 B, V25 T,
 * V26 1/T (float) */
#define VADD_G(g, d) \
	MXU3_WORD(GSHUFVB_W(2, 10 + (g), 1)) MXU3_WORD(GSHUFVB_W(6, 10 + (g), 5)) \
	MXU3_WORD(MULW_W(2, 2, 22)) MXU3_WORD(MULW_W(6, 6, 23)) MXU3_WORD(ADDW_W(2, 2, 6)) \
	MXU3_WORD(SLLIW_W(2, 2, 6)) MXU3_WORD(ADDW_W(2, 2, 24)) \
	MXU3_WORD(FCVT_W(2, 6, 2)) MXU3_WORD(FMULW_W(6, 6, 26)) MXU3_WORD(FCVT_W(7, 6, 6)) \
	MXU3_WORD(MULW_W(7, 6, 25)) MXU3_WORD(SUBW_W(7, 2, 7)) \
	MXU3_WORD(CLTSW_W(8, 7, 20)) MXU3_WORD(ADDW_W(6, 6, 8)) \
	MXU3_WORD(CLESW_W(8, 25, 7)) MXU3_WORD(SUBW_W(6, 6, 8)) \
	MXU3_WORD(ADDW_W(6, 6, 18)) MXU3_WORD(MAXSW_W(6, 6, 20)) MXU3_WORD(MINSW_W(6, 6, 21)) \
	MXU3_WORD(GSHUFVB_W(d, 14 + (g), 6)) MXU3_WORD(ORV_W(4, 4, d))

void nna_vadd(const uint8_t *a, const uint8_t *b, int n, int wa, int wb, int32_t B, int32_t T, uint8_t *out)
{
	int32_t k4[4][16] __attribute__((aligned(64)));
	float it[16] __attribute__((aligned(64)));
	int i = 0;

	for (int k = 0; k < 16; k++) {
		k4[0][k] = wa, k4[1][k] = wb, k4[2][k] = B, k4[3][k] = T;
		it[k] = 1.0f / (float)T;
	}
	vb_load();
	VLOAD(22, k4[0]); VLOAD(23, k4[1]); VLOAD(24, k4[2]); VLOAD(25, k4[3]); VLOAD(26, it);
	for (; i + 64 <= n; i += 64) {
		VLOAD(1, a + i);
		VLOAD(5, b + i);
		__asm__ volatile(VADD_G(0, 4) VADD_G(1, 3) VADD_G(2, 3) VADD_G(3, 3) ::: "memory");
		VSTORE(4, out + i);
	}
	if (i < n)
		nna_add_ref(a + i, b + i, n - i, wa, wb, B, T, out + i);
}
void nna_vrescale_px(const uint8_t *src, int sps, uint8_t *dst, int dps, int c, long npx, float a)
{
	float av[16] __attribute__((aligned(64)));

	for (int k = 0; k < 16; k++)
		av[k] = a;
	vb_load();
	VLOAD(22, av);
	for (long p = 0; p < npx; p++, src += sps, dst += dps)
		for (int i = 0; i < c; i += 64) {
			VLOAD(1, src + i);
			__asm__ volatile(VRS_G(0, 4) VRS_G(1, 3) VRS_G(2, 3) VRS_G(3, 3) ::: "memory");
			VSTORE(4, dst + i);
		}
}

void nna_vmerge16(const uint8_t *a, const uint8_t *b, long npx, uint8_t *out)
{
	/* per 2 pixels: a's bytes 0..15 and 32..47 stay, b's move up by 16 */
	static const uint8_t sel[2][64] __attribute__((aligned(64))) = {
		{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
		  128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128,
		  32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47,
		  128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128 },
		{ 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128,
		  0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
		  128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128,
		  32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47 },
	};

	VLOAD(10, sel[0]);
	VLOAD(11, sel[1]);
	for (long p = 0; p < npx; p += 2, a += 64, b += 64, out += 64) {
		VLOAD(1, a);
		VLOAD(2, b);
		__asm__ volatile(MXU3_WORD(GSHUFVB_W(3, 10, 1)) MXU3_WORD(GSHUFVB_W(4, 11, 2))
				 MXU3_WORD(ORV_W(3, 3, 4)) ::: "memory");
		VSTORE(3, out);
	}
}
#endif
