/*
 * nnacpu: the CPU half of a venus-compatible T41 NNA runtime.
 *
 * Bit-exact with venus 0.1.2.5 (see docs/nna-runtime.md). Feature maps are HWC uint8 codes,
 * zero point 128. Build with -ffp-contract=off: float32 rounding order matters.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: MIT
 */
#ifndef NNACPU_H
#define NNACPU_H
#include <stdint.h>

/* the per-layer SiLU table: 32 segments of 1024 t-units, de-interleaved */
struct nna_silu {
	int16_t dy[32];
	int16_t y[32];
};

/* venus stores it interleaved: (dy_i, dy_16+i) x16, then (y_i, y_16+i) x16 */
void nna_silu_load(struct nna_silu *t, const int16_t raw[64]);

/* q = clip(rint(PWL(rint(f32(S + B) * Fl)) / 128)), per pixel, per channel */
void nna_requant_silu(const int32_t *S, int npx, int noc, const int32_t *B, const float *Fl,
		      const struct nna_silu *t, uint8_t *q);

/* the same as nna_requant_silu through a per-layer table of every output byte by t (built once,
 * NNA_SILU_LUT bytes): q = lut[clip(rint(f32(S + B) * Fl), -1, 32767) + 1] */
#define NNA_SILU_LUT 32769
void nna_silu_lut(const struct nna_silu *t, uint8_t lut[NNA_SILU_LUT]);
void nna_requant_lut(const int32_t *S, int npx, int noc, const int32_t *B, const float *Fl,
		     const uint8_t lut[NNA_SILU_LUT], uint8_t *q);

/* t = rint(f32(S + B) * Fl) for nvec vectors of 16 sums sharing one 16-channel block's B and Fl
 * (S and t: nvec x 16 int32). MXU3 when built with ONNA_MXU3, else the scalar float32 path. */
void nna_requant_t(const int32_t *S, int nvec, const int32_t *B16, const float *Fl16, int32_t *t);

/* The whole SiLU requant in MXU3 for one pixel's 32 channels (two 16-channel halves with their own
 * B and Fl): q[32] = lut[clip(rint(f32(S + B) * Fl), -1, 32767) + 1], bit-identical to
 * nna_requant_silu. nna_vrq_load() puts the block's numbers and constants into VPRs 1..30 (they stay
 * there: no other code uses those); then nna_vrq_px() per pixel. q must be 32-byte aligned. */
struct nna_vrq {
	int32_t B[2][16];
	float Fl[2][16];
	struct nna_silu tb;		/* dy[32], y[32]: one 64-byte table each */
	int wide;			/* 16-channel block written as 32-byte pixels: half 1 packs at byte 32 */
} __attribute__((aligned(64)));
#ifdef ONNA_MXU3
void nna_vrq_load(const struct nna_vrq *v);
/* the same in two parts: what a conv's blocks share (tables, constants, packs), and one block's B
 * and Fl (VPRs 2, 3, 12, 13) */
void nna_vrq_load_const(const struct nna_vrq *v);
void nna_vrq_load_bfl(const struct nna_vrq *v);
void nna_vrq_px(const int32_t *S0, const int32_t *S1, uint8_t *q);
/* the same straight from the NNA: NNDRD 2p and 2p + 1 (pixel p's two channel halves) into VPR 4,
 * no memory round trip. q == NULL: read both and drop them (pixels outside the output) */
void nna_vrq_drd_px(int p, uint8_t *q);
/* a 16-channel block (B, Fl of half 1 = half 0): pixels p and p + 1 (p even) at once, from NNDRD
 * 2p and 2p + 2 (2p + 1, 2p + 3 read and dropped); q[0..15] = pixel p, then pixel p + 1 at q[16]
 * (or at q[32] with wide: the 32-byte-pixel layout, bytes 16..31 and 48..63 zero) */
void nna_vrq_drd_pair16(int p, uint8_t *q, int nbytes);	/* stores nbytes: 32 or 64 */
/* head: f32(S + B) * Fl of pixel p's two halves (NNDRD 2p, 2p + 1) into f0[16], f1[16] (64-byte
 * aligned), with B and Fl from nna_vrq_load */
void nna_vhead_drd_px(int p, float *f0, float *f1);
#endif

#ifdef ONNA_MXU3
/* MXU3 forms of nna_rescale and nna_add_ref, bit-identical (vaddtest: every input). 64 bytes per
 * step, pointers 32-byte aligned; n any (the tail runs the scalar code) */
void nna_vrescale(const uint8_t *x, int n, float a, uint8_t *out);
void nna_vadd(const uint8_t *a, const uint8_t *b, int n, int wa, int wb, int32_t B, int32_t T, uint8_t *out);
/* nna_vadd, then each output byte through a 256-byte table (post, 64-byte aligned) in the same pass:
 * out = post[add] (a per-channel 1x1 folded into the add) */
void nna_vadd_post(const uint8_t *a, const uint8_t *b, int n, int wa, int wb, int32_t B, int32_t T,
		   const uint8_t *post, uint8_t *out);
/* nna_vrescale over npx pixels of c bytes (c a multiple of 64): src stride sps, dst stride dps */
void nna_vrescale_px(const uint8_t *src, int sps, uint8_t *dst, int dps, int c, long npx, float a);
/* two 16-channel tensors in 32-byte pixels -> one dense 32-channel tensor [a | b], npx even */
void nna_vmerge16(const uint8_t *a, const uint8_t *b, long npx, uint8_t *out);
#endif

/* head: out = f32(S + B) * Fl */
void nna_head(const int32_t *S, int npx, int noc, const int32_t *B, const float *Fl, float *out);

/* concat input rescale: out = clip(rint((x - 128) * a) + 128) */
void nna_rescale(const uint8_t *x, int n, float a, uint8_t *out);

/* residual add as the NNA's QSUM computes it (CPU reference):
 * out = clip(floor((64 * (wa*a + wb*b) + B) / T) + 128) */
void nna_add_ref(const uint8_t *a, const uint8_t *b, int n, int wa, int wb, int32_t B, int32_t T,
		 uint8_t *out);

/* the same through a table of every (a, b) pair, built once: out = lut[a << 8 | b] */
void nna_add_lut(int wa, int wb, int32_t B, int32_t T, uint8_t lut[65536]);
void nna_add_apply(const uint8_t *a, const uint8_t *b, int n, const uint8_t lut[65536], uint8_t *out);

/* 5x5 max pool, stride 1, pad 2 (padding never wins) */
void nna_maxpool5(const uint8_t *x, int h, int w, int c, uint8_t *out);

/* nearest-neighbour x2 upsample */
void nna_upsample2(const uint8_t *x, int h, int w, int c, uint8_t *out);

#endif
