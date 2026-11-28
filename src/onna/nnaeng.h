/*
 * nnaeng: drive the T41 NNA's convolution engine for one conv layer.
 * Semantics: docs/nna-runtime.md (engine registers, W entries, F RAM, MAC, CPOC, NNDRD).
 * The engine is reached through a backend: the real MXU3 NN instructions on the camera
 * (be_mxu3.c), or a test oracle on the host.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: MIT
 */
#ifndef NNAENG_H
#define NNAENG_H
#include <stdint.h>
#include "nnacpu.h"

/* one conv layer for nna_conv: weights wq [ky][kx][oc][ic] int8 (raw - 128); the stem is the 6x6/2
 * on RGBA (ic 4), run as 6-row strips; rec keys nnaeng's per-record W caches */
#define NNA_MAXREC 1024		/* conv records (nna_node.rec) nnaeng caches weights for */

struct nna_node {
	int rec, oc, ic, kh, kw, stride, pad, padval;
	int stem;
	int dw;			/* depthwise (one input channel per output channel): wq is [ky][kx][oc] */
	int8_t *wq;
};

struct nna_be {
	void *ctx;
	void (*rwr)(void *ctx, unsigned reg, uint32_t v);		/* NNRWR */
	void (*dwr)(void *ctx, unsigned imm, const uint8_t v[64]);	/* NNDWR */
	void (*drd)(void *ctx, unsigned imm, uint8_t v[64]);		/* NNDRD */
	void (*mac)(void *ctx, unsigned imm, uint32_t word);		/* NNMAC */
	void (*cmd)(void *ctx, unsigned imm);				/* NNCMD */
	int direct;	/* 1: this is the CPU's own MXU3 (NN instructions inline, read-backs into VPRs) */
};

/* requant fused into the read-back: each NNDRD's 16 sums become output bytes at once */
struct nna_rq {
	const int32_t *B;
	const float *Fl;
	const uint8_t *lut;	/* NNA_SILU_LUT bytes */
	uint8_t *q;		/* OH x OW x oc output codes (64-byte aligned for the MXU3 path) */
	const struct nna_vrq *vrq;	/* per 32-oc block, for the MXU3 SiLU requant; or NULL (LUT) */
	float *f;		/* a head: OH x OW x oc floats f32(S + B) * Fl instead of q */
	int ps;			/* bytes per output pixel in q (0: oc). 32 for oc 16: the 16-channel
				 * layout with 32-byte pixels, channels 16..31 zero */
	const int32_t *bt;	/* QSUM: per 32-oc block B[32] then T[32] (BT RAM rows). The NNA
				 * requantizes, q = clamp(floor((4 S + B) / T) + 128), and the read-backs are
				 * the output bytes (4 NNDRD 0x20 per tile instead of 16) */
	const uint8_t *post;	/* QSUM: a 256-byte table applied to every output byte, or NULL */
};

/* one conv on the NNA: in = HWC uint8 codes (C channels, ps bytes per pixel: C, or 32 for C = 16). With rq == NULL the int32 sums go to
 * S (OH x OW x oc); with rq (oc a multiple of 16) they are requantized straight into rq->q.
 * 0 on success, -1 if the layer does not fit this tiling (message on stderr). */
int nna_conv(const struct nna_be *be, const uint8_t *in, int H, int W, int C, int ps,
	     const struct nna_node *nd, int32_t *S, const struct nna_rq *rq, int *oh, int *ow);

#endif
