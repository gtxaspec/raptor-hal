/*
 * nnabin: read a venus (magik-transform-tools 2.0.4) T41 .bin and onna's text graph.
 * Every structure used is validated; anything unexpected is an error, never a guess.
 * Format: docs/nna-runtime.md ("Reading the .bin without a trace").
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: MIT
 */
#ifndef NNABIN_H
#define NNABIN_H
#include <stdint.h>
#include "nnacpu.h"

struct nna_conv_rec {		/* one conv's record, in .bin file order */
	const uint8_t *w;	/* nent x 256-byte W entries */
	int nent, nob, kh, kw;	/* as stored (the stem says 1x1) */
	int32_t B[256];		/* per oc (copied: the .bin is unaligned) */
	float Fl[256];
	struct nna_silu silu;
	int has_silu;		/* 0 for the heads: they have no SiLU table */
	uint8_t *lut;		/* NNA_SILU_LUT bytes, built at load */
	long after;		/* file offset after the two table copies */
};

struct nna_add_rec {
	int wa, wb;
	int32_t B, T;
};

struct nna_bin {
	uint8_t *data;
	long len;
	int nconv, nadd, nhcat;
	struct nna_conv_rec *conv;
	struct nna_add_rec add[16];
	float hcat[8][2];	/* 2-input header concat records: a_i = s_i * (1/s_out) */
};

enum nna_op { OP_CONV, OP_ADD, OP_CONCAT, OP_MAXPOOL5, OP_UPSAMPLE2 };

struct nna_node {
	enum nna_op op;
	int out, in[4], nin;
	/* conv */
	int rec, oc, ic, kh, kw, stride, pad, padval, stem, head, nch, ch0[32], chk[32];
	int dw;			/* depthwise (one input channel per output channel): wq is [ky][kx][oc] */
	int8_t *wq;		/* [ky][kx][oc][ic] */
	/* add */
	int arec;
	/* concat: per input, 1 = none, else a */
	float a[4];
};

struct nna_graph {
	int ntensor, nnode, nout, out[4];
	int in_h, in_w, in_c;
	struct nna_node *node;
};

/* 0 on success, else -1 with a message on stderr */
int nna_bin_load(struct nna_bin *b, const char *path);
int nna_graph_load(struct nna_graph *g, const char *path, const struct nna_bin *b);

#endif
