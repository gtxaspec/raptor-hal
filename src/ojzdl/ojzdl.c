/* ojzdl: open runtime for JZDL v5 models. Plain C reference path (step 1).
 *
 * Every rule here is from ../../jzdl-format.md ("Layer types", "Arithmetic"),
 * found by designed experiments against libjzdl 5.4.0 under QEMU and checked
 * on the v3. Section names are cited as [fmt: <section>].
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: MIT
 */
#include "ojzdl.h"
#include "ojzdl_mxu2.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* [fmt: File layout] header words before the per-layer records */
#define HDR_WORDS 8
#define HDR_PARAM_OFS 24
/* versions 4 (e.g. Ingenic's yolov5 sample) and 5 (persondet) [fmt: File layout] */

/* [fmt: Layer types] */
enum {
	T_INPUT = 16, T_SPLIT = 33, T_CONV_FIRST = 46, T_HEAD = 51, T_PW = 53,
	T_UPSAMPLE = 69, T_CONCAT = 71, T_DW = 73, T_PW_POOL = 74, T_POOL = 75,
	T_PACK = 76, T_ELTWISE = 78,
};

/* [fmt: Arithmetic] MXU2 weight blocks: 16 outputs x 2 inputs = 32 bytes */
#define GRP 16
#define BLK 32
/* [fmt: Weight sizes] v5 pads 9 taps to 10 */
#define TAPS_PADDED 10
/* [fmt: Arithmetic] Q of the requant multiply */
#define Q_FIRST 31
#define Q_OTHER 15
/* [fmt: Arithmetic] input = rgb - 128, 8-bit */
#define IMG_BITS 8
#define IMG_OFFSET 128

#define MAX_IO 4 /* most bottoms/tops of any supported type (concat: 2) */

typedef struct {
	int h, w, c;
	uint8_t bits;     /* activation width; 0 = float blob */
	int8_t *i8;       /* points into own storage, or aliases another blob (split) */
	float *f32;
	size_t n;         /* element count */
	int owner;        /* blob index that owns the storage */
	int last_use;     /* last layer index reading it */
} blob_t;

typedef struct {
	int16_t type;
	uint8_t nb, nt;
	int32_t bottom[MAX_IO], top[MAX_IO];
	const int32_t *p;      /* params, into ojzdl_net.params */
	int np;
	/* unpacked weights (natural layout) and requant arrays */
	int8_t *w;             /* 46: [co][kh][kw][ci]; 73: [c][9]; 53/74/51: [co][ci] */
	const int8_t *wp;      /* MXU2 layout (v5 blocks): the blob for v5, wp_own for v4 */
	int8_t *wp_own;
	int wper;              /* first conv: bytes of wp per 16 outputs */
	const uint8_t *blob[4];/* raw data blobs, into the model buffer */
	uint8_t simd;          /* MXU2 kernel applies to this layer */
	int32_t *q;            /* conv: bias[co], mult[co], shift[co] decoded once */
	int32_t *qtab;         /* first conv MXU2: even/odd-arranged q [ojzdl_mxu2.h] */
	size_t scratch;        /* scratch bytes this layer needs */
	uint64_t ns;
} layer_t;

struct ojzdl_net {
	uint8_t *buf;
	size_t len;
	int in_blob, out_blob, in_h, in_w, in_c, version;
	int nlayers, nblobs;
	layer_t *L;
	blob_t *B;
	int32_t *params;        /* all layers' params, decoded once */
	int timing, keep_all, simd;
	uint8_t *arena;          /* every blob, offsets planned from live intervals */
	size_t arena_sz;
	int8_t *scratch;         /* pads and temps; size = max over layers */
	size_t scratch_sz;
	int ran;
};

/* ------------------------------------------------------------------ helpers */

static int32_t rd32(const uint8_t *p)
{
	return (int32_t)((uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24);
}

static int16_t rd16(const uint8_t *p)
{
	return (int16_t)((uint16_t)p[0] | (uint16_t)p[1] << 8);
}

static float rdf(const uint8_t *p)
{
	uint32_t u = (uint32_t)rd32(p);
	float f;
	memcpy(&f, &u, 4);
	return f;
}

/* [fmt: Arithmetic] rsr = add 2^(s-1), then arithmetic shift right; s <= 0 is a no-op */
static inline int64_t rsr64(int64_t v, int s)
{
	return s > 0 ? (v + ((int64_t)1 << (s - 1))) >> s : v;
}

static inline int32_t rsr32(int32_t v, int s)
{
	return s > 0 ? (v + ((int32_t)1 << (s - 1))) >> s : v;
}

static inline int8_t clamp_out(int64_t v, int bits)
{
	int64_t hi = ((int64_t)1 << bits) - 1;
	if (v < 0)
		v = 0;
	if (v > hi)
		v = hi;
	return (int8_t)(v - ((int64_t)1 << (bits - 1)));
}

/* param words per (type, version) [fmt: Layer types]; v4 lacks the trailing v5 flag */
static int param_count(int type, int version)
{
	int v5 = version >= 5;
	switch (type) {
	case T_INPUT: return 3;
	case T_SPLIT: case T_CONCAT: return 1;
	case T_CONV_FIRST: return v5 ? 19 : 18;
	case T_HEAD: return v5 ? 17 : 16;
	case T_PW: case T_DW: return v5 ? 18 : 17;
	case T_PW_POOL: return v5 ? 18 : -1;
	case T_UPSAMPLE: return 5;
	case T_POOL: return v5 ? 12 : -1;
	case T_PACK: return 0;
	case T_ELTWISE: return v5 ? -1 : 2;   /* only seen, and verified, in v4 */
	default: return -1;
	}
}

/* data blob byte sizes in load order [fmt: Layer types] */
static int blob_sizes(const layer_t *l, size_t sz[4])
{
	int32_t c = l->p ? l->p[0] : 0;
	switch (l->type) {
	case T_CONV_FIRST:
		sz[0] = (size_t)l->p[12]; sz[1] = sz[2] = sz[3] = 4 * (size_t)c;
		return 4;
	case T_HEAD:
		sz[0] = (size_t)l->p[12]; sz[1] = sz[2] = 4 * (size_t)c;
		return 3;
	case T_PW: case T_DW: case T_PW_POOL:
		/* 4 x int16[c]; the 4th is unused (always 0) [fmt: Arithmetic] */
		sz[0] = (size_t)l->p[12]; sz[1] = sz[2] = sz[3] = 2 * (size_t)c;
		return 4; /* the 5th blob (int16[c]) is skipped below */
	default:
		return 0;
	}
}

static size_t blob_skip(const layer_t *l)
{
	if (l->type == T_PW || l->type == T_DW || l->type == T_PW_POOL)
		return 2 * (size_t)l->p[0];
	return 0;
}

/* ------------------------------------------------------------------ loading */

#define FAIL(msg) do { if (err) *err = (msg); goto fail; } while (0)

static int infer_shapes(ojzdl_net *n, const char **err);
static int unpack_weights(ojzdl_net *n, const char **err);
static int plan_memory(ojzdl_net *n);

#define ALIGN16(x) (((x) + 15) & ~(size_t)15)

/* bias/mult/shift as int32 [fmt: Arithmetic]: int32 for 46, int16 for 53/73/74 */
static int decode_requant(ojzdl_net *n)
{
	int i, o;
	for (i = 0; i < n->nlayers; i++) {
		layer_t *l = &n->L[i];
		int co = l->p ? l->p[0] : 0, k;
		if (l->type != T_CONV_FIRST && l->type != T_DW && l->type != T_PW && l->type != T_PW_POOL)
			continue;
		l->q = malloc(3 * (size_t)co * sizeof(int32_t));
		if (!l->q)
			return -1;
		for (k = 0; k < 3; k++)
			for (o = 0; o < co; o++)
				l->q[k * co + o] = l->type == T_CONV_FIRST ? rd32(l->blob[k + 1] + 4 * o)
				                                           : rd16(l->blob[k + 1] + 2 * o);
		/* [fmt: Arithmetic] libjzdl takes an int16 layer's shift modulo 16 (a shift of
		 * 16 acts as 0; verified 2026-11-25): normalise once for the C and MXU2 paths */
		if (l->type != T_CONV_FIRST)
			for (o = 0; o < co; o++)
				l->q[2 * co + o] &= 15;
		if (l->type == T_CONV_FIRST && l->simd) {
			/* even/odd arrangement for the vector requant; also its limits:
			 * acc + bias must not wrap int32, SRAR.w shifts mod 32 [mxu2.h] */
			int g, v, j;
			l->qtab = malloc((size_t)(co / GRP) * 48 * sizeof(int32_t));
			if (!l->qtab)
				return -1;
			for (o = 0; o < co; o++)
				if (l->q[o] > INT32_MAX - 32768 || l->q[o] < INT32_MIN + 32768 ||
				    l->q[2 * co + o] < 0 || l->q[2 * co + o] > 31)
					l->simd = 0;
			for (g = 0; g < co / GRP; g++)
				for (k = 0; k < 3; k++)          /* bias, mult, shift */
					for (v = 0; v < 4; v++)  /* Elo, Olo, Ehi, Ohi */
						for (j = 0; j < 4; j++)
							l->qtab[g * 48 + (k * 4 + v) * 4 + j] =
								l->q[k * co + g * GRP + (v / 2) * 8 + 2 * j + (v % 2)];
		}
	}
	return 0;
}

/* Which layers fit the MXU2 kernels' constraints (ojzdl_mxu2.h). */
static void mark_simd(ojzdl_net *n)
{
	int i, o;
	for (i = 0; i < n->nlayers; i++) {
		layer_t *l = &n->L[i];
		int ok;
		const int32_t *P = l->p;
		const int8_t *W = l->wp ? l->wp : (const int8_t *)l->blob[0];
		if (l->type == T_CONV_FIRST) {
			/* 3x3 x cin 3 rows of 9 bytes; slot 9 reads the next pixel, so its
			 * weight must be 0; int16 accumulators must not wrap:
			 * sum|w| * 128 <= 32767 per output [fmt: Arithmetic] */
			int per = l->wper, ky, k;
			ok = l->wp && P[1] == 3 && P[2] == 3 && n->B[l->bottom[0]].c == 3;
			for (o = 0; ok && o < P[0]; o++) {
				const int8_t *g = W + (o / GRP) * per;
				int32_t sum = 0;
				for (ky = 0; ky < 3; ky++) {
					int s9 = ky * TAPS_PADDED + 9;
					if (g[(s9 / 2) * BLK + (o % GRP) * 2 + s9 % 2])
						ok = 0;
				}
				for (k = 0; k < 9 * 3; k++)
					sum += abs(l->w[(size_t)o * 27 + k]);
				if (sum * 128 > 32767)
					ok = 0;
			}
			l->simd = (uint8_t)ok;
			continue;
		}
		if (l->type == T_HEAD) {
			/* DOTPS.h pair sums must fit int16: 2 * max|w| * 2^(bits_in-1) <= 32767 */
			int maxw = 0, ci = n->B[l->bottom[0]].c;
			size_t k;
			for (k = 0; k < (size_t)P[12]; k++)
				if (abs(W[k]) > maxw)
					maxw = abs(W[k]);
			l->simd = (uint8_t)(ci % 16 == 0 && 2L * maxw * (1L << (P[13] - 1)) <= 32767);
			continue;
		}
		if (l->type == T_DW) {
			/* equal strides 1 or 2, pads 0/1 (checked at load); pad slot (pair 4,
			 * slot 0) weight 0 */
			ok = l->wp && P[5] == P[6];
			for (o = 0; ok && o < P[0]; o++)
				if (W[(o / GRP) * GRP * TAPS_PADDED + 4 * BLK + (o % GRP) * 2])
					ok = 0;
		} else if (l->type == T_PW || l->type == T_PW_POOL) {
			ok = n->B[l->bottom[0]].c % 16 == 0;
		} else {
			continue;
		}
		/* SRAR.h uses the shift count mod 16 [mxu2.h] */
		ok = ok && P[0] % 16 == 0;
		for (o = 0; ok && o < l->p[0]; o++) {
			int16_t sh = rd16(l->blob[3] + 2 * o);
			ok = sh >= 0 && sh <= 15;
		}
		l->simd = (uint8_t)ok;
	}
}

ojzdl_net *ojzdl_load(const uint8_t *src, size_t len, const char **err)
{
	ojzdl_net *n = calloc(1, sizeof(*n));
	size_t pos, i, np_used = 0;
	int32_t pbytes, w;

	if (!n)
		FAIL("out of memory");
	if (len < HDR_WORDS * 4)
		FAIL("file too short");
	n->buf = malloc(len);
	if (!n->buf)
		FAIL("out of memory");
	memcpy(n->buf, src, len);
	n->len = len;

	pbytes = rd32(n->buf);
	n->in_blob = rd32(n->buf + 4);
	n->out_blob = rd32(n->buf + 8);
	n->in_h = rd32(n->buf + 12);
	n->in_w = rd32(n->buf + 16);
	n->in_c = rd32(n->buf + 20);
	w = rd32(n->buf + 24);
	n->nlayers = (int)((uint32_t)w & 0xffffff);
	n->version = (int)((uint32_t)w >> 24);
	n->nblobs = (int)((uint32_t)rd32(n->buf + 28) & 0xffffff);
	if (n->version != 4 && n->version != 5)
		FAIL("only version 4 and 5 models are supported");
	if (pbytes < 8 || (size_t)pbytes + HDR_PARAM_OFS > len)
		FAIL("bad param_bytes");
	if (n->in_h <= 0 || n->in_w <= 0 || n->in_c != 3 || n->in_h > 4096 || n->in_w > 4096)
		FAIL("bad input shape");
	if (n->nlayers <= 0 || n->nblobs <= 0 || n->nblobs > 65536 || n->nlayers > 65536)
		FAIL("bad layer/blob count");
	if (n->in_blob < 0 || n->in_blob >= n->nblobs || n->out_blob < 0 || n->out_blob >= n->nblobs)
		FAIL("bad input/output blob");

	n->L = calloc((size_t)n->nlayers, sizeof(layer_t));
	n->B = calloc((size_t)n->nblobs, sizeof(blob_t));
	n->params = malloc((size_t)pbytes); /* upper bound on the param words */
	if (!n->L || !n->B || !n->params)
		FAIL("out of memory");

	pos = HDR_WORDS * 4;
	for (i = 0; i < (size_t)n->nlayers; i++) {
		layer_t *l = &n->L[i];
		int32_t t, nb, nt, k;
		if (pos + 12 > HDR_PARAM_OFS + (size_t)pbytes)
			FAIL("param section truncated");
		t = rd32(n->buf + pos); nb = rd32(n->buf + pos + 4); nt = rd32(n->buf + pos + 8);
		pos += 12;
		if (param_count(t, n->version) < 0)
			FAIL("unsupported layer type for this version");
		if (nb < 0 || nb > MAX_IO || nt < 0 || nt > MAX_IO)
			FAIL("bad bottom/top count");
		l->type = (int16_t)t; l->nb = (uint8_t)nb; l->nt = (uint8_t)nt;
		l->np = param_count(t, n->version);
		if (pos + 4 * ((size_t)nb + (size_t)nt + (size_t)l->np) > HDR_PARAM_OFS + (size_t)pbytes)
			FAIL("param section truncated");
		for (k = 0; k < nb; k++, pos += 4) {
			l->bottom[k] = rd32(n->buf + pos);
			if (l->bottom[k] < 0 || l->bottom[k] >= n->nblobs)
				FAIL("bottom blob out of range");
		}
		for (k = 0; k < nt; k++, pos += 4) {
			l->top[k] = rd32(n->buf + pos);
			if (l->top[k] < 0 || l->top[k] >= n->nblobs)
				FAIL("top blob out of range");
		}
		{
			int32_t *dst = n->params + np_used;
			for (k = 0; k < l->np; k++, pos += 4)
				dst[k] = rd32(n->buf + pos);
			l->p = dst;
			np_used += (size_t)l->np;
		}
	}
	if (pos != HDR_PARAM_OFS + (size_t)pbytes)
		FAIL("param section length mismatch");

	for (i = 0; i < (size_t)n->nlayers; i++) {
		layer_t *l = &n->L[i];
		size_t sz[4];
		int nbl = blob_sizes(l, sz), k;
		for (k = 0; k < nbl; k++) {
			if (sz[k] > len - pos)
				FAIL("model data truncated");
			l->blob[k] = n->buf + pos;
			pos += sz[k];
		}
		if (blob_skip(l) > len - pos)
			FAIL("model data truncated");
		pos += blob_skip(l);
	}
	if (pos != len)
		FAIL("trailing bytes after model data");

	if (infer_shapes(n, err) || unpack_weights(n, err))
		goto fail;
	mark_simd(n);
	n->simd = OJZDL_HAVE_MXU2;
	if (decode_requant(n) || plan_memory(n)) {
		if (err) *err = "out of memory";
		goto fail;
	}
	return n;
fail:
	ojzdl_free(n);
	return NULL;
}

ojzdl_net *ojzdl_load_file(const char *path, const char **err)
{
	FILE *f = fopen(path, "rb");
	uint8_t *b = NULL;
	long sz;
	ojzdl_net *n = NULL;

	if (!f) {
		if (err) *err = "cannot open model";
		return NULL;
	}
	if (fseek(f, 0, SEEK_END) || (sz = ftell(f)) <= 0 || fseek(f, 0, SEEK_SET)) {
		if (err) *err = "cannot size model";
		fclose(f);
		return NULL;
	}
	b = malloc((size_t)sz);
	if (b && fread(b, 1, (size_t)sz, f) == (size_t)sz)
		n = ojzdl_load(b, (size_t)sz, err);
	else if (err)
		*err = b ? "short read" : "out of memory";
	free(b);
	fclose(f);
	return n;
}

void ojzdl_free(ojzdl_net *n)
{
	int i;
	if (!n)
		return;
	if (n->L)
		for (i = 0; i < n->nlayers; i++) {
			free(n->L[i].w);
			free(n->L[i].q);
			free(n->L[i].qtab);
			free(n->L[i].wp_own);
		}
	free(n->arena);
	free(n->scratch);
	free(n->B);
	free(n->L);
	free(n->params);
	free(n->buf);
	free(n);
}

/* Walk the graph once: shapes, bits, last use. Validates every layer's params. */
static int infer_shapes(ojzdl_net *n, const char **err)
{
	int i, k;
	char *known = calloc((size_t)n->nblobs, 1);
	if (!known)
		FAIL("out of memory");
	for (i = 0; i < n->nblobs; i++) {
		n->B[i].owner = i;
		n->B[i].last_use = -1;
	}
	n->B[n->in_blob] = (blob_t){ n->in_h, n->in_w, n->in_c, IMG_BITS, NULL, NULL,
		(size_t)n->in_h * n->in_w * n->in_c, n->in_blob, -1 };
	known[n->in_blob] = 1;

	for (i = 0; i < n->nlayers; i++) {
		layer_t *l = &n->L[i];
		const int32_t *P = l->p;
		blob_t *x, *y;
		if (l->type == T_INPUT)
			continue;
		if (l->nb < 1 || l->nt < 1)
			FAIL("layer without bottom/top");
		for (k = 0; k < l->nb; k++) {
			if (!known[l->bottom[k]])
				FAIL("layer reads a blob before it is produced");
			n->B[l->bottom[k]].last_use = i;
		}
		for (k = 0; k < l->nt; k++)
			if (known[l->top[k]])
				FAIL("blob produced twice");
		x = &n->B[l->bottom[0]];
		y = &n->B[l->top[0]];
		if (x->bits == 0 && l->type != T_PACK)
			FAIL("float blob fed to an int8 layer");
		*y = (blob_t){ 0, 0, 0, 0, NULL, NULL, 0, l->top[0], -1 };

		switch (l->type) {
		case T_CONV_FIRST: case T_DW: {
			int kh = P[1], kw = P[2], sh = P[5], sw = P[6];
			int pt = P[8], pb = P[9], pl = P[10], pr = P[11];
			int bin = l->type == T_CONV_FIRST ? P[15] : P[14];
			int bout = l->type == T_CONV_FIRST ? P[16] : P[15];
			if (kh != 3 || kw != 3 || P[3] != 1 || P[4] != 1 || sh < 1 || sw < 1 || sh > 2 || sw > 2)
				FAIL("conv: only 3x3, dilation 1, stride 1/2");
			if (pt < 0 || pb < 0 || pl < 0 || pr < 0 || pt > 1 || pb > 1 || pl > 1 || pr > 1)
				FAIL("conv: pad must be 0/1");
			if (bin != x->bits || bout < 2 || bout > 8)
				FAIL("conv: bits mismatch");
			if (P[0] <= 0 || P[0] % GRP)
				FAIL("conv: num_output must be a multiple of 16");
			if (l->type == T_CONV_FIRST) {
				if (P[14] < 1 || P[14] > 8)
					FAIL("first conv: weight bits");
				if (n->version >= 5) {
					/* tap s = ky*10 + kx*cin + ci must fit a 10-wide row [fmt: Weight layouts] */
					if (x->c * kw > TAPS_PADDED)
						FAIL("first conv: kw*cin > 10");
					if (P[12] % (P[0] / GRP) || (size_t)P[12] / (size_t)(P[0] / GRP) < (size_t)((kh * TAPS_PADDED + 1) / 2) * BLK)
						FAIL("first conv: weight_bytes");
				} else if ((size_t)P[12] != (size_t)P[0] * kh * kw * x->c) {
					FAIL("first conv: weight_bytes != cout*kh*kw*cin");
				}
			} else {
				if (P[0] != x->c || P[12] != P[0] * (n->version >= 5 ? TAPS_PADDED : 9))
					FAIL("depthwise: channel/weight size mismatch");
			}
			y->h = (x->h + pt + pb - kh) / sh + 1;
			y->w = (x->w + pl + pr - kw) / sw + 1;
			y->c = P[0];
			y->bits = (uint8_t)bout;
			break;
		}
		case T_PW: case T_PW_POOL: case T_HEAD:
			if (P[2] != 1 || P[3] != 1)
				FAIL("1x1: kernel must be 1x1");
			if (P[0] <= 0 || (size_t)P[12] != (size_t)P[0] * (size_t)x->c)
				FAIL("1x1: weight_bytes != cin*cout");
			if (l->type == T_HEAD) {
				if (P[13] != x->bits)
					FAIL("head: bits mismatch");
				y->h = x->h; y->w = x->w; y->c = P[0]; y->bits = 0;
				break;
			}
			if (P[0] % GRP || x->c % 2 || P[14] != x->bits || P[15] < 2 || P[15] > 8)
				FAIL("1x1: cout%16, cin%2 or bits");
			y->h = x->h; y->w = x->w; y->c = P[0]; y->bits = (uint8_t)P[15];
			if (l->type == T_PW_POOL) {
				y->h /= 2;
				y->w /= 2;
			}
			break;
		case T_SPLIT:
			for (k = 0; k < l->nt; k++) {
				n->B[l->top[k]] = *x;
				n->B[l->top[k]].owner = x->owner;
				n->B[l->top[k]].last_use = -1;
				known[l->top[k]] = 1;
			}
			continue;
		case T_POOL:
			if (P[1] != 2 || P[2] != 2 || P[3] != 2 || P[4] != 2)
				FAIL("pool: only 2x2/2");
			y->h = x->h / 2; y->w = x->w / 2; y->c = x->c; y->bits = x->bits;
			break;
		case T_UPSAMPLE:
			/* [fmt: Layer types] verified: mode 4 + f4 0 = zero insertion (persondet);
			 * modes 4 and 5 + f4 1 = nearest neighbour (yolov5). Others refused. */
			if (!((P[1] == 4 && P[4] == 0) || ((P[1] == 4 || P[1] == 5) && P[4] == 1)) ||
			    P[2] < 1 || P[3] < 1 || P[2] > 4 || P[3] > 4)
				FAIL("upsample: unverified mode");
			y->h = x->h * P[2]; y->w = x->w * P[3]; y->c = x->c; y->bits = x->bits;
			break;
		case T_ELTWISE: {
			/* [fmt: Layer types] verified for equal input widths: mean, round half even */
			const blob_t *b = &n->B[l->bottom[1]];
			if (l->nb != 2 || b->h != x->h || b->w != x->w || b->c != x->c || b->bits != x->bits ||
			    P[0] < 2 || P[0] > 8)
				FAIL("eltwise: needs two equal-shape, equal-width inputs");
			y->h = x->h; y->w = x->w; y->c = x->c; y->bits = (uint8_t)P[0];
			break;
		}
		case T_CONCAT:
			y->h = x->h; y->w = x->w; y->bits = x->bits;
			for (k = 0; k < l->nb; k++) {
				blob_t *b = &n->B[l->bottom[k]];
				if (b->h != x->h || b->w != x->w || b->bits != x->bits)
					FAIL("concat: shape/bits mismatch");
				y->c += b->c;
			}
			break;
		case T_PACK: {
			size_t tot = 1;
			for (k = 0; k < l->nb; k++) {
				blob_t *b = &n->B[l->bottom[k]];
				if (b->bits != 0)
					FAIL("pack: input is not a float head");
				tot += b->n + 3;
			}
			y->h = 1; y->w = 1; y->c = (int)tot; y->bits = 0;
			break;
		}
		default:
			FAIL("unsupported layer type");
		}
		if (l->nt != 1)
			FAIL("only split has more than one top");
		y->n = (size_t)y->h * y->w * y->c;
		if (y->h <= 0 || y->w <= 0 || y->c <= 0)
			FAIL("empty output shape");
		known[l->top[0]] = 1;
	}
	if (!known[n->out_blob] || n->B[n->out_blob].bits != 0)
		FAIL("output blob is not a float blob");
	/* a split alias keeps its owner alive: fold last use into the owner */
	for (i = 0; i < n->nblobs; i++) {
		blob_t *o = &n->B[n->B[i].owner];
		if (n->B[i].last_use > o->last_use)
			o->last_use = n->B[i].last_use;
	}
	free(known);
	return 0;
fail:
	free(known);
	return -1;
}

/* Weights to natural layout for the C path, and to the MXU2 (v5 block) layout
 * for the kernels [fmt: Weight layouts]. v5 stores blocks: unpack them, and the
 * kernels read the blob. v4 stores plain arrays (1x1 [co][ci], depthwise
 * [tap][c], first conv [ky][kx][ci][co]; verified 2026-11-24): copy them to
 * natural order, and pack a v5-layout copy for the kernels. */
static void pack_blocks(int8_t *dst, const int8_t *w, int co, int nslots, int per,
                        int (*get)(const int8_t *, int, int, void *), void *ctx)
{
	int g, sl, o;
	memset(dst, 0, (size_t)(co / GRP) * per);
	for (g = 0; g < co / GRP; g++)
		for (sl = 0; sl < nslots; sl++)
			for (o = 0; o < GRP; o++)
				dst[(size_t)g * per + (sl / 2) * BLK + o * 2 + sl % 2] =
					(int8_t)get(w, g * GRP + o, sl, ctx);
}

static int get_pw(const int8_t *w, int o, int sl, void *ctx)
{
	return w[(size_t)o * *(int *)ctx + sl];
}

static int get_dw(const int8_t *w, int o, int sl, void *ctx)
{
	/* slots t0..t7, pad (0), t8 [fmt: Weight layouts] */
	(void)ctx;
	return sl == 8 ? 0 : w[(size_t)o * 9 + (sl == 9 ? 8 : sl)];
}

static int get_first(const int8_t *w, int o, int sl, void *ctx)
{
	/* rows of 10 slots: kx*3 + ci, slot 9 = 0; 3x3x3 only */
	int ky = sl / TAPS_PADDED, r = sl % TAPS_PADDED;
	(void)ctx;
	return r >= 9 ? 0 : w[(size_t)o * 27 + ky * 9 + r];
}

static int unpack_weights(ojzdl_net *n, const char **err)
{
	int i, v5 = n->version >= 5;
	for (i = 0; i < n->nlayers; i++) {
		layer_t *l = &n->L[i];
		const int32_t *P = l->p;
		const int8_t *W = (const int8_t *)l->blob[0];
		int co, ci, o, g, k;
		if (l->type == T_CONV_FIRST) {
			int kh = P[1], kw = P[2], ky, kx;
			ci = n->B[l->bottom[0]].c; co = P[0];
			l->w = malloc((size_t)co * kh * kw * ci);
			if (!l->w)
				FAIL("out of memory");
			if (v5) {
				int per = P[12] / (P[0] / GRP);
				for (g = 0; g < co / GRP; g++)
					for (ky = 0; ky < kh; ky++)
						for (kx = 0; kx < kw; kx++)
							for (k = 0; k < ci; k++) {
								int s = ky * TAPS_PADDED + kx * ci + k;
								for (o = 0; o < GRP; o++)
									l->w[(((size_t)(g * GRP + o) * kh + ky) * kw + kx) * ci + k] =
										W[g * per + (s / 2) * BLK + o * 2 + s % 2];
							}
				l->wp = W;
				l->wper = per;
			} else {
				for (o = 0; o < co; o++)
					for (k = 0; k < kh * kw * ci; k++)
						l->w[(size_t)o * kh * kw * ci + k] = W[(size_t)k * co + o];
				if (kh == 3 && kw == 3 && ci == 3) {
					l->wper = ((3 * TAPS_PADDED + 1) / 2) * BLK;
					l->wp = l->wp_own = malloc((size_t)(co / GRP) * l->wper);
					if (!l->wp_own)
						FAIL("out of memory");
					pack_blocks(l->wp_own, l->w, co, 3 * TAPS_PADDED, l->wper, get_first, NULL);
				}
			}
		} else if (l->type == T_DW) {
			/* tap slots: t0..t7 then a pad slot, t8 last */
			static const uint8_t slot[9] = { 0, 1, 2, 3, 4, 5, 6, 7, 9 };
			co = P[0];
			l->w = malloc((size_t)co * 9);
			if (!l->w)
				FAIL("out of memory");
			if (v5) {
				for (g = 0; g < co / GRP; g++)
					for (k = 0; k < 9; k++)
						for (o = 0; o < GRP; o++)
							l->w[(size_t)(g * GRP + o) * 9 + k] =
								W[g * GRP * TAPS_PADDED + (slot[k] / 2) * BLK + o * 2 + slot[k] % 2];
				l->wp = W;
			} else {
				for (o = 0; o < co; o++)
					for (k = 0; k < 9; k++)
						l->w[(size_t)o * 9 + k] = W[(size_t)k * co + o];
				l->wp = l->wp_own = malloc((size_t)co * TAPS_PADDED);
				if (!l->wp_own)
					FAIL("out of memory");
				pack_blocks(l->wp_own, l->w, co, TAPS_PADDED, GRP * TAPS_PADDED, get_dw, NULL);
			}
		} else if (l->type == T_PW || l->type == T_PW_POOL) {
			int ncp, cp;
			ci = n->B[l->bottom[0]].c; co = P[0]; ncp = ci / 2;
			l->w = malloc((size_t)co * ci);
			if (!l->w)
				FAIL("out of memory");
			if (v5) {
				for (g = 0; g < co / GRP; g++)
					for (cp = 0; cp < ncp; cp++)
						for (o = 0; o < GRP; o++)
							for (k = 0; k < 2; k++)
								l->w[(size_t)(g * GRP + o) * ci + cp * 2 + k] =
									W[((size_t)g * ncp + cp) * BLK + o * 2 + k];
				l->wp = W;
			} else {
				memcpy(l->w, W, (size_t)co * ci);
				l->wp = l->wp_own = malloc((size_t)co * ci);
				if (!l->wp_own)
					FAIL("out of memory");
				pack_blocks(l->wp_own, l->w, co, ci, ci * GRP, get_pw, &ci);
			}
		}
		/* T_HEAD weights are plain [out][in] in both versions */
	}
	return 0;
fail:
	return -1;
}

/* ------------------------------------------------------------------ layers */

#if OJZDL_HAVE_MXU2
static int first_requant_row(const layer_t *l, const int16_t *acc, int wo, int8_t *dst, int bits)
{
	int px, o, co = l->p[0];
	const int32_t *bias = l->q, *mult = l->q + co, *sh = l->q + 2 * co;
	for (px = 0; px < wo; px++)
		for (o = 0; o < co; o++) {
			int64_t v = ((int64_t)acc[(size_t)px * co + o] + bias[o]) * mult[o];
			v >>= Q_FIRST;
			v = rsr64(v, sh[o]);
			dst[(size_t)px * co + o] = clamp_out(v, bits);
		}
	return 0;
}
#endif

static void conv_first_c(const layer_t *l, const blob_t *x, blob_t *y);

static void conv_first(const ojzdl_net *n, const layer_t *l, const blob_t *x, blob_t *y)
{
#if OJZDL_HAVE_MXU2
	if (n->simd && l->simd) {
		const int32_t *P = l->p;
		int pt = P[8], pb = P[9], pl = P[10], pr = P[11], sh = P[5], sw = P[6];
		int hp = x->h + pt + pb, wpx = x->w + pl + pr, ci = x->c, co = y->c;
		size_t rs = (size_t)wpx * ci, iy, oy;
		/* scratch: padded image (+16: the kernel reads 16 bytes from each row
		 * start), then the tail pixels' int16 accumulators [plan_memory] */
		int8_t *xp = n->scratch;
		int16_t *acc = (int16_t *)(void *)(n->scratch + ALIGN16((size_t)hp * rs + 16));
		memset(xp, -IMG_OFFSET, (size_t)hp * rs + 16);
		for (iy = 0; iy < (size_t)x->h; iy++)
			memcpy(xp + (iy + pt) * rs + (size_t)pl * ci, x->i8 + iy * x->w * ci, (size_t)x->w * ci);
		for (oy = 0; oy < (size_t)y->h; oy++) {
			const int8_t *row0 = xp + oy * sh * rs;
			int8_t *drow = y->i8 + oy * y->w * co;
			int px = (int)ojzdl_first_mxu2(row0, rs, y->w, sw * ci, l->wp,
			                               l->wper, co, l->qtab, y->bits, drow);
			int px0 = px;
			for (; px < y->w; px++) {      /* tail pixels: same sums in C */
				int o, ky, k;
				for (o = 0; o < co; o++) {
					int32_t a = 0;
					for (ky = 0; ky < 3; ky++)
						for (k = 0; k < 9; k++)
							a += (int32_t)l->w[(size_t)o * 27 + ky * 9 + k] *
							     row0[ky * rs + (size_t)px * sw * ci + k];
					acc[(size_t)(px - px0) * co + o] = (int16_t)a;
				}
			}
			first_requant_row(l, acc, y->w - px0, drow + (size_t)px0 * co, y->bits);
		}
		return;
	}
#else
	(void)n;
#endif
	conv_first_c(l, x, y);
}

/* TF 'SAME' padding before the data: libjzdl ignores type 46's pad fields [fmt: Layer types] */
static int same_pad_before(int n, int k, int s)
{
	int out = (n + s - 1) / s, tot = (out - 1) * s + k - n;
	return tot > 0 ? tot / 2 : 0;
}

static void conv_first_c(const layer_t *l, const blob_t *x, blob_t *y)
{
	const int32_t *P = l->p;
	int sh = P[5], sw = P[6], bits = y->bits;
	int kh = P[1], kw = P[2], ci = x->c, co = y->c;
	int pt = same_pad_before(x->h, kh, sh), pl = same_pad_before(x->w, kw, sw);
	int oy, ox, o, ky, kx, k;
	/* [fmt: Arithmetic] the pad is input code 0: -128 (pixel 0) on the image,
	 * -2^(bits_in-1) for a type 46 inside the graph (P[15] = input bits) */
	const int8_t pad = (int8_t)-(1 << (P[15] - 1));

	for (oy = 0; oy < y->h; oy++)
		for (ox = 0; ox < y->w; ox++)
			for (o = 0; o < co; o++) {
				const int8_t *wk = l->w + (size_t)o * kh * kw * ci;
				int32_t acc = 0;
				int64_t v;
				for (ky = 0; ky < kh; ky++) {
					int iy = oy * sh + ky - pt;
					for (kx = 0; kx < kw; kx++) {
						int ix = ox * sw + kx - pl;
						const int8_t *w8 = wk + (ky * kw + kx) * ci;
						if (iy >= 0 && iy < x->h && ix >= 0 && ix < x->w) {
							const int8_t *px = x->i8 + ((size_t)iy * x->w + ix) * ci;
							for (k = 0; k < ci; k++)
								acc += (int32_t)w8[k] * px[k];
						} else {
							for (k = 0; k < ci; k++)
								acc += (int32_t)w8[k] * pad;
						}
					}
				}
				/* [fmt: Arithmetic] the sum wraps to int16 before the int32 bias (as the
				 * MXU2 path's int16 accumulators); Q31, int32 bias/mult/shift, multiply floors */
				v = ((int64_t)(int16_t)(uint16_t)(uint32_t)acc + l->q[o]) * l->q[co + o];
				v >>= Q_FIRST;
				v = rsr64(v, l->q[2 * co + o]);
				y->i8[((size_t)oy * y->w + ox) * co + o] = clamp_out(v, bits);
			}
}

/* [fmt: Arithmetic] 1x1/depthwise: wrap16(acc + bias), * int16 mult, >> 15 (floor),
 * rounding shift, clamp to [0, 2^bits - 1], minus 2^(bits-1). */
static inline int8_t requant16(int32_t acc, int16_t bias, int16_t mult, int16_t sh, int bits)
{
	int16_t a = (int16_t)(uint16_t)(uint32_t)(acc + bias); /* modulo 2^16 */
	/* >> on a negative int is arithmetic in GCC (implementation-defined, C99 6.5.7) */
	int32_t v = ((int32_t)a * mult) >> Q_OTHER;
	return clamp_out(rsr32(v, sh), bits);
}

static void conv_dw_c(const layer_t *l, const blob_t *x, blob_t *y);

static void conv_dw(const ojzdl_net *n, const layer_t *l, const blob_t *x, blob_t *y)
{
#if OJZDL_HAVE_MXU2
	if (n->simd && l->simd) {
		const int32_t *P = l->p;
		int pt = P[8], pb = P[9], pl = P[10], pr = P[11];
		int wpx = x->w + pl + pr, c = x->c;
		size_t rs = (size_t)wpx * c, iy;
		int8_t *xp = n->scratch; /* padded input [plan_memory] */
		{
			/* [fmt: Arithmetic] pad with the activation zero */
			memset(xp, -(1 << (x->bits - 1)), (size_t)(x->h + pt + pb) * rs);
			for (iy = 0; iy < (size_t)x->h; iy++)
				memcpy(xp + (iy + pt) * rs + (size_t)pl * c, x->i8 + iy * x->w * c, (size_t)x->w * c);
			ojzdl_dw_mxu2(xp, wpx, y->h, y->w, c, P[5], l->wp,
			              l->blob[1], l->blob[2], l->blob[3], y->bits, y->i8);
			return;
		}
	}
#else
	(void)n;
#endif
	conv_dw_c(l, x, y);
}

static void conv_dw_c(const layer_t *l, const blob_t *x, blob_t *y)
{
	const int32_t *P = l->p;
	int sh = P[5], sw = P[6], pt = P[8], pl = P[10], bits = y->bits, c = y->c;
	int8_t pad = (int8_t)-(1 << (x->bits - 1));
	int oy, ox, o, ky, kx;

	for (oy = 0; oy < y->h; oy++)
		for (ox = 0; ox < y->w; ox++) {
			int8_t *dst = y->i8 + ((size_t)oy * y->w + ox) * c;
			for (o = 0; o < c; o++) {
				int32_t acc = 0;
				for (ky = 0; ky < 3; ky++) {
					int iy = oy * sh + ky - pt;
					for (kx = 0; kx < 3; kx++) {
						int ix = ox * sw + kx - pl;
						int8_t v = (iy >= 0 && iy < x->h && ix >= 0 && ix < x->w)
							? x->i8[((size_t)iy * x->w + ix) * c + o] : pad;
						acc += (int32_t)l->w[(size_t)o * 9 + ky * 3 + kx] * v;
					}
				}
				dst[o] = requant16(acc, (int16_t)l->q[o], (int16_t)l->q[c + o],
				                   (int16_t)l->q[2 * c + o], bits);
			}
		}
}

static int32_t dot8(const int8_t *a, const int8_t *b, int n)
{
	int32_t s = 0;
	int i;
	for (i = 0; i < n; i++)
		s += (int32_t)a[i] * b[i];
	return s;
}

static void conv_pw(const ojzdl_net *n, const layer_t *l, const blob_t *x, int8_t *dst, int bits)
{
	int ci = x->c, co = l->p[0], o;
	size_t px = 0, npx = (size_t)x->h * x->w;
#if OJZDL_HAVE_MXU2
	if (n->simd && l->simd)
		px = ojzdl_pw_mxu2(x->i8, npx, ci, l->wp, co,
		                   l->blob[1], l->blob[2], l->blob[3], bits, dst);
#else
	(void)n;
#endif
	for (; px < npx; px++) {
		const int8_t *src = x->i8 + px * ci;
		int8_t *d = dst + px * co;
		for (o = 0; o < co; o++)
			d[o] = requant16(dot8(src, l->w + (size_t)o * ci, ci), (int16_t)l->q[o],
			                 (int16_t)l->q[co + o], (int16_t)l->q[2 * co + o], bits);
	}
}

/* one 2x2/2 pooled output row from input rows a, b */
static void pool_row(const ojzdl_net *n, const int8_t *a, const int8_t *b, int w, int c, int8_t *dst)
{
	int ox, k;
#if OJZDL_HAVE_MXU2
	if (n->simd && c % 16 == 0) {
		ojzdl_pool2_mxu2(a, b, w, c, dst);
		return;
	}
#else
	(void)n;
#endif
	for (ox = 0; ox < w / 2; ox++)
		for (k = 0; k < c; k++) {
			const int8_t *p = a + (size_t)2 * ox * c + k, *q = b + (size_t)2 * ox * c + k;
			int8_t m = p[0];
			if (p[c] > m) m = p[c];
			if (q[0] > m) m = q[0];
			if (q[c] > m) m = q[c];
			dst[(size_t)ox * c + k] = m;
		}
}

static void head(const ojzdl_net *n, const layer_t *l, const blob_t *x, blob_t *y)
{
	int ci = x->c, co = y->c, o;
	size_t px, npx = (size_t)x->h * x->w;
#if OJZDL_HAVE_MXU2
	if (n->simd && l->simd) {
		int32_t *acc = (int32_t *)(void *)n->scratch; /* [plan_memory] */
		ojzdl_head_mxu2(x->i8, npx, ci, (const int8_t *)l->blob[0], co, acc);
		for (px = 0; px < npx; px++)
			for (o = 0; o < co; o++)
				y->f32[px * co + o] = (float)(acc[px * co + o] + rd32(l->blob[1] + 4 * o)) *
				                      rdf(l->blob[2] + 4 * o);
		return;
	}
#else
	(void)n;
#endif
	for (px = 0; px < npx; px++)
		for (o = 0; o < co; o++) {
			int32_t acc = dot8(x->i8 + px * ci, (const int8_t *)l->blob[0] + (size_t)o * ci, ci);
			/* [fmt: Arithmetic] float32(acc + bias) * scale, no clamp */
			y->f32[px * co + o] = (float)(acc + rd32(l->blob[1] + 4 * o)) * rdf(l->blob[2] + 4 * o);
		}
}

static void upsample(const layer_t *l, const blob_t *x, blob_t *y)
{
	int sy = l->p[2], sx = l->p[3], iy, ix, oy, ox;
	if (l->p[4] == 1) {
		/* [fmt: Layer types] f4 = 1 (modes 4, 5): nearest neighbour */
		for (oy = 0; oy < y->h; oy++)
			for (ox = 0; ox < y->w; ox++)
				memcpy(y->i8 + ((size_t)oy * y->w + ox) * y->c,
				       x->i8 + ((size_t)(oy / sy) * x->w + ox / sx) * x->c, (size_t)x->c);
		return;
	}
	/* [fmt: Layer types] mode 4, f4 = 0: out[s*i, s*j] = x, else the activation zero */
	memset(y->i8, -(1 << (x->bits - 1)), y->n);
	for (iy = 0; iy < x->h; iy++)
		for (ix = 0; ix < x->w; ix++)
			memcpy(y->i8 + ((size_t)iy * sy * y->w + (size_t)ix * sx) * y->c,
			       x->i8 + ((size_t)iy * x->w + ix) * x->c, (size_t)x->c);
}

/* [fmt: Layer types] type 78, equal input widths: u = round_half_even((ua + ub) / 2) */
static void eltwise(const blob_t *a, const blob_t *b, blob_t *y)
{
	int off = 1 << (a->bits - 1), hi = (1 << y->bits) - 1, yo = 1 << (y->bits - 1);
	size_t i;
	for (i = 0; i < y->n; i++) {
		int s2 = a->i8[i] + b->i8[i] + 2 * off, k = s2 >> 1;
		int u = k + ((s2 & 1) & (k & 1));
		if (u > hi)
			u = hi;
		y->i8[i] = (int8_t)(u - yo);
	}
}

static void concat(const ojzdl_net *n, const layer_t *l, blob_t *y)
{
	size_t px, npx = (size_t)y->h * y->w;
	int k, off = 0;
	for (k = 0; k < l->nb; k++) {
		const blob_t *b = &n->B[l->bottom[k]];
		for (px = 0; px < npx; px++)
			memcpy(y->i8 + px * y->c + off, b->i8 + px * b->c, (size_t)b->c);
		off += b->c;
	}
}

static void pack(const ojzdl_net *n, const layer_t *l, blob_t *y)
{
	float *o = y->f32;
	int k;
	/* [fmt: Arithmetic] heads flattened HWC, then h, w, c per head, then count */
	for (k = 0; k < l->nb; k++) {
		const blob_t *b = &n->B[l->bottom[k]];
		memcpy(o, b->f32, b->n * sizeof(float));
		o += b->n;
	}
	for (k = 0; k < l->nb; k++) {
		const blob_t *b = &n->B[l->bottom[k]];
		*o++ = (float)b->h;
		*o++ = (float)b->w;
		*o++ = (float)b->c;
	}
	*o = (float)l->nb;
}

/* ------------------------------------------------------------------ forward */

/* Scratch bytes a layer needs (pads, temps); must match the users above. */
static size_t layer_scratch(const ojzdl_net *n, const layer_t *l)
{
	const blob_t *x, *y;
	if (l->type == T_INPUT || l->nb < 1)
		return 0;
	x = &n->B[l->bottom[0]];
	y = &n->B[l->top[0]];
	switch (l->type) {
	case T_PW_POOL:
		return (size_t)2 * x->w * y->c;
	case T_HEAD:
		return OJZDL_HAVE_MXU2 && l->simd ? (size_t)x->h * x->w * y->c * sizeof(int32_t) : 0;
	case T_DW:
		return OJZDL_HAVE_MXU2 && l->simd
			? (size_t)(x->h + l->p[8] + l->p[9]) * (x->w + l->p[10] + l->p[11]) * x->c : 0;
	case T_CONV_FIRST:
		return OJZDL_HAVE_MXU2 && l->simd
			? ALIGN16((size_t)(x->h + l->p[8] + l->p[9]) * (x->w + l->p[10] + l->p[11]) * x->c + 16)
			  + 4 * (size_t)y->c * sizeof(int16_t) /* < 4 tail pixels */
			: 0;
	default:
		return 0;
	}
}

/* Place every owner blob in one arena: blob b lives from its producing layer
 * to its last reader (to the end for the output, or for all with keep_all);
 * first fit by offset among blobs whose lifetimes overlap. Touch it all once
 * so no frame takes a page fault. */
static int plan_memory(ojzdl_net *n)
{
	int nb = n->nblobs, i, j, moved;
	int *start = malloc((size_t)nb * sizeof(int)), *end = malloc((size_t)nb * sizeof(int));
	size_t *off = malloc((size_t)nb * sizeof(size_t)), *sz = malloc((size_t)nb * sizeof(size_t));
	size_t total = 0, scr = 0;
	int rc = -1;

	if (!start || !end || !off || !sz)
		goto out;
	for (i = 0; i < nb; i++) {
		blob_t *b = &n->B[i];
		start[i] = -2; /* not an owner / not produced */
		sz[i] = b->bits ? b->n : b->n * sizeof(float);
	}
	start[n->in_blob] = -1;
	for (i = 0; i < n->nlayers; i++) {
		const layer_t *l = &n->L[i];
		size_t s = layer_scratch(n, l);
		if (s > scr)
			scr = s;
		if (l->type != T_INPUT && l->type != T_SPLIT)
			start[l->top[0]] = i;
	}
	for (i = 0; i < nb; i++) {
		blob_t *b = &n->B[i];
		if (start[i] == -2 || b->owner != i)
			continue;
		end[i] = (n->keep_all || i == n->out_blob || b->last_use < start[i]) ? n->nlayers : b->last_use;
		off[i] = 0;
		do { /* first fit: bump past any placed, live-overlapping blob in the way */
			moved = 0;
			for (j = 0; j < i; j++) {
				if (start[j] == -2 || n->B[j].owner != j)
					continue;
				if (start[j] > end[i] || start[i] > end[j])
					continue;
				if (off[i] < off[j] + ALIGN16(sz[j]) && off[j] < off[i] + ALIGN16(sz[i])) {
					off[i] = off[j] + ALIGN16(sz[j]);
					moved = 1;
				}
			}
		} while (moved);
		if (off[i] + ALIGN16(sz[i]) > total)
			total = off[i] + ALIGN16(sz[i]);
	}
	free(n->arena);
	free(n->scratch);
	n->arena = malloc(total + 16);
	n->scratch = scr ? malloc(scr + 16) : NULL;
	if (!n->arena || (scr && !n->scratch))
		goto out;
	memset(n->arena, 0, total + 16);
	if (n->scratch)
		memset(n->scratch, 0, scr + 16);
	n->arena_sz = total;
	n->scratch_sz = scr;
	for (i = 0; i < nb; i++) {
		blob_t *b = &n->B[i];
		uint8_t *base = (uint8_t *)ALIGN16((size_t)n->arena);
		if (start[i] == -2 || b->owner != i)
			continue;
		b->i8 = b->bits ? (int8_t *)(base + off[i]) : NULL;
		b->f32 = b->bits ? NULL : (float *)(void *)(base + off[i]);
	}
	for (i = 0; i < nb; i++) { /* split tops alias their owner */
		blob_t *b = &n->B[i];
		if (b->owner != i) {
			b->i8 = n->B[b->owner].i8;
			b->f32 = n->B[b->owner].f32;
		}
	}
	n->ran = 0;
	rc = 0;
out:
	free(start);
	free(end);
	free(off);
	free(sz);
	return rc;
}

int ojzdl_input_rgb(ojzdl_net *n, const uint8_t *rgb)
{
	blob_t *b = &n->B[n->in_blob];
	size_t i = 0;
#if OJZDL_HAVE_MXU2
	if (n->simd)
		i = ojzdl_input_mxu2(rgb, b->n, b->i8);
#endif
	for (; i < b->n; i++)
		b->i8[i] = (int8_t)(rgb[i] - IMG_OFFSET);
	return 0;
}

static uint64_t now_ns(void)
{
	struct timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	return (uint64_t)t.tv_sec * 1000000000u + (uint64_t)t.tv_nsec;
}

int ojzdl_forward(ojzdl_net *n)
{
	int i;
	if (!n->arena)
		return -1;
	for (i = 0; i < n->nlayers; i++) {
		layer_t *l = &n->L[i];
		blob_t *x, *y;
		uint64_t t0 = n->timing ? now_ns() : 0;
		if (l->type == T_INPUT)
			continue;
		x = &n->B[l->bottom[0]];
		y = &n->B[l->top[0]];
		if (l->type == T_SPLIT)
			continue; /* tops alias the bottom [plan_memory] */
		switch (l->type) {
		case T_CONV_FIRST: conv_first(n, l, x, y); break;
		case T_DW: conv_dw(n, l, x, y); break;
		case T_PW: conv_pw(n, l, x, y->i8, y->bits); break;
		case T_PW_POOL: {
			/* [fmt: Layer types] conv -> requant -> 2x2/2 max-pool */
			/* two input rows at a time: conv into scratch, pool into one output row */
			int8_t *tmp = n->scratch; /* 2 * w * co [plan_memory] */
			blob_t rows = *x;
			int oy;
			rows.h = 2;
			for (oy = 0; oy < y->h; oy++) {
				rows.i8 = x->i8 + (size_t)2 * oy * x->w * x->c;
				conv_pw(n, l, &rows, tmp, y->bits);
				pool_row(n, tmp, tmp + (size_t)x->w * y->c, x->w, y->c, y->i8 + (size_t)oy * y->w * y->c);
			}
			break;
		}
		case T_POOL: {
			int oy;
			for (oy = 0; oy < y->h; oy++)
				pool_row(n, x->i8 + (size_t)2 * oy * x->w * x->c, x->i8 + (size_t)(2 * oy + 1) * x->w * x->c,
				         x->w, x->c, y->i8 + (size_t)oy * y->w * y->c);
			break;
		}
		case T_HEAD: head(n, l, x, y); break;
		case T_UPSAMPLE: upsample(l, x, y); break;
		case T_ELTWISE: eltwise(x, &n->B[l->bottom[1]], y); break;
		case T_CONCAT: concat(n, l, y); break;
		case T_PACK: pack(n, l, y); break;
		default: return -1;
		}
		if (n->timing)
			l->ns = now_ns() - t0;
	}
	n->ran = 1;
	return 0;
}

const float *ojzdl_output(const ojzdl_net *n, size_t *count)
{
	const blob_t *b = &n->B[n->out_blob];
	if (count)
		*count = n->ran ? b->n : 0;
	return n->ran ? b->f32 : NULL;
}

int ojzdl_blob(const ojzdl_net *n, int blob, int *h, int *w, int *c, int *bits,
               const int8_t **i8, const float **f32)
{
	const blob_t *b;
	if (blob < 0 || blob >= n->nblobs)
		return -1;
	b = &n->B[blob];
	if (!b->i8 && !b->f32)
		return -1;
	*h = b->h; *w = b->w; *c = b->c; *bits = b->bits;
	*i8 = n->B[b->owner].i8; *f32 = n->B[b->owner].f32;
	return 0;
}

int ojzdl_in_h(const ojzdl_net *n) { return n->in_h; }
int ojzdl_in_w(const ojzdl_net *n) { return n->in_w; }
int ojzdl_in_c(const ojzdl_net *n) { return n->in_c; }

void ojzdl_set_timing(ojzdl_net *n, int on) { n->timing = on; }
int ojzdl_keep_blobs(ojzdl_net *n, int on)
{
	n->keep_all = on;
	return plan_memory(n); /* lifetimes change: re-plan */
}

size_t ojzdl_memory(const ojzdl_net *n, size_t *arena, size_t *scratch)
{
	if (arena)
		*arena = n->arena_sz;
	if (scratch)
		*scratch = n->scratch_sz;
	return n->arena_sz + n->scratch_sz;
}
int ojzdl_set_simd(ojzdl_net *n, int on)
{
	n->simd = on && OJZDL_HAVE_MXU2;
	return 0; /* scratch is planned for the MXU2 layers, which covers the C paths */
}
int ojzdl_layer_count(const ojzdl_net *n) { return n->nlayers; }

int ojzdl_layer_info(const ojzdl_net *n, int i, int *type, uint64_t *ns)
{
	if (i < 0 || i >= n->nlayers)
		return -1;
	*type = n->L[i].type;
	*ns = n->L[i].ns;
	return 0;
}
