/*
 * onnaq: run a venus .bin of the QSUM kind (the NNA requantizes every conv; ReLU; depthwise 3x3,
 * per-channel 1x1, 2x2 maxpool, ConvTranspose 2x2/2, concat) on the T41 NNA, or on the CPU.
 * First model: the thingino-ai person/pet/package detector (416x320, 2 heads).
 *
 *   onnaq [-e cpu|oracle|nna] [-c] [-T] [-n N] model.bin model.qgraph frame.rgb prefix
 * With -DONNAQ_LIB there is no main: onnaq.h is the API (raptor-hal's onna backend).
 *   -e  where convs run: cpu (integer loops), oracle (the NNA driver against the host test
 *       oracle; ORACLE=1 build) or nna (the real NNA; T41 build, pins itself to CPU0)
 *   -c  check every engine conv against the CPU conv
 *   -T  time per op kind
 *   -n  run N times (timing); the heads must be identical in every run
 * frame.rgb: H x W x 3 RGB bytes (the network size). Writes prefix.<k>.f32 per head (NHWC float).
 *
 * The arithmetic (docs/nna-runtime.md, "QSUM models"): conv q = clamp(floor((4 S + B) / T) + 128),
 * S = sum (raw - 128) * x over raw input bytes x; heads f32(S + B) * Fl; 2x2 maxpool = byte max;
 * concat input i = rint(x * s_i / s_out); padding 0. All numbers come from the .bin.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: MIT
 */
#ifdef ONNA_MXU3
#define _GNU_SOURCE
#endif
#include <malloc.h>
#include <math.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "nnabin.h"
#include "nnaeng.h"
#include "onnaq.h"
#ifdef ONNA_ORACLE
#include "be_oracle.h"
#endif
#ifdef ONNA_MXU3
#include "be_mxu3.h"
#include <sched.h>
#endif

/* errors: inside onnaq_open / onnaq_run they return -1 (longjmp back, message in onnaq_error());
 * in the command-line tool they exit */
static jmp_buf *die_jb;
static char die_msg[256];

static void die(const char *fmt, ...) __attribute__((noreturn, format(printf, 1, 2)));
static void die(const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	vsnprintf(die_msg, sizeof(die_msg), fmt, ap);
	va_end(ap);
	if (die_jb)
		longjmp(*die_jb, 1);
	fprintf(stderr, "onnaq: %s\n", die_msg);
	exit(1);
}
#define DIE(...) die(__VA_ARGS__)

/* ---- the .bin: W blocks with their param records, header concat records */

/* capacities: YOLO-style graphs (yoloxn) have 300+ convs and 450+ ops */
#define MAXREC 1024
#define MAXADD 512
#define MAXNODE 1024
#define MAXTENS 1024
#define MAXCAT 64

struct qrec {
	int nob, nic, kh, kw, dw;
	int8_t *wq;		/* dense [ky][kx][oc][ic] (oc = 32 nob, ic = 32 nic); dw [ky][kx][oc] */
	int32_t *bt;		/* per block: B[32], T[32] */
	float *fl;		/* per oc: the head factor (the T slot read as float) */
	int sh;			/* requant: q = floor(((S << sh) + B) / T) + 128; sh from the op list, 2 (x4) by default */
};

static struct qrec rec[MAXREC];
static int nrec;
static float cat_a[MAXCAT][4];
/* the header's name key: name byte i is stored XOR NAMEKEY[i % 9] (a 9-byte cycle, recovered
 * from known tensor names) */
static const uint8_t NAMEKEY[9] = { 0xf4, 0x36, 0xa5, 0x3e, 0x96, 0x7c, 0xce, 0x94, 0xe8 };

/* a header name at o: int32 length, then the bytes XOR NAMEKEY. Returns the length (and the text in
 * out) when every byte decodes to [A-Za-z0-9._-/:*], else -1 */
static int dstr(const uint8_t *b, long n, long o, char *out, int cap)
{
	int32_t L = o + 4 <= n ? (int32_t)(b[o] | b[o + 1] << 8 | b[o + 2] << 16 | (uint32_t)b[o + 3] << 24) : -1;

	if (L < 1 || L >= cap || o + 4 + L > n)
		return -1;
	for (int i = 0; i < L; i++) {
		int c = b[o + 4 + i] ^ NAMEKEY[i % 9];

		if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '.' || c == '_' ||
		      c == '-' || c == '/' || c == ':' || c == '*'))
			return -1;
		out[i] = (char)c;
	}
	out[L] = 0;
	return L;
}
struct addrec { int32_t wa, wb, B, T; };
static struct addrec addr[MAXADD];
static int nadd;
static int ncat, cat_n[MAXCAT];

static int32_t rd32(const uint8_t *p)
{
	int32_t v;

	memcpy(&v, p, 4);
	return v;
}

static const uint8_t SIG[10] = { 2, 0, 0, 0, 2, 0, 0, 0, 1, 0 };

static void load_bin(const char *path)
{
	FILE *f = fopen(path, "rb");
	long n;
	uint8_t *b;

	if (!f)
		DIE("cannot open %s", path);
	fseek(f, 0, SEEK_END);
	n = ftell(f);
	rewind(f);
	b = malloc((size_t)n);
	if (!b || fread(b, 1, (size_t)n, f) != (size_t)n)
		DIE("cannot read %s", path);
	fclose(f);
	/* W header [bytes, 7, nob, nic, kh, kw, 4, 32, icg] + W, then the param record (SIG) */
	for (long o = 36; o + 36 <= n; o++) {
		const uint8_t *h = b + o - 36;
		int32_t nb = rd32(h), nob = rd32(h + 8), nic = rd32(h + 12), kh = rd32(h + 16), kw = rd32(h + 20);
		int32_t icg = rd32(h + 32);

		if (rd32(h + 4) != 7 || rd32(h + 24) != 4 || rd32(h + 28) != 32 || (icg != 1 && icg != 32) ||
		    nb <= 0 || nb > n - o || nb % (icg == 1 ? 32 : 256) || o + nb + 0x25 + 256L * nob > n ||
		    memcmp(b + o + nb, SIG, sizeof(SIG)))
			continue;
		struct qrec *r = &rec[nrec];
		const uint8_t *w = b + o, *p = b + o + nb;
		int taps = kh * kw, ic = icg == 1 ? 1 : 32 * nic, oc = 32 * nob;

		if (nrec == MAXREC)
			DIE("too many W blocks");
		if (nob < 1 || nob > 64 || nic < 1 || kh < 1 || kh > 3 || kw < 1 || kw > 3 ||
		    nb != (icg == 1 ? 32 * nob * taps : 1024 * nob * nic * taps))
			DIE("W block at %ld: bytes %d nob %d nic %d k %dx%d", o, nb, nob, nic, kh, kw);
		if (rd32(p + 8) != (nob << 16 | 1) || rd32(p + 12) != 1280 || rd32(p + 20) != 512 * nob)
			DIE("param record at %ld: fields %d %d %d", o + nb, rd32(p + 8), rd32(p + 12), rd32(p + 20));
		r->nob = nob, r->nic = icg == 1 ? 1 : nic, r->kh = kh, r->kw = kw, r->dw = icg == 1;
		r->wq = malloc((size_t)taps * oc * ic);
		/* 2-bit slices: slice k of each weight in group entry k; raw - 128 */
		if (r->dw) {
			for (int ob = 0; ob < nob; ob++)	/* block -> tap (ky, kx) -> 4 slices x 8 B (32 oc) */
				for (int t = 0; t < taps; t++) {
					const uint8_t *e = w + 32L * (ob * taps + t);

					for (int c = 0; c < 32; c++) {
						int v = 0;

						for (int k = 0; k < 4; k++)
							v |= (e[8 * k + c / 4] >> (2 * (c % 4)) & 3) << (2 * k);
						r->wq[(long)t * oc + 32 * ob + c] = (int8_t)(v - 128);
					}
				}
		} else {
			/* block -> input chunk -> tap (ky, kx) -> 4 entries [32 oc][8 B]; wq [t][oc][ic] */
			for (int ob = 0; ob < nob; ob++)
				for (int ck = 0; ck < nic; ck++)
					for (int t = 0; t < taps; t++) {
						const uint8_t *e = w + 1024L * (((long)ob * nic + ck) * taps + t);

						for (int o2 = 0; o2 < 32; o2++)
							for (int c = 0; c < 32; c++) {
								int v = 0;

								for (int k = 0; k < 4; k++)
									v |= (e[256 * k + 8 * o2 + c / 4] >> (2 * (c % 4)) & 3) << (2 * k);
								r->wq[((long)t * oc + 32 * ob + o2) * ic + 32 * ck + c] = (int8_t)(v - 128);
							}
					}
		}
		r->bt = malloc(256L * nob);
		r->fl = malloc(sizeof(float) * (size_t)oc);
		for (int ob = 0; ob < nob; ob++) {
			memcpy(r->bt + 64 * ob, p + 0x25 + 256 * ob, 256);
			memcpy(r->fl + 32 * ob, p + 0xa5 + 256 * ob, 128);
		}
		nrec++;
		o += nb - 1;
	}
	/* header concat records: n, 0 x n, n, float s_1..s_n, 1/s_out (zero point 0) */
	for (int k = 2; k <= 4; k++)
		for (long o = 0; o + 4 * (2 * k + 3) <= n; o++) {
			int ok = rd32(b + o) == k && rd32(b + o + 4 * (k + 1)) == k;
			float s[5];

			for (int i = 0; ok && i < k; i++)
				ok = rd32(b + o + 4 + 4 * i) == 0;
			if (!ok)
				continue;
			memcpy(s, b + o + 4 * (k + 2), 4 * (size_t)(k + 1));
			for (int i = 0; ok && i <= k; i++)
				ok = s[i] > 1e-6f && s[i] < 1e3f;
			if (!ok || ncat == MAXCAT)
				continue;
			/* records of different arity are not interleaved in the models seen; keep file order
			 * per arity and fail later if a concat's arity does not match */
			cat_n[ncat] = k;
			for (int i = 0; i < k; i++)
				cat_a[ncat][i] = s[i] * s[k];
			ncat++;
		}
	/* add records, in the converter's op order. The header is the converter's op list: names are
	 * length-prefixed strings XORed with a fixed per-position key (NAMEKEY; recovered from known
	 * tensor names). Each Add op carries its kernel string "Add/add_..." and, a few bytes after it,
	 * its record 2, wA, wB, 2, float sA, sB, B, T. out = clamp(floor((64 (wA a + wB b) + B) / T) +
	 * 128): nna_add_ref's arithmetic (fitted 2026-11-25 on the PC executor's tensors, 0 of 532480
	 * wrong). An identity clamp has wB = 0 (a plain rescale of a), so no value filter. The graph's
	 * add count must match (load_graph) */
	for (long o = 0; o + 4 + 8 <= n; o++) {
		static const char pre[] = "Add/add_";
		int32_t L = rd32(b + o);
		int k;

		if (L < 8 || L > 256 || o + 4 + L > n)
			continue;
		for (k = 0; k < 8 && (b[o + 4 + k] ^ NAMEKEY[k % 9]) == (uint8_t)pre[k]; k++)
			;
		if (k < 8)
			continue;
		for (long d = o + 4 + L; d + 32 <= n && d < o + 4 + L + 160; d++) {
			const uint8_t *p = b + d;
			int32_t wa = rd32(p + 4), wb = rd32(p + 8);
			float bt[2];

			if (rd32(p) != 2 || rd32(p + 12) != 2 || wa < 0 || wa > 255 || wb < 0 || wb > 255)
				continue;
			memcpy(bt, p + 24, sizeof(bt));
			if (nadd == MAXADD)
				DIE("too many add records");
			if (!(bt[1] >= 1))
				DIE("add record at %ld: T %g", d, (double)bt[1]);
			addr[nadd++] = (struct addrec){ wa, wb, (int32_t)bt[0], (int32_t)bt[1] };
			break;
		}
		o += 4 + L - 1;
	}
	/* per conv requant shift, in op order (= W block order). A Convolution op starts with the names
	 * "Convolution", <out>, "UNIQUE"; heads are Convolution ops too. In an op, after its kernel
	 * string "Convolution/...": 01, int32 2, int32 1, <name>, int32, int32, int32 shift. Most layers
	 * use 2 (x4); a layer whose int8 weights the converter scaled down uses more (6: x64, fitted on
	 * the PC executor). The stem's and heads' records differ: they keep 2 */
	{
		int k = 0, cur = -1, done = 0;
		char s1[260], s2[260], s3[260];

		for (int i = 0; i < nrec; i++)
			rec[i].sh = 2;
		for (long o = 0; o + 4 < n; o++) {
			int l1 = dstr(b, n, o, s1, sizeof(s1)), l2, l3;

			if (l1 < 0)
				continue;
			if (l1 == 11 && !strcmp(s1, "Convolution") && (l2 = dstr(b, n, o + 15, s2, sizeof(s2))) > 0 &&
			    (l3 = dstr(b, n, o + 19 + l2, s3, sizeof(s3))) == 6 && !strcmp(s3, "UNIQUE")) {
				cur = k++, done = 0;
				o += 4 + l1 - 1;
				continue;
			}
			/* "Convolution/base_i8/..." (byte out); heads are "base_i8of" (float out), another record */
			if (l1 > 20 && !strncmp(s1, "Convolution/base_i8/", 20) && cur >= 0 && cur < nrec && !done) {
				long e = o + 4 + l1;
				int32_t nl = e + 13 <= n ? rd32(b + e + 9) : -1;

				if (b[e] == 1 && rd32(b + e + 1) == 2 && rd32(b + e + 5) == 1 && nl > 0 && nl < 256 &&
				    e + 13 + nl + 12 <= n) {
					int32_t v = rd32(b + e + 13 + nl + 8);

					if (v < 0 || v > 16)
						DIE("conv op %d (%s): requant shift %d", cur, s1, v);
					rec[cur].sh = v;
				}
				done = 1;
			}
			o += 4 + l1 - 1;
		}
		if (k && k != nrec)
			DIE("%d Convolution ops, %d W blocks", k, nrec);
	}
	free(b);
}

/* ---- the graph */

enum { G_STEM, G_CONV, G_PC, G_HEAD, G_POOL, G_DECONV, G_CAT, G_ADD };
struct gnode {
	int op, out, in[4], nin, rec, oc, ic, k, pad, dw, crec;
	int stride, padb;	/* a conv: stride, and the bottom/right pad (pad = top/left) */
	int skip;		/* folded into the conv before it, or a deconv folded into its concat */
	int dec[4];		/* a concat: input k comes from deconv node dec[k] - 1 (0: none) */
	uint8_t *post;		/* a conv: the folded per-channel ops as one byte table (all channels) */
};
static struct gnode G[MAXNODE];
static int ng, ntens, in_h, in_w, outs[4], nouts;
static int nanc, anc_stride[4];		/* "anchors <stride> w0 h0 w1 h1 w2 h2" lines */
static float anc_wh[4][6];

static void load_graph(const char *path)
{
	FILE *f = fopen(path, "r");
	char line[512];

	if (!f)
		DIE("cannot open %s", path);
	ng = ntens = in_h = in_w = nouts = nanc = 0;
	while (fgets(line, sizeof(line), f)) {
		struct gnode g;
		char op[32];
		int a[12], na;

		memset(&g, 0, sizeof(g));
		na = sscanf(line, "%31s %d %d %d %d %d %d %d %d %d %d", op, &a[0], &a[1], &a[2], &a[3], &a[4],
			    &a[5], &a[6], &a[7], &a[8], &a[9]) - 1;
		if (na < 0)
			continue;
		if (!strcmp(op, "onnaq-graph"))
			continue;
		if (!strcmp(op, "tensors")) {
			ntens = a[0];
			if (ntens < 1 || ntens > MAXTENS)
				DIE("graph: %d tensors (max %d)", ntens, MAXTENS);
			continue;
		}
		if (!strcmp(op, "input")) {
			in_h = a[1], in_w = a[2];
			continue;
		}
		if (!strcmp(op, "anchors")) {
			float v[7];

			if (sscanf(line, "%*s %f %f %f %f %f %f %f", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6]) != 7 ||
			    nanc == 4)
				DIE("graph line: %s", line);
			anc_stride[nanc] = (int)v[0];
			memcpy(anc_wh[nanc++], v + 1, sizeof(anc_wh[0]));
			continue;
		}
		if (!strcmp(op, "output")) {
			for (nouts = 0; nouts < na && nouts < 4; nouts++)
				outs[nouts] = a[nouts];
			continue;
		}
		g.out = a[0], g.in[0] = a[1], g.nin = 1;
		if (!strcmp(op, "stem") && na == 4)
			g.op = G_STEM, g.rec = a[2], g.oc = a[3];
		else if (!strcmp(op, "conv") && (na == 8 || na == 10)) {
			g.op = G_CONV, g.rec = a[2], g.oc = a[3], g.ic = a[4], g.k = a[5], g.pad = a[6], g.dw = a[7];
			g.stride = na == 10 ? a[8] : 1, g.padb = na == 10 ? a[9] : g.pad;
			if (g.stride < 1 || g.stride > 2 || g.padb < 0 || g.padb > 2)
				DIE("graph line: %s", line);
		}
		else if (!strcmp(op, "pc") && na == 4)
			g.op = G_PC, g.rec = a[2], g.oc = a[3];
		else if (!strcmp(op, "head") && na == 5)
			g.op = G_HEAD, g.rec = a[2], g.oc = a[3], g.ic = a[4];
		else if (!strcmp(op, "maxpool2") && na == 2)
			g.op = G_POOL;
		else if (!strcmp(op, "deconv2") && na == 4)
			g.op = G_DECONV, g.rec = a[2], g.oc = a[3];
		else if (!strcmp(op, "concat") && na >= 4 && na == a[1] + 3) {
			g.op = G_CAT, g.nin = a[1];
			if (g.nin > 4)
				DIE("concat of %d", g.nin);
			for (int i = 0; i < g.nin; i++)
				g.in[i] = a[2 + i];
			g.crec = a[2 + g.nin];
		} else if (!strcmp(op, "add") && na == 4) {
			g.op = G_ADD, g.nin = 2, g.in[1] = a[2], g.crec = a[3];
			if (g.crec < 0 || g.crec >= nadd)
				DIE("graph: add record %d of %d", g.crec, nadd);
		} else
			DIE("graph line: %s", line);
		if (g.op != G_POOL && g.op != G_CAT && g.op != G_ADD && (g.rec < 0 || g.rec >= nrec))
			DIE("graph: record %d of %d", g.rec, nrec);
		if (g.op == G_CAT && (g.crec >= ncat || cat_n[g.crec] != g.nin))
			DIE("graph: concat record %d", g.crec);
		if (ng == MAXNODE)
			DIE("graph too long");
		for (int k = 0; k < g.nin; k++)
			if (g.in[k] < 0 || g.in[k] >= ntens)
				DIE("graph: tensor %d of %d", g.in[k], ntens);
		if (g.out < 0 || g.out >= ntens)
			DIE("graph: tensor %d of %d", g.out, ntens);
		G[ng++] = g;
	}
	fclose(f);
	if (!ntens || !ng || !nouts || !in_h)
		DIE("graph %s incomplete", path);
	{
		int na = 0;

		for (int i = 0; i < ng; i++)
			na += G[i].op == G_ADD;
		if (na != nadd)
			DIE("graph has %d adds, the .bin %d add records", na, nadd);
	}
}

static uint8_t add_q(const struct addrec *r, int a, int b)
{
	int64_t num = 64LL * (r->wa * a + r->wb * b) + r->B, q = num / r->T;

	if ((num % r->T) && ((num < 0) != (r->T < 0)))
		q--;					/* floor division */
	q += 128;
	return (uint8_t)(q < 0 ? 0 : q > 255 ? 255 : q);
}

/* ---- tensors and CPU ops */

struct tensor { int h, w, c, ps; uint8_t *u8; float *f; size_t cap; };	/* ps: bytes per pixel (0: c) */
static struct tensor T[MAXTENS];

/* tensor buffers live across runs: allocated on first use (page faults once, not per frame) */
static void need_u8(struct tensor *t, size_t n)
{
	if (t->cap < n) {
		free(t->u8);
		if (posix_memalign((void **)&t->u8, 64, (n + 63) & ~(size_t)63))
			DIE("out of memory");
		t->cap = n;
	}
}

static void need_f(struct tensor *t, size_t n)
{
	if (!t->f && !(t->f = malloc(n * sizeof(float))))
		DIE("out of memory");
}

static uint8_t qsum(int64_t S, int32_t B, int32_t Tv, int sh);
static uint8_t *pc_table(const struct qrec *r, int oc, int tap);

/* the clamp min(x, t) the thingino-ai T41 export builds from Add (x - relu(x - t)): tensor X is
 * read by a chain of per-channel 1x1 ops ending in N and by add(X, N). Every input of that add is a
 * function of X's byte, so the whole clamp becomes one table, composed into the conv's (tab).
 * Returns 1 when it folded one. */
static int clamp_fold(struct gnode *g, uint8_t *tab, const int *uses, int *nthis)
{
	int X = g->out, jp = -1, ja = -1, cur, N = -1, chain[8], nc = 0, aX;
	uint8_t ct[256], t2[256];

	if (uses[X] != 2)
		return 0;
	for (int k = (int)(g - G) + 1; k < ng; k++) {
		if (G[k].skip)
			continue;
		if (G[k].op == G_PC && G[k].in[0] == X && jp < 0)
			jp = k;
		if (G[k].op == G_ADD && (G[k].in[0] == X || G[k].in[1] == X) && ja < 0)
			ja = k;
	}
	if (jp < 0 || ja < 0)
		return 0;
	for (int x = 0; x < 256; x++)
		ct[x] = (uint8_t)x;
	for (cur = jp;;) {
		uint8_t *pt;
		int same = 1, next = -1;

		if (G[cur].oc != g->oc || nc == 8)
			return 0;
		pt = pc_table(&rec[G[cur].rec], g->oc, 0);
		for (int c = 1; c < g->oc && same; c++)
			same = !memcmp(pt, pt + 256 * c, 256);
		if (same)
			for (int x = 0; x < 256; x++)
				ct[x] = pt[ct[x]];
		free(pt);
		if (!same)
			return 0;
		chain[nc++] = cur;
		N = G[cur].out;
		if (uses[N] != 1)
			return 0;
		for (int k = cur + 1; k < ng && next < 0; k++)
			if (!G[k].skip && (G[k].in[0] == N || (G[k].nin > 1 && G[k].in[1] == N)))
				next = k;
		if (next == ja)
			break;
		if (next < 0 || G[next].op != G_PC || G[next].nin != 1)
			return 0;
		cur = next;
	}
	aX = G[ja].in[0] == X ? 0 : 1;
	if (G[ja].in[1 - aX] != N)
		return 0;
	for (int v = 0; v < 256; v++) {
		int xv = tab[v], nv = ct[xv];

		t2[v] = aX == 0 ? add_q(&addr[G[ja].crec], xv, nv) : add_q(&addr[G[ja].crec], nv, xv);
	}
	memcpy(tab, t2, 256);
	for (int k = 0; k < nc; k++)
		G[chain[k]].skip = 1;
	G[ja].skip = 1;
	g->out = G[ja].out;
	*nthis += nc + 1;
	return 1;
}

/* a 2x2 deconv record whose taps 0..2 are all zero (only output phase 0 reads the input) */
static int deconv_zero_insert(const struct qrec *r, int oc)
{
	if (r->kh != 2 || r->kw != 2 || !r->dw)
		return 0;
	for (int t = 0; t < 3; t++)
		for (int c = 0; c < oc; c++)
			if (r->wq[(long)t * 32 * r->nob + c])
				return 0;
	return 1;
}

/* fold per-channel 1x1 ops into the conv before them: conv -> pc -> pc ... becomes one conv whose
 * output bytes go through one composed table, when that table is the same for every channel and
 * each intermediate tensor has no other reader */
static int fold(void)
{
	static int uses[MAXTENS];
	int nf = 0;

	memset(uses, 0, sizeof(uses));

	for (int i = 0; i < ng; i++)
		for (int k = 0; k < G[i].nin; k++)
			uses[G[i].in[k]]++;
	for (int k = 0; k < nouts; k++)
		uses[outs[k]]++;
	for (int i = 0; i < ng; i++) {
		struct gnode *g = &G[i];
		uint8_t tab[256];
		int nthis = 0, ident = 1;

		if (g->op != G_CONV && g->op != G_STEM)
			continue;
		for (int x = 0; x < 256; x++)
			tab[x] = (uint8_t)x;
		for (;;) {
			for (;;) {
				int j = -1, same = 1;
				uint8_t *pt, t2[256];

				if (uses[g->out] != 1)
					break;
				for (int k = i + 1; k < ng && j < 0; k++)
					if (!G[k].skip && G[k].nin == 1 && G[k].in[0] == g->out)
						j = k;
				if (j < 0 || G[j].op != G_PC || G[j].oc != g->oc)
					break;
				pt = pc_table(&rec[G[j].rec], g->oc, 0);
				for (int c = 1; c < g->oc && same; c++)
					same = !memcmp(pt, pt + 256 * c, 256);
				if (same)
					for (int x = 0; x < 256; x++)
						t2[x] = pt[tab[x]];
				free(pt);
				if (!same)
					break;
				memcpy(tab, t2, 256);
				G[j].skip = 1;
				g->out = G[j].out;
				nthis++;
			}
			if (!clamp_fold(g, tab, uses, &nthis))
				break;
		}
		for (int x = 0; x < 256 && ident; x++)
			ident = tab[x] == x;
		if (nthis && !ident) {
			if (posix_memalign((void **)&g->post, 64, 256))	/* 64-aligned: VPR loads */
				DIE("out of memory");
			memcpy(g->post, tab, 256);
		}
		nf += nthis;
	}
	/* a zero-insert deconv read only by a concat: the concat builds that input itself. Only taps 3
	 * (phase 0) may be non-zero: the other three phases are then constants. A nearest-neighbour
	 * deconv (all four taps, YOLO-style upsampling) stays an op of its own */
	for (int c = 0; c < ng; c++)
		for (int k = 0; G[c].op == G_CAT && k < G[c].nin; k++)
			for (int j = 0; j < c; j++)
				if (G[j].op == G_DECONV && !G[j].skip && G[j].out == G[c].in[k] && uses[G[j].out] == 1 &&
				    deconv_zero_insert(&rec[G[j].rec], G[j].oc)) {
					G[j].skip = 1;
					G[c].dec[k] = j + 1;
					nf++;
				}
	return nf;
}

static uint8_t qsum(int64_t S, int32_t B, int32_t Tv, int sh)
{
	int64_t r = S * ((int64_t)1 << sh) + B, t = Tv & 0x3ffffff, q;

	if (!t)
		return r >= 0 ? 255 : 0;
	q = r / t - (r % t && r < 0);
	q += 128;
	return (uint8_t)(q < 0 ? 0 : q > 255 ? 255 : q);
}

static const int8_t STEM[32][3] = {	/* stem slot -> c, ky, kx (-1: unused) */
	{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 0, 1}, {2, 0, 1}, {1, 1, 0},
	{0, 0, 2}, {1, 0, 2}, {2, 0, 2}, {2, 1, 0}, {0, 1, 1}, {1, 1, 1}, {2, 1, 1}, {-1, 0, 0},
	{0, 2, 0}, {1, 2, 0}, {2, 2, 0}, {-1, 0, 0}, {0, 2, 1}, {1, 2, 1}, {2, 2, 1}, {-1, 0, 0},
	{0, 2, 2}, {1, 2, 2}, {2, 2, 2}, {-1, 0, 0}, {0, 1, 2}, {1, 1, 2}, {2, 1, 2}, {-1, 0, 0},
};

/* the stem's 32-byte pixels (3x3/2, pad bottom/right 1) into f (OH x OW x 32, zeroed once: the
 * unused slots are never written). Interior pixels: 27 bytes at fixed offsets from the window */
static void stem_f(const uint8_t *rgb, int H, int W, uint8_t *f)
{
	int OH = H / 2, OW = W / 2, off[32], slot[32], n = 0, unrolled;

	for (int i = 0; i < 32; i++)
		if (STEM[i][0] >= 0)
			slot[n] = i, off[n++] = (STEM[i][1] * W + STEM[i][2]) * 3 + STEM[i][0];
	/* the unrolled copy below is the STEM table; check that it is */
	unrolled = n == 27;
	for (int k = 0; k < n && unrolled; k++) {
		static const int8_t u[32][2] = {	/* slot -> row (0..2), byte in the row's 9 */
			{0, 0}, {0, 1}, {0, 2}, {1, 0}, {0, 3}, {0, 4}, {0, 5}, {1, 1}, {0, 6}, {0, 7}, {0, 8}, {1, 2},
			{1, 3}, {1, 4}, {1, 5}, {-1, 0}, {2, 0}, {2, 1}, {2, 2}, {-1, 0}, {2, 3}, {2, 4}, {2, 5}, {-1, 0},
			{2, 6}, {2, 7}, {2, 8}, {-1, 0}, {1, 6}, {1, 7}, {1, 8}, {-1, 0} };

		unrolled = u[slot[k]][0] >= 0 && off[k] == u[slot[k]][0] * 3 * W + u[slot[k]][1];
	}
	for (int y = 0; y < OH; y++) {
		const uint8_t *row = rgb + 2L * y * W * 3;
		uint8_t *d = f + (long)y * OW * 32;
		int xe = 2 * y + 2 < H ? OW - 1 : 0;	/* the last row and column take the checked path */

		for (int x = 0; x < xe; x++, d += 32) {
			const uint8_t *s = row + 6L * x, *s1 = s + 3L * W, *s2 = s1 + 3L * W;

			if (unrolled) {	/* the STEM table, spelled out */
				d[0] = s[0], d[1] = s[1], d[2] = s[2], d[3] = s1[0];
				d[4] = s[3], d[5] = s[4], d[6] = s[5], d[7] = s1[1];
				d[8] = s[6], d[9] = s[7], d[10] = s[8], d[11] = s1[2];
				d[12] = s1[3], d[13] = s1[4], d[14] = s1[5];
				d[16] = s2[0], d[17] = s2[1], d[18] = s2[2];
				d[20] = s2[3], d[21] = s2[4], d[22] = s2[5];
				d[24] = s2[6], d[25] = s2[7], d[26] = s2[8];
				d[28] = s1[6], d[29] = s1[7], d[30] = s1[8];
			} else {
				for (int k = 0; k < n; k++)
					d[slot[k]] = s[off[k]];
			}
		}
		for (int x = xe; x < OW; x++, d += 32)
			for (int i = 0; i < 32; i++) {
				int c = STEM[i][0], iy = 2 * y + STEM[i][1], ix = 2 * x + STEM[i][2];

				d[i] = c >= 0 && iy < H && ix < W ? rgb[((long)iy * W + ix) * 3 + c] : 0;
			}
	}
}

/* CPU conv: int sums [OH][OW][oc]; dense 1x1 or depthwise kxk; stride st, pad pt top/left and pb
 * bottom/right with byte 0 (the zero code of these ReLU tensors) */
static void cpu_sums(const struct qrec *r, const uint8_t *in, int H, int W, int C, int ips, int oc, int k,
		     int pt, int pb, int st, int64_t *S)
{
	int OH = (H + pt + pb - k) / st + 1, OW = (W + pt + pb - k) / st + 1;

	for (int y = 0; y < OH; y++)
		for (int x = 0; x < OW; x++) {
			int64_t *s = S + ((long)y * OW + x) * oc;

			memset(s, 0, sizeof(*s) * (size_t)oc);
			if (r->dw) {
				for (int ky = 0; ky < k; ky++)
					for (int kx = 0; kx < k; kx++) {
						int iy = y * st + ky - pt, ix = x * st + kx - pt;
						const int8_t *w = r->wq + (long)(ky * r->kw + kx) * 32 * r->nob;

						if (iy < 0 || iy >= H || ix < 0 || ix >= W)
							continue;
						for (int c = 0; c < oc; c++)
							s[c] += (int64_t)w[c] * in[((long)iy * W + ix) * ips + c];
					}
			} else {		/* dense k x k: wq [t][oc][ic] */
				for (int ky = 0; ky < k; ky++)
					for (int kx = 0; kx < k; kx++) {
						int iy = y * st + ky - pt, ix = x * st + kx - pt;
						const uint8_t *px = in + ((long)iy * W + ix) * ips;

						if (iy < 0 || iy >= H || ix < 0 || ix >= W)
							continue;
						for (int o = 0; o < oc; o++) {
							const int8_t *w = r->wq + ((long)(ky * k + kx) * 32 * r->nob + o) * 32 * r->nic;
							int64_t a = 0;

							for (int c = 0; c < C; c++)
								a += (int64_t)w[c] * px[c];
							s[o] += a;
						}
					}
			}
		}
}

static void qsum_all(const struct qrec *r, const int64_t *S, long npx, int oc, uint8_t *q)
{
	for (long p = 0; p < npx; p++)
		for (int c = 0; c < oc; c++)
			q[p * oc + c] = qsum(S[p * oc + c], r->bt[64 * (c / 32) + c % 32], r->bt[64 * (c / 32) + 32 + c % 32], r->sh);
}

/* per-channel byte -> byte table: q(w x) */
static uint8_t *pc_table(const struct qrec *r, int oc, int tap)
{
	uint8_t *t = malloc(256L * oc);

	for (int c = 0; c < oc; c++)
		for (int x = 0; x < 256; x++)
			t[c * 256 + x] = qsum((int64_t)r->wq[(long)tap * 32 * r->nob + c] * x, r->bt[64 * (c / 32) + c % 32],
					      r->bt[64 * (c / 32) + 32 + c % 32], r->sh);
	return t;
}

static double now_ms(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts.tv_sec * 1e3 + ts.tv_nsec / 1e6;
}

static struct nna_be be;
static int use_nna, check;
static uint8_t *tabs[MAXNODE];		/* per node: pc / deconv tables, built once */
static void *cfuse[MAXNODE];		/* per concat node: its tables (struct catf) */
static int8_t *wtrim[MAXREC];		/* per record: weights trimmed to the conv's oc */
static uint8_t *stem_buf;
static uint8_t conv_readers_only[MAXTENS];	/* tensor read by convs / heads only (after folding) */		/* the stem's 32-byte pixels (unused slots stay 0) */

/* which tensors only convs / heads read (after folding): those may be stored as 32-byte pixels */
static void set_readers(void)
{
	for (int t = 0; t < MAXTENS; t++)
		conv_readers_only[t] = 1;
	for (int k = 0; k < nouts; k++)
		conv_readers_only[outs[k]] = 0;
	for (int i = 0; i < ng; i++)
		if (!G[i].skip && G[i].op != G_CONV && G[i].op != G_HEAD)
			for (int k = 0; k < G[i].nin; k++)
				conv_readers_only[G[i].in[k]] = 0;
}

static void conv_op(const struct gnode *g, const struct qrec *r, const uint8_t *in, int H, int W, int C, int ips,
		    struct tensor *o, int head)
{
	int k = g->k ? g->k : 1, pad = g->pad, st = g->stride ? g->stride : 1;
	int pb = g->op == G_CONV ? g->padb : pad;
	int OH = (H + pad + pb - k) / st + 1, OW = (W + pad + pb - k) / st + 1, oc = g->oc;
	long npx = (long)OH * OW;
	int64_t *S = NULL;
	/* stride 2 / bottom-right-only padding (stride-2 depthwise): CPU sums, requantized as the NNA
	 * does (qsum), until the NNA driver takes them */
	int nna = use_nna && st == 1 && pb == pad && (head || r->sh == 2);	/* the NNA's QSUM is x4 */

	/* on the NNA a 16-channel output is stored as 32-byte pixels: the next conv then writes F
	 * straight from the tensor (its weights for channels 16..31 are 0). Only when convs alone read
	 * it (an add, pc, pool or concat reads packed pixels) */
	int ops = nna && !head && oc == 16 && conv_readers_only[g->out] ? 32 : oc;

	o->h = OH, o->w = OW, o->c = oc, o->ps = ops;
	if (head)
		need_f(o, (size_t)(npx * oc));
	else
		need_u8(o, (size_t)(npx * ops));
	if (!nna || check) {
		S = malloc(sizeof(*S) * (size_t)(npx * oc));
		cpu_sums(r, in, H, W, C, ips, oc, k, pad, pb, st, S);
	}
	if (nna) {
		struct nna_node nd;
		struct nna_rq rq;
		int oh, ow;
		int32_t *S32 = head ? malloc(sizeof(int32_t) * (size_t)(npx * 32 * r->nob)) : NULL;

		memset(&nd, 0, sizeof(nd));
		memset(&rq, 0, sizeof(rq));
		nd.rec = g->rec, nd.oc = oc, nd.ic = r->dw ? 1 : 32 * r->nic, nd.kh = nd.kw = k, nd.stride = 1;
		nd.pad = pad, nd.padval = 0, nd.dw = r->dw, nd.wq = r->wq;
		/* nna_conv reads wq as [ky][kx][oc][ic] with oc = nd.oc; the records have oc = 32 nob */
		if (oc != 32 * r->nob) {
			int taps = k * k;

			if (!wtrim[g->rec]) {
				wtrim[g->rec] = malloc((size_t)taps * oc * nd.ic);
				for (int t = 0; t < taps; t++)
					memcpy(wtrim[g->rec] + (long)t * oc * nd.ic, r->wq + (long)t * 32 * r->nob * nd.ic,
					       (size_t)oc * nd.ic);
			}
			nd.wq = wtrim[g->rec];
		}
		rq.q = o->u8, rq.bt = r->bt, rq.post = g->post, rq.ps = ops;
		if (nna_conv(&be, in, H, W, C, ips, &nd, S32, head ? NULL : &rq, &oh, &ow) || oh != OH || ow != OW)
			DIE("rec %d: nna_conv failed", g->rec);
		if (head) {
			for (long p = 0; p < npx; p++)
				for (int c = 0; c < oc; c++) {
					int32_t s = S32[p * oc + c];

					if (check && s != S[p * oc + c])
						DIE("rec %d (head): sum differs at px %ld oc %d", g->rec, p, c);
					o->f[p * oc + c] = (float)((int64_t)s + r->bt[64 * (c / 32) + c % 32]) * r->fl[c];
				}
			free(S32);
		} else if (check) {
			uint8_t *ref = malloc((size_t)(npx * oc));

			qsum_all(r, S, npx, oc, ref);
			if (g->post)
				for (long i = 0; i < npx * oc; i++)
					ref[i] = g->post[ref[i]];
			for (long i = 0; i < npx * oc; i++)
				if (ref[i] != o->u8[i / oc * ops + i % oc])
					DIE("rec %d: NNA byte %d, CPU %d at px %ld oc %ld", g->rec, o->u8[i / oc * ops + i % oc],
					    ref[i], i / oc, i % oc);
			free(ref);
		}
	} else if (head) {
		for (long p = 0; p < npx; p++)
			for (int c = 0; c < oc; c++)
				o->f[p * oc + c] = (float)(S[p * oc + c] + r->bt[64 * (c / 32) + c % 32]) * r->fl[c];
	} else {
		qsum_all(r, S, npx, oc, o->u8);
		if (g->post)
			for (long i = 0; i < npx * oc; i++)
				o->u8[i] = g->post[o->u8[i]];
	}
	free(S);
}

#ifdef ONNA_MXU3
#include "mxu3.h"
/* 2x2/2 byte max with MAXU.B, 64 bytes per op. C = 32: one VPR gathers octwords [p0 | p2], another
 * [p1 | p3] of a row (LAO's immediate offsets), so 4 maxes give 2 output pixels. C % 64 = 0: the max
 * of 4 whole pixels per 64 bytes. Needs 64-byte aligned rows; else 0 (the scalar loop runs) */
static int pool2_mxu3(const struct tensor *x, struct tensor *o)
{
	long rs = (long)x->w * x->c, ors = (long)o->w * o->c;

	if ((x->c != 32 && x->c % 64) || (rs & 63) || (ors & 63) || ((uintptr_t)x->u8 & 63) ||
	    ((uintptr_t)o->u8 & 63) || (x->c == 32 && o->w % 2))
		return 0;
	for (int y = 0; y < o->h; y++) {
		const uint8_t *a = x->u8 + 2L * y * rs, *b = a + rs;
		uint8_t *d = o->u8 + y * ors;

		if (x->c == 32) {
			for (int xx = 0; xx < o->w; xx += 2, a += 128, b += 128, d += 64) {
				register const void *ra __asm__("$9") = a;
				register const void *rb __asm__("$10") = b;

				__asm__ volatile(MXU3_WORD(LAO_W(9, 0, 0, 1)) MXU3_WORD(LAO_W(9, 2, 1, 1))
						 MXU3_WORD(LAO_W(9, 1, 0, 2)) MXU3_WORD(LAO_W(9, 3, 1, 2))
						 MXU3_WORD(LAO_W(10, 0, 0, 3)) MXU3_WORD(LAO_W(10, 2, 1, 3))
						 MXU3_WORD(LAO_W(10, 1, 0, 4)) MXU3_WORD(LAO_W(10, 3, 1, 4))
						 MXU3_WORD(MAXUB_W(1, 1, 2)) MXU3_WORD(MAXUB_W(3, 3, 4))
						 MXU3_WORD(MAXUB_W(1, 1, 3))
						 :: "r"(ra), "r"(rb) : "memory");
				VSTORE(1, d);
			}
		} else {
			for (int xx = 0; xx < o->w; xx++, a += 2 * x->c, b += 2 * x->c)
				for (int v = 0; v < x->c; v += 64, d += 64) {
					VLOAD(1, a + v);
					VLOAD(2, a + x->c + v);
					VLOAD(3, b + v);
					VLOAD(4, b + x->c + v);
					__asm__ volatile(MXU3_WORD(MAXUB_W(1, 1, 2)) MXU3_WORD(MAXUB_W(3, 3, 4))
							 MXU3_WORD(MAXUB_W(1, 1, 3)) ::: "memory");
					VSTORE(1, d);
				}
		}
	}
	return 1;
}
#endif

static double optime[8], nodetime[MAXNODE];	/* -T: per op kind, summed over the runs */
static int timing;

static void run_op(int i);

/* ONNAQ_DUMP=dir: each op's output tensor as dir/t<id>.u8 (h w c, then the bytes, pixels packed) */
static void dump_tensor(const char *dir, int id)
{
	const struct tensor *t = &T[id];
	char path[512];
	FILE *f;
	int ps = t->ps ? t->ps : t->c;

	if (!t->u8 || t->f)
		return;
	snprintf(path, sizeof(path), "%s/t%d.u8", dir, id);
	if (!(f = fopen(path, "wb")))
		return;
	fwrite(&t->h, sizeof(int), 1, f), fwrite(&t->w, sizeof(int), 1, f), fwrite(&t->c, sizeof(int), 1, f);
	for (long p = 0; p < (long)t->h * t->w; p++)
		fwrite(t->u8 + p * ps, 1, (size_t)t->c, f);
	fclose(f);
}

static void run(const uint8_t *rgb)
{
	const char *dump = getenv("ONNAQ_DUMP");

	T[0].h = in_h, T[0].w = in_w, T[0].c = 3, T[0].u8 = (uint8_t *)rgb;
	for (int i = 0; i < ng; i++) {
		double t0 = timing ? now_ms() : 0;

		run_op(i);
		if (dump && !G[i].skip)
			dump_tensor(dump, G[i].out);
		if (timing) {
			optime[G[i].op] += now_ms() - t0;
			nodetime[i] += now_ms() - t0;
		}
	}
}

static void run_op(int i)
{
	{
		const struct gnode *g = &G[i];
		struct tensor *x = &T[g->in[0]], *o = &T[g->out];
		const struct qrec *r = g->op == G_POOL || g->op == G_CAT || g->op == G_ADD ? NULL : &rec[g->rec];
		long npx = (long)x->h * x->w;

		if (g->skip)
			return;
		for (int k = 0; k < g->nin; k++)
			if (g->op != G_CONV && g->op != G_HEAD && T[g->in[k]].ps && T[g->in[k]].ps != T[g->in[k]].c)
				DIE("node %d reads a %d-channel tensor stored with %d-byte pixels", i, T[g->in[k]].c,
				    T[g->in[k]].ps);
		o->ps = 0;
		switch (g->op) {
		case G_STEM: {
			uint8_t *f = stem_buf;
			struct gnode g1 = *g;

			if (!f) {		/* zeroed once: the unused slots stay so */
				if (posix_memalign((void **)&stem_buf, 64, (size_t)(x->h / 2) * (x->w / 2) * 32))
					DIE("out of memory");
				f = stem_buf;
				memset(f, 0, (size_t)(x->h / 2) * (x->w / 2) * 32);
			}
			stem_f(x->u8, x->h, x->w, f);

			g1.k = 1, g1.pad = 0;
			if (r->dw || r->nic != 1 || r->kh != 1)
				DIE("stem record %d", g->rec);
			conv_op(&g1, r, f, x->h / 2, x->w / 2, 32, 32, o, 0);
			break;
		}
		case G_CONV:
		case G_HEAD:
			if (x->c != (r->dw ? g->oc : g->ic) || (g->op == G_CONV && r->kh != (g->k ? g->k : 1)))
				DIE("node %d: shapes (in %d, rec k %d)", i, x->c, r->kh);
			conv_op(g, r, x->u8, x->h, x->w, x->c, x->ps ? x->ps : x->c, o, g->op == G_HEAD);
			break;
		case G_PC:
			if (!tabs[i])
				tabs[i] = pc_table(r, g->oc, 0);
			o->h = x->h, o->w = x->w, o->c = x->c;
			need_u8(o, (size_t)(npx * x->c));
			for (long p = 0; p < npx; p++)
				for (int c = 0; c < x->c; c++)
					o->u8[p * x->c + c] = tabs[i][c * 256 + x->u8[p * x->c + c]];
			break;
		case G_ADD: {
			const struct tensor *y = &T[g->in[1]];

			if (y->h != x->h || y->w != x->w || y->c != x->c)
				DIE("node %d: add of %dx%dx%d and %dx%dx%d", i, x->h, x->w, x->c, y->h, y->w, y->c);
			if (!tabs[i]) {		/* the whole (a, b) -> out table, built once */
				if (!(tabs[i] = malloc(65536)))
					DIE("out of memory");
				for (int a = 0; a < 256; a++)
					for (int b = 0; b < 256; b++)
						tabs[i][a << 8 | b] = add_q(&addr[g->crec], a, b);
			}
			o->h = x->h, o->w = x->w, o->c = x->c;
			need_u8(o, (size_t)(npx * x->c));
			for (long p = 0; p < npx * x->c; p++)
				o->u8[p] = tabs[i][x->u8[p] << 8 | y->u8[p]];
			break;
		}
		case G_POOL:
			o->h = x->h / 2, o->w = x->w / 2, o->c = x->c;
			need_u8(o, (size_t)o->h * o->w * o->c);
#ifdef ONNA_MXU3
			if (pool2_mxu3(x, o))
				break;
#endif
			for (int y = 0; y < o->h; y++)
				for (int xx = 0; xx < o->w; xx++)
					for (int c = 0; c < o->c; c++) {
						const uint8_t *a = x->u8 + ((long)(2 * y) * x->w + 2 * xx) * x->c + c;
						uint8_t m = a[0];

						m = a[x->c] > m ? a[x->c] : m;
						m = a[(long)x->w * x->c] > m ? a[(long)x->w * x->c] : m;
						m = a[(long)x->w * x->c + x->c] > m ? a[(long)x->w * x->c + x->c] : m;
						o->u8[((long)y * o->w + xx) * o->c + c] = m;
					}
			break;
		case G_DECONV: {
			/* ConvTranspose 2x2/2, stored flipped: output (2y + dy, 2x + dx) uses tap (1 - dy, 1 - dx).
			 * Each output is q(w x): a table per channel and phase */
			if (r->kh != 2 || r->kw != 2 || !r->dw)
				DIE("deconv record %d", g->rec);
			if (!tabs[i]) {
				tabs[i] = malloc(4 * 256L * g->oc);
				for (int t = 0; t < 4; t++) {
					uint8_t *tt = pc_table(r, g->oc, t);

					memcpy(tabs[i] + t * 256L * g->oc, tt, 256L * g->oc);
					free(tt);
				}
			}
			o->h = 2 * x->h, o->w = 2 * x->w, o->c = x->c;
			need_u8(o, (size_t)o->h * o->w * o->c);
			for (int y = 0; y < o->h; y++)
				for (int xx = 0; xx < o->w; xx++) {
					int t = (1 - y % 2) * 2 + (1 - xx % 2);
					const uint8_t *tb = tabs[i] + t * 256L * g->oc, *s = x->u8 + ((long)(y / 2) * x->w + xx / 2) * x->c;
					uint8_t *d = o->u8 + ((long)y * o->w + xx) * o->c;

					for (int c = 0; c < x->c; c++)
						d[c] = tb[c * 256 + s[c]];
				}
			break;
		}
		case G_CAT: {
			/* per input: a byte table rint(x s_i / s_out) (a memcpy when it is the identity); an
			 * input from a folded deconv is built here: even (y, x) through the deconv's per-channel
			 * table and the concat's, the other three phases a constant per channel */
			struct catf { int ident[4]; uint8_t *comp[4], *cst[4][3]; } *cf = cfuse[i];
			int ctot = 0, off = 0, h = -1, w = -1;

			for (int k = 0; k < g->nin; k++) {
				const struct tensor *a = g->dec[k] ? &T[G[g->dec[k] - 1].in[0]] : &T[g->in[k]];
				int ah = g->dec[k] ? 2 * a->h : a->h, aw = g->dec[k] ? 2 * a->w : a->w;

				if (h < 0)
					h = ah, w = aw;
				if (ah != h || aw != w)
					DIE("concat %d: input sizes", i);
				ctot += a->c;
			}
			if (!cf) {
				cf = cfuse[i] = calloc(1, sizeof(*cf));
				for (int k = 0; k < g->nin; k++) {
					uint8_t tb[256];

					cf->ident[k] = 1;
					for (int v = 0; v < 256; v++) {
						float y = rintf((float)v * cat_a[g->crec][k]);

						tb[v] = (uint8_t)(y < 0 ? 0 : y > 255 ? 255 : y);
						cf->ident[k] &= tb[v] == v;
					}
					if (g->dec[k]) {
						const struct gnode *dn = &G[g->dec[k] - 1];
						const struct qrec *dr = &rec[dn->rec];
						int C = dn->oc;

						if (dr->kh != 2 || dr->kw != 2 || !dr->dw)
							DIE("deconv record %d", dn->rec);
						cf->comp[k] = malloc(256L * C);
						for (int ph = 0; ph < 4; ph++) {	/* output phase (dy, dx) uses tap 3 - ph */
							uint8_t *pt = pc_table(dr, C, 3 - ph);

							if (ph)
								cf->cst[k][ph - 1] = malloc((size_t)(C & 0xffff));
							for (int c = 0; c < C; c++)
								for (int v = 0; v < 256; v++) {
									uint8_t y = tb[pt[c * 256 + v]];

									if (!ph)
										cf->comp[k][c * 256 + v] = y;
									else if (!v)
										cf->cst[k][ph - 1][c] = y;
									else if (y != cf->cst[k][ph - 1][c])
										DIE("deconv %d: phase %d depends on the input", dn->rec, ph);
								}
							free(pt);
						}
					} else if (!cf->ident[k]) {
						cf->comp[k] = malloc(256);
						memcpy(cf->comp[k], tb, 256);
					}
				}
			}
			o->h = h, o->w = w, o->c = ctot;
			npx = (long)h * w;
			need_u8(o, (size_t)(npx * ctot));
			for (int k = 0; k < g->nin; k++) {
				if (g->dec[k]) {
					const struct tensor *a = &T[G[g->dec[k] - 1].in[0]];
					int C = a->c;

					for (int y = 0; y < h; y++)
						for (int xx = 0; xx < w; xx++) {
							uint8_t *d = o->u8 + ((long)y * w + xx) * ctot + off;
							int ph = (y & 1) * 2 + (xx & 1);

							if (ph) {
								memcpy(d, cf->cst[k][ph - 1], (size_t)C);
								continue;
							}
							const uint8_t *sp = a->u8 + ((long)(y / 2) * a->w + xx / 2) * C;

							for (int c = 0; c < C; c++)
								d[c] = cf->comp[k][c * 256 + sp[c]];
						}
					off += C;
					continue;
				}
				const struct tensor *a = &T[g->in[k]];

				for (long p = 0; p < npx; p++) {
					uint8_t *d = o->u8 + p * ctot + off;
					const uint8_t *sp = a->u8 + p * a->c;

					if (cf->ident[k])
						memcpy(d, sp, (size_t)a->c);
					else
						for (int c = 0; c < a->c; c++)
							d[c] = cf->comp[k][sp[c]];
				}
				off += a->c;
			}
			break;
		}
		}
	}
}

/* ---- library API (onnaq.h) */

struct catf { int ident[4]; uint8_t *comp[4], *cst[4][3]; };

const char *onnaq_error(void)
{
	return die_msg;
}

void onnaq_close(void)
{
	for (int i = 0; i < 512; i++) {
		struct catf *cf = cfuse[i];

		if (cf) {
			for (int k = 0; k < 4; k++) {
				free(cf->comp[k]);
				for (int ph = 0; ph < 3; ph++)
					free(cf->cst[k][ph]);
			}
			free(cf);
		}
		free(tabs[i]);
		free(G[i].post);
		tabs[i] = NULL, cfuse[i] = NULL;
	}
	for (int i = 0; i < 256; i++) {
		if (i < nrec) {
			free(rec[i].wq);
			free(rec[i].bt);
			free(rec[i].fl);
		}
		free(wtrim[i]);
		wtrim[i] = NULL;
		if (i)			/* T[0] is the caller's frame */
			free(T[i].u8);
		free(T[i].f);
	}
	free(stem_buf);
	stem_buf = NULL;
	memset(T, 0, sizeof(T));
	memset(G, 0, sizeof(G));
	memset(rec, 0, sizeof(rec));
	nrec = ncat = ng = ntens = nouts = nanc = in_h = in_w = 0;
	use_nna = 0;
}

int onnaq_open(const char *bin_path, const char *graph_path, int nna)
{
	jmp_buf jb;

	onnaq_close();
	die_msg[0] = 0;
	die_jb = &jb;
	if (setjmp(jb)) {
		die_jb = NULL;
		onnaq_close();
		return -1;
	}
	if (nna) {
#ifdef ONNA_MXU3
		nna_be_mxu3(&be);
		nna_pin_isp_cpu0();
		use_nna = 1;
#else
		DIE("not a T41 build: no NNA");
#endif
	}
	load_bin(bin_path);
	load_graph(graph_path);
	fold();
	set_readers();
	die_jb = NULL;
	return 0;
}

void onnaq_input_size(int *w, int *h)
{
	*w = in_w, *h = in_h;
}

int onnaq_run(const uint8_t *rgb)
{
	jmp_buf jb;

	if (!ng) {
		snprintf(die_msg, sizeof(die_msg), "no model");
		return -1;
	}
	die_jb = &jb;
	if (setjmp(jb)) {
		die_jb = NULL;
		return -1;
	}
	run(rgb);
	die_jb = NULL;
	return 0;
}

int onnaq_num_heads(void)
{
	return nouts;
}

const float *onnaq_head(int k, int *h, int *w, int *c)
{
	const struct tensor *t;

	if (k < 0 || k >= nouts)
		return NULL;
	t = &T[outs[k]];
	*h = t->h, *w = t->w, *c = t->c;
	return t->f;
}

int onnaq_anchors(int stride, float wh[6])
{
	static const float yolo[3][6] = { { 10, 13, 16, 30, 33, 23 }, { 30, 61, 62, 45, 59, 119 },
					  { 116, 90, 156, 198, 373, 326 } };

	for (int i = 0; i < nanc; i++)
		if (anc_stride[i] == stride) {
			memcpy(wh, anc_wh[i], sizeof(anc_wh[0]));
			return 0;
		}
	if (nanc || (stride != 8 && stride != 16 && stride != 32))
		return -1;
	memcpy(wh, yolo[stride == 8 ? 0 : stride == 16 ? 1 : 2], sizeof(yolo[0]));
	return 0;
}

#ifndef ONNAQ_LIB
int main(int argc, char **argv)
{
	int ai = 1, loops = 1;
	const char *eng = "cpu";

	while (ai < argc && argv[ai][0] == '-') {
		if (!strcmp(argv[ai], "-e") && ai + 1 < argc)
			eng = argv[++ai];
		else if (!strcmp(argv[ai], "-c"))
			check = 1;
		else if (!strcmp(argv[ai], "-T"))
			timing = 1;
		else if (!strcmp(argv[ai], "-TT"))
			timing = 2;
		else if (!strcmp(argv[ai], "-n") && ai + 1 < argc)
			loops = atoi(argv[++ai]);
		else
			break;
		ai++;
	}
	if (argc - ai != 4) {
		fprintf(stderr, "usage: %s [-e cpu|oracle|nna] [-c] [-T] [-n N] model.bin model.qgraph frame.rgb prefix\n", argv[0]);
		return 2;
	}
	if (!strcmp(eng, "oracle")) {
#ifdef ONNA_ORACLE
		nna_be_oracle(&be);
		use_nna = 1;
#else
		DIE("built without the oracle (make ORACLE=1)");
#endif
	} else if (!strcmp(eng, "nna")) {
#ifdef ONNA_MXU3
		cpu_set_t cs;

		CPU_ZERO(&cs);
		CPU_SET(0, &cs);
		if (sched_setaffinity(0, sizeof(cs), &cs))
			DIE("cannot pin to CPU0");
		nna_be_mxu3(&be);
		nna_pin_isp_cpu0();
		use_nna = 1;
#else
		DIE("not a T41 build");
#endif
	} else if (strcmp(eng, "cpu"))
		DIE("engine %s", eng);
	mallopt(M_MMAP_THRESHOLD, 64 << 20);	/* tensors from the heap, not fresh mmaps */
	mallopt(M_TRIM_THRESHOLD, 128 << 20);
	load_bin(argv[ai]);
	load_graph(argv[ai + 1]);
	int nfold = getenv("ONNAQ_NOFOLD") ? 0 : fold();

	set_readers();

	fprintf(stderr, "onnaq: %d W blocks, %d concat records, %d ops (%d folded into convs), input %dx%d\n", nrec,
		ncat, ng, nfold, in_w, in_h);

	size_t fsz = (size_t)in_h * in_w * 3;
	uint8_t *rgb = malloc(fsz);
	FILE *f = fopen(argv[ai + 2], "rb");

	if (!f || fread(rgb, 1, fsz, f) != fsz)
		DIE("cannot read %d x %d x 3 bytes from %s", in_w, in_h, argv[ai + 2]);
	fclose(f);

	float *first[4] = { 0 };
	double best = 1e30, tot = 0;

	for (int it = 0; it < loops; it++) {
		double t0 = now_ms(), dt;

		run(rgb);
		dt = now_ms() - t0;
		tot += dt;
		best = dt < best ? dt : best;
		for (int k = 0; k < nouts; k++) {
			const struct tensor *h = &T[outs[k]];
			size_t n = (size_t)h->h * h->w * h->c;

			if (!first[k]) {
				first[k] = malloc(n * sizeof(float));
				memcpy(first[k], h->f, n * sizeof(float));
			} else if (memcmp(first[k], h->f, n * sizeof(float))) {
				DIE("run %d: head %d differs from run 0", it, k);
			}
		}
	}
	for (int k = 0; k < nouts; k++) {
		const struct tensor *h = &T[outs[k]];
		char path[512];
		FILE *o;

		snprintf(path, sizeof(path), "%s.%d.f32", argv[ai + 3], k);
		o = fopen(path, "wb");
		if (!o || fwrite(h->f, sizeof(float), (size_t)h->h * h->w * h->c, o) != (size_t)h->h * h->w * h->c)
			DIE("cannot write %s", path);
		fclose(o);
		printf("head %d: %d x %d x %d -> %s\n", k, h->h, h->w, h->c, path);
	}
	if (timing) {
		static const char *nm[] = { "stem", "conv", "pc", "head", "maxpool2", "deconv2", "concat", "add" };

		for (int k = 0; k < 8; k++)
			printf("  %-9s %7.2f ms/run\n", nm[k], optime[k] / loops);
		if (timing > 1)		/* -TT: per node */
			for (int i = 0; i < ng; i++)
				if (!G[i].skip)
					printf("  node %2d %-8s rec %2d %3dx%3dx%3d -> %3d  %6.2f ms\n", i, nm[G[i].op], G[i].rec,
					       T[G[i].in[0]].h, T[G[i].in[0]].w, T[G[i].in[0]].c, T[G[i].out].c, nodetime[i] / loops);
	}
	printf("%d run%s: %.2f ms avg, %.2f ms best (%s)%s\n", loops, loops > 1 ? "s" : "", tot / loops, best, eng,
	       loops > 1 ? ", heads identical in every run" : "");
	return 0;
}
#endif
