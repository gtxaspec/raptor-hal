/* nnaeng: the T41 NNA conv engine driver. See nnaeng.h.
 *
 * Tiling (v1, simple and correct; speed comes later): one MAC per 2x4 output tile (0x38 =
 * 0x12, as venus; the stem uses venus's one-row mode 0x3). For each tile the input window
 * ((sy + kh) rows x pitch pixels x planes) is written linearly into F RAM from address 0; the
 * engine's bounds check (0x2b/0x29 vs 0x2e/0x2c) supplies the padding.
 *
 * F RAM on silicon is sliced (as QEMU models only for the T40): a written byte is stored as
 * four 2-bit slices at lines l, l+d, l+2d, l+3d with d = 0x37/4, and a read sums them. So a
 * plane's data must fit in its first 0x37/4 lines, planes sit 0x37 lines apart, and each plane
 * is written separately from line k*0x37. (QEMU's T41 model stores bytes: the same result as
 * long as nothing collides.)
 * Weights: per 32-output-channel block, the groups (plane k, tap in snake order) go to W RAM
 * from entry 0. Sums: CPOC|CLROC, then 16 NNDRD = 8 pixels x 32 channels.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: MIT
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "nnaeng.h"
#include "nnacpu.h"
#ifdef ONNA_MXU3
#include "mxu3.h"
#endif

#define ERR(...) do { fprintf(stderr, "nnaeng: " __VA_ARGS__); fputc('\n', stderr); return -1; } while (0)

enum {
	R_FY = 0x29, R_FX = 0x2b, R_FH = 0x2c, R_FW = 0x2e, R_PAD = 0x2f, R_WRD = 0x30,
	R_PLANE = 0x37, R_MACTL = 0x38, R_PITCH = 0x39, R_STRIDE = 0x3a, R_FMT = 0x3b,
	R_NPLANE = 0x3c, R_TAPS = 0x3e, R_PXMASK = 0x3f, R_MSF = 0x46, R_FLINE = 0x50,
	R_VEC = 0x55, R_WFCWN = 0x5a, R_WWR = 0x5c,
};
enum { C_CPOC = 1, C_CLROC = 2, C_RDBT = 4, C_QSUM = 8 };
enum { R_BTRD = 0x42, R_QSH = 0x43, R_OBIT = 0x44, R_BTWR = 0x5e };

/* the tap order of W groups: snake, row 1 reversed */
static const uint8_t snake[9][2] = {
	{ 0, 0 }, { 0, 1 }, { 0, 2 }, { 1, 2 }, { 1, 1 }, { 1, 0 }, { 2, 0 }, { 2, 1 }, { 2, 2 },
};

/* one W group (32 oc x 32 ic, 8-bit, msf 2) as 4 entries of 256 bytes: slice s = bits 2s..2s+1 */
static void encode_group(const int8_t w[32][32], uint8_t ent[1024])
{
	for (int s = 0; s < 4; s++)
		for (int o = 0; o < 32; o++)
			for (int j = 0; j < 8; j++) {
				unsigned b = 0;

				for (int q = 0; q < 4; q++)
					b |= (((unsigned)(w[o][4 * j + q] + 128) >> (2 * s)) & 3) << (2 * q);
				ent[s * 256 + o * 8 + j] = (uint8_t)b;
			}
}

/* stream n encoded 256-byte W entries (W write pointer already set) */
static void put_entries(const struct nna_be *be, const uint8_t *e, int n)
{
	for (int i = 0; i < n; i++, e += 256) {
		be->dwr(be->ctx, 0x00, e);
		be->dwr(be->ctx, 0x02, e + 64);
		be->dwr(be->ctx, 0x04, e + 128);
		be->dwr(be->ctx, 0x07, e + 192);
	}
}

/* the encoded W of each conv record, built on first use (weights never change); nna_conv
 * rejects a record past the end (YOLO-style graphs have 300+) */
static uint8_t *wcache[NNA_MAXREC];

static void common_regs(const struct nna_be *be, int padval, int fw, int fh, int r21)
{
	be->cmd(be->ctx, 0);				/* reset */
	be->rwr(be->ctx, R_FMT, 0x33);			/* 8-bit W, 8-bit F */
	be->rwr(be->ctx, R_MSF, 2);			/* w = v - 128 */
	be->rwr(be->ctx, R_VEC, 2);			/* 32-byte pixels */
	be->rwr(be->ctx, R_MACTL, 0x03);		/* one row of 8 pixels (venus's stem value) */
	be->rwr(be->ctx, R_PXMASK, 0xff);
	be->rwr(be->ctx, R_PAD, (uint32_t)padval);
	be->rwr(be->ctx, R_FW, (uint32_t)fw);
	be->rwr(be->ctx, R_FH, (uint32_t)fh);
	/* registers venus writes that QEMU does not interpret, with venus's values. 0x21 and 0x23
	 * only for the stem (venus: 3 and 3); other layers keep their reset values */
	if (r21) {
		be->rwr(be->ctx, 0x21, (uint32_t)r21);
		be->rwr(be->ctx, 0x23, 3);
	}
	be->rwr(be->ctx, 0x28, 2);
}

/* after the weights, as venus does: the F write program (one word: count 1, step +1 line per
 * 4-pixel slot; QEMU does this unprogrammed, silicon needs it) and NNCMD 0x60 */
static void arm(const struct nna_be *be)
{
	be->rwr(be->ctx, 0x5b, 0);
	be->rwr(be->ctx, 0x45, 2);
	be->rwr(be->ctx, 0x58, 0);
	be->rwr(be->ctx, 0x59, 0x801);
	be->rwr(be->ctx, R_WFCWN, 1);
	be->cmd(be->ctx, 0x60);
}

/* CPOC|CLROC, then 16 NNDRD = 8 pixels x 32 channels. rows2: pixel p is (p/4, p%4) of a
 * 2x4 tile, else (0, p) of an 8-pixel row */
/* the per-tile instructions: inline on the CPU's own MXU3 (register / immediate as an assembler
 * constant), else through the backend */
#ifdef ONNA_MXU3
#define RWR(be, reg, v) do { if ((be)->direct) { VWR0_SET(v); \
	__asm__ volatile(".word %0" :: "i"(NNW_W(0, reg, 0))); } else (be)->rwr((be)->ctx, reg, v); } while (0)
#define MAC(be, imm, word) do { if ((be)->direct) { VWR0_SET(word); \
	__asm__ volatile(".word %0" :: "i"(NNW_W(4, imm, 0))); } else (be)->mac((be)->ctx, imm, word); } while (0)
#define CMD(be, imm) do { if ((be)->direct) \
	__asm__ volatile(".word %0" :: "i"(NNCMD_W(imm))); else (be)->cmd((be)->ctx, imm); } while (0)
#else
#define RWR(be, reg, v) (be)->rwr((be)->ctx, reg, v)
#define MAC(be, imm, word) (be)->mac((be)->ctx, imm, word)
#define CMD(be, imm) (be)->cmd((be)->ctx, imm)
#endif

/* ONNA_PROF (timing experiments only; the output is then wrong): bit 0 skips the F writes,
 * bit 1 skips requant, bit 2 only its LUT stage (clamp, lookup, byte stores), bit 3 the MACs,
 * bit 4 the stem's strip building */
static int prof(void)
{
	static int p = -1;

	if (p < 0) {
		const char *e = getenv("ONNA_PROF");

		p = e ? atoi(e) : 0;
	}
	return p;
}

#ifdef ONNA_MXU3
/* whose byte table (a QSUM conv's folded ops) is in VPRs 20..23; forgotten at every conv start */
static const uint8_t *lut_loaded;
/* whose shared requant VPRs (5..11, 14..17) are loaded: only read_sums touches them within a conv;
 * nna_conv forgets it at every start (the CPU ops between convs use those VPRs too) */
static const struct nna_vrq *vconst;
#endif

static void read_sums(const struct nna_be *be, int32_t *S, const struct nna_rq *rq, int OH, int OW,
		      int noc, int oy, int ox0, int ob, int rows2)
{
	long ps = rq && rq->ps ? rq->ps : noc;	/* output bytes per pixel */
	/* the tile's 16 read-backs, by channel half: buf[h][p] = pixel p, channels 32ob + 16h .. +15 */
	int32_t buf[2][8][16] __attribute__((aligned(64)));
	int32_t tq[8][16] __attribute__((aligned(64)));

	if (rq && rq->bt) {		/* QSUM: the NNA requantizes; 256 output bytes, pixel-major */
		uint8_t v[4][64] __attribute__((aligned(64)));
		int n = noc - 32 * ob < 32 ? noc - 32 * ob : 32;

		RWR(be, R_BTRD, (uint32_t)(4 * ob));	/* this block's BT rows */
		CMD(be, 0x80 | C_RDBT);			/* 0x84, as venus */
		CMD(be, 0x80 | C_CPOC | C_CLROC | C_QSUM);	/* 0x8b, as venus */
#ifdef ONNA_MXU3
		if (be->direct) {	/* read-backs into VPR 1, through the table (VPRs 20..23) */
			int post = rq->post && !(prof() & 2);

			if (post && lut_loaded != rq->post) {
				VLOAD(20, rq->post);
				VLOAD(21, rq->post + 64);
				VLOAD(22, rq->post + 128);
				VLOAD(23, rq->post + 192);
				lut_loaded = rq->post;
			}
			/* 32-byte aligned output pixels with a whole 32-byte slot each (n == 32, or n == 16 in a
			 * 32-byte pixel): each half of the vector goes straight to its pixel with SAO */
			static int nosao = -1;

			if (nosao < 0)
				nosao = getenv("ONNA_NOSAO") != NULL;	/* stability tests: the memcpy path */
			int sao = !nosao && (ps & 31) == 0 && !((uintptr_t)rq->q & 31) && (n == 32 || (n == 16 && ps == 32));

			for (int i = 0; i < 4; i++) {
				__asm__ volatile(MXU3_WORD(NNR_W(3, 0x20, 1)) ::: "memory");
				if (post)
					__asm__ volatile(LUT256(3, 1, 20, 4) ::: "memory");
				if (sao) {
					uint8_t *d[2];

					for (int h = 0; h < 2; h++) {
						int p = 2 * i + h, y = oy + (rows2 ? p >> 2 : 0), ox = ox0 + (rows2 ? p & 3 : p);

						d[h] = ox < OW && y < OH ? rq->q + ((long)y * OW + ox) * ps + 32 * ob : NULL;
					}
					register uint8_t *d0 __asm__("$9") = d[0];
					register uint8_t *d1 __asm__("$10") = d[1];

					if (post) {
						if (d0)
							__asm__ volatile(MXU3_WORD(SAO_W(9, 0, 0, 3)) :: "r"(d0) : "memory");
						if (d1)
							__asm__ volatile(MXU3_WORD(SAO_W(10, 0, 1, 3)) :: "r"(d1) : "memory");
					} else {
						if (d0)
							__asm__ volatile(MXU3_WORD(SAO_W(9, 0, 0, 1)) :: "r"(d0) : "memory");
						if (d1)
							__asm__ volatile(MXU3_WORD(SAO_W(10, 0, 1, 1)) :: "r"(d1) : "memory");
					}
					continue;
				}
				if (post)
					VSTORE(3, v[i]);
				else
					VSTORE(1, v[i]);
			}
			if (sao)
				return;
			for (int p = 0; p < 8; p++) {
				int y = oy + (rows2 ? p >> 2 : 0), ox = ox0 + (rows2 ? p & 3 : p);

				if (ox < OW && y < OH)
					memcpy(rq->q + ((long)y * OW + ox) * ps + 32 * ob, &v[p >> 1][32 * (p & 1)],
					       (size_t)n);
			}
			return;
		}
#endif
		for (int i = 0; i < 4; i++)
			be->drd(be->ctx, 0x20, v[i]);
		for (int p = 0; p < 8; p++) {
			int y = oy + (rows2 ? p >> 2 : 0), ox = ox0 + (rows2 ? p & 3 : p);

			if (ox >= OW || y >= OH)
				continue;
			if (rq->post) {
				uint8_t *o = rq->q + ((long)y * OW + ox) * ps + 32 * ob;
				const uint8_t *iv = &v[p >> 1][32 * (p & 1)];

				for (int c = 0; c < n; c++)
					o[c] = rq->post[iv[c]];
			} else {
				memcpy(rq->q + ((long)y * OW + ox) * ps + 32 * ob, &v[p >> 1][32 * (p & 1)], (size_t)n);
			}
		}
		return;
	}
	CMD(be, 0x80 | C_CPOC | C_CLROC);	/* 0x83, as venus */
#ifdef ONNA_MXU3
	if (rq && rq->f && rq->vrq && be->direct) {	/* a head: read-backs straight into the dequant */
		int n0 = noc - 32 * ob < 16 ? noc - 32 * ob : 16, n1 = noc - 32 * ob - 16;
		float f0[16] __attribute__((aligned(64))), f1[16] __attribute__((aligned(64)));

		n1 = n1 < 0 ? 0 : n1 > 16 ? 16 : n1;
		nna_vrq_load_bfl(&rq->vrq[ob]);
		for (int p = 0; p < 8; p++) {
			int y = oy + (rows2 ? p >> 2 : 0), ox = ox0 + (rows2 ? p & 3 : p);
			float *o;

			if (ox >= OW || y >= OH) {
				nna_vrq_drd_px(p, NULL);
				continue;
			}
			nna_vhead_drd_px(p, f0, f1);
			o = rq->f + ((long)y * OW + ox) * noc + 32 * ob;
			memcpy(o, f0, 4 * (size_t)n0);
			memcpy(o + 16, f1, 4 * (size_t)n1);
		}
		return;
	}
	if (rq && rq->vrq && be->direct && !(prof() & 2)) {	/* read-backs straight into the requant */
		int n = noc - 32 * ob < 32 ? noc - 32 * ob : 32;

		if (vconst != rq->vrq) {	/* the conv's shared VPRs, once per conv */
			nna_vrq_load_const(rq->vrq);
			vconst = rq->vrq;
		}
		nna_vrq_load_bfl(&rq->vrq[ob]);
		if (n == 16) {		/* two pixels per call; in a pair, p + 1 is the next x */
			for (int p = 0; p < 8; p += 2) {
				int y = oy + (rows2 ? p >> 2 : 0), ox = ox0 + (rows2 ? p & 3 : p);
				uint8_t *o, tmp[64] __attribute__((aligned(64)));

				if (ox >= OW || y >= OH) {
					nna_vrq_drd_px(p, NULL);
					nna_vrq_drd_px(p + 1, NULL);
					continue;
				}
				o = rq->q + ((long)y * OW + ox) * ps + 32 * ob;
				if (ps == 32) {		/* [p | 0 | p+1 | 0] (pack1 put p+1 at byte 32) */
					nna_vrq_drd_pair16(p, o, ox + 1 < OW ? 64 : 32);
				} else if (ox + 1 < OW && !((uintptr_t)o & 31)) {
					nna_vrq_drd_pair16(p, o, 32);
				} else {
					nna_vrq_drd_pair16(p, tmp, 32);
					memcpy(o, tmp, ox + 1 < OW ? 32 : 16);
				}
			}
			return;
		}
		for (int p = 0; p < 8; p++) {
			int y = oy + (rows2 ? p >> 2 : 0), ox = ox0 + (rows2 ? p & 3 : p);
			uint8_t *o, tmp[32] __attribute__((aligned(32)));

			if (ox >= OW || y >= OH) {
				nna_vrq_drd_px(p, NULL);
				continue;
			}
			o = rq->q + ((long)y * OW + ox) * ps + 32 * ob;
			if (n == 32 && !((uintptr_t)o & 31)) {
				nna_vrq_drd_px(p, o);
			} else {
				nna_vrq_drd_px(p, tmp);
				memcpy(o, tmp, (size_t)n);
			}
		}
		return;
	}
#endif
	if (rq) {
		for (int k = 0; k < 16; k++)
			be->drd(be->ctx, (unsigned)k, (uint8_t *)buf[k % 2][k / 2]);
		if (prof() & 2)
			return;
		if (rq->f) {			/* a head */
			for (int h = 0; h < 2; h++) {
				int oc0 = 32 * ob + 16 * h, n = noc - oc0 < 16 ? noc - oc0 : 16;

				for (int p = 0; p < 8 && n > 0; p++) {
					int y = oy + (rows2 ? p >> 2 : 0), ox = ox0 + (rows2 ? p & 3 : p);
					float *o;

					if (ox >= OW || y >= OH)
						continue;
					o = rq->f + ((long)y * OW + ox) * noc + oc0;
					for (int c = 0; c < n; c++)
						o[c] = (float)(buf[h][p][c] + rq->B[oc0 + c]) * rq->Fl[oc0 + c];
				}
			}
			return;
		}
#ifdef ONNA_MXU3
		if (rq->vrq) {		/* all in MXU3: 32 channels of a pixel per call */
			int n = noc - 32 * ob < 32 ? noc - 32 * ob : 32;

			nna_vrq_load(&rq->vrq[ob]);
			for (int p = 0; p < 8; p++) {
				int y = oy + (rows2 ? p >> 2 : 0), ox = ox0 + (rows2 ? p & 3 : p);
				uint8_t *o, tmp[32] __attribute__((aligned(32)));

				if (ox >= OW || y >= OH)
					continue;
				o = rq->q + ((long)y * OW + ox) * ps + 32 * ob;
				if (n == 32 && !((uintptr_t)o & 31)) {
					nna_vrq_px(buf[0][p], buf[1][p], o);
				} else {
					nna_vrq_px(buf[0][p], buf[1][p], tmp);
					memcpy(o, tmp, (size_t)n);
				}
			}
			return;
		}
#endif
		for (int h = 0; h < 2; h++) {
			int oc0 = 32 * ob + 16 * h;

			if (oc0 >= noc)
				continue;
			/* B and Fl loaded once for the 8 pixels; all t stored before any lookup */
			nna_requant_t(buf[h][0], 8, rq->B + oc0, rq->Fl + oc0, tq[0]);
			if (prof() & 4)
				continue;
			for (int p = 0; p < 8; p++) {
				int y = oy + (rows2 ? p >> 2 : 0), ox = ox0 + (rows2 ? p & 3 : p);
				uint8_t *o;

				if (ox >= OW || y >= OH)
					continue;
				o = rq->q + ((long)y * OW + ox) * ps + oc0;
				for (int c = 0; c < 16; c++) {
					int32_t tv = tq[p][c];

					tv = tv < 0 ? -1 : tv > 32767 ? 32767 : tv;
					o[c] = rq->lut[tv + 1];
				}
			}
		}
		return;
	}
	for (int k = 0; k < 16; k++) {
		int p = k / 2, y = oy + (rows2 ? p >> 2 : 0), ox = ox0 + (rows2 ? p & 3 : p);
		int oc0 = 32 * ob + 16 * (k % 2);
		long at = ((long)y * OW + ox) * noc + oc0;
		int32_t v[16] __attribute__((aligned(64)));

		if (ox >= OW || y >= OH || oc0 >= noc) {
			be->drd(be->ctx, (unsigned)k, (uint8_t *)v);	/* outside: read and drop */
			continue;
		}
		if (oc0 + 16 <= noc) {			/* 16 channels in place */
			be->drd(be->ctx, (unsigned)k, (uint8_t *)&S[at]);
		} else {
			be->drd(be->ctx, (unsigned)k, (uint8_t *)v);
			memcpy(&S[at], v, 4 * (size_t)(noc - oc0));
		}
	}
}

/* write a run of 32-byte pixels into F RAM from line `line`, 2 pixels per NNDWR */
static void put_f_at(const struct nna_be *be, const uint8_t *px, int npx, int line)
{
	RWR(be, R_FLINE, (uint32_t)line);
#ifdef ONNA_MXU3
	if (be->direct) {		/* inline: no call per 64 bytes (px 64-byte aligned) */
		for (int i = 0; i < npx; i += 2) {
			VPR0_LOAD(px + 32L * i);
			NNDWR(0x20);
		}
		return;
	}
#endif
	for (int i = 0; i < npx; i += 2)
		be->dwr(be->ctx, 0x20, px + 32L * i);
}

static void put_f(const struct nna_be *be, const uint8_t *px, int npx)
{
	put_f_at(be, px, npx, 0);
}

static int stem(const struct nna_be *be, const uint8_t *in, int H, int W, const struct nna_node *nd,
		int32_t *S, const struct nna_rq *rq, int OH, int OW)
{
	/* pixels = vertical strips of 6 RGBA rows; taps along x: two MACs of 3, the second at word 3
	 * with the W pointer advanced by the first (MAC imm bit 0). Input row y lives in slot y mod 6
	 * of every column (ic = 4 * slot + c), so each output row writes only its 2 new rows; the
	 * slots then hold ky = (slot - r) mod 6 with r = (2 oy - 2) mod 6, one of 3 weight variants */
	enum { PITCH = 32, NCOL = 20 };	/* 8 outputs at stride 2 read columns x0 .. x0 + 19 */
	static uint8_t *strip;		/* one output row's input: column x at (x + 2) * 32 */
	static int strip_w;
	int8_t w[32][32];

	common_regs(be, 0, W - 1, 0, 3);
	be->rwr(be->ctx, R_TAPS, 0x7);			/* (0,0) (0,1) (0,2) */
	be->rwr(be->ctx, R_NPLANE, 0);
	be->rwr(be->ctx, R_STRIDE, 0x11);		/* sx 2 (sy 2 as venus; one row) */
	be->rwr(be->ctx, R_PITCH, PITCH);
	be->rwr(be->ctx, R_PLANE, PITCH);
	if (!wcache[nd->rec]) {		/* variant v (r = 2v): groups kx 0..5 = entries 0..23 */
		if (!(wcache[nd->rec] = malloc(3 * 6 * 1024)))
			ERR("out of memory");
		for (int v = 0; v < 3; v++)
			for (int kx = 0; kx < 6; kx++) {
				memset(w, 0, sizeof(w));
				for (int o = 0; o < nd->oc; o++)
					for (int sl = 0; sl < 6; sl++) {
						int ky = (sl - 2 * v + 6) % 6;

						for (int c = 0; c < 4; c++)
							w[o][4 * sl + c] = nd->wq[(((long)ky * 6 + kx) * nd->oc + o) * nd->ic + c];
					}
				encode_group(w, wcache[nd->rec] + (v * 6 + kx) * 1024);
			}
	}
	if (strip_w < W) {
		/* 2 pad columns left, the tile overhang right; bytes 24..31 of a column stay 0 */
		void *p = NULL;

		free(strip);
		strip = NULL;
		if (posix_memalign(&p, 64, (size_t)(W + 2 + 2 * 8 + NCOL) * 32))
			return -1;
		strip = p;
		strip_w = W;
	}
	memset(strip, 0, (size_t)(W + 2 + 2 * 8 + NCOL) * 32);
	for (int oy = 0; oy < OH; oy++) {
		int r = ((2 * oy - 2) % 6 + 6) % 6;

		/* this row's weight variant, then its new input rows into their slots (all 6 at first) */
		be->rwr(be->ctx, R_WWR, 0);
		put_entries(be, wcache[nd->rec] + (long)(r / 2) * 6 * 1024, 24);
		arm(be);
		for (int ky = oy ? 4 : 0; ky < 6 && !(prof() & 16); ky++) {
			int y = 2 * oy - 2 + ky;
			uint32_t *d = (uint32_t *)__builtin_assume_aligned(strip + 2 * 32, 32) + ((y % 6 + 6) % 6);
			const uint32_t *sp = (const uint32_t *)__builtin_assume_aligned(in, 4) + (long)y * W;

			if (y < 0 || y >= H) {
				for (int x = 0; x < W; x++)
					d[8 * x] = 0;
				continue;
			}
			for (int x = 0; x < W; x++)
				d[8 * x] = sp[x];
		}
		for (int ox0 = 0; ox0 < OW; ox0 += 8) {
			int x0 = 2 * ox0 - 2;

			put_f(be, strip + (long)(x0 + 2) * 32, NCOL);
			RWR(be, R_FY, 0);
			RWR(be, R_FX, (uint32_t)x0 & 0xfff);
			RWR(be, 0x31, 0);
			RWR(be, R_WRD, 0);
			MAC(be, 1, 0);				/* kx 0..2, W groups 0..2 */
			RWR(be, R_FX, (uint32_t)(x0 + 3) & 0xfff);
			MAC(be, 1, 3);				/* kx 3..5, W groups 3..5 */
			read_sums(be, S, rq, OH, OW, nd->oc, oy, ox0, 0, 0);
		}
	}
	return 0;
}

/* write n 32-byte pixels (n even, p 32-byte aligned) to F from the current line on */
static void put_px(const struct nna_be *be, const uint8_t *p, int n)
{
#ifdef ONNA_MXU3
	if (be->direct) {
		for (int i = 0; i < n; i += 2, p += 64) {
			VPR0_LOAD(p);
			NNDWR(0x20);
		}
		return;
	}
#endif
	for (int i = 0; i < n; i += 2, p += 64)
		be->dwr(be->ctx, 0x20, p);
}

/* the same 2 pixels (64 bytes) n / 2 times into F */
static void put_same(const struct nna_be *be, const uint8_t *p2, int n)
{
#ifdef ONNA_MXU3
	if (be->direct) {
		VPR0_LOAD(p2);
		for (int i = 0; i < n; i += 2)
			NNDWR(0x20);
		return;
	}
#endif
	for (int i = 0; i < n; i += 2)
		be->dwr(be->ctx, 0x20, p2);
}

int nna_conv(const struct nna_be *be, const uint8_t *in, int H, int W, int C, int ps,
	     const struct nna_node *nd, int32_t *S, const struct nna_rq *rq, int *oh, int *ow)
{
	int st = nd->stride, pd = nd->pad, kh = nd->kh, kw = nd->kw;
	int OH = (H + 2 * pd - kh) / st + 1, OW = (W + 2 * pd - kw) / st + 1;
	int P = (C + 31) / 32, ntaps = kh * kw, pitch, plsz;
	unsigned taps = 0;
	uint8_t pair[64] __attribute__((aligned(64)));
	int8_t w[32][32];
	int order[9], nt = 0;

	*oh = OH, *ow = OW;
#ifdef ONNA_MXU3
	vconst = NULL;
	lut_loaded = NULL;
#endif
	if (ps <= 0)
		ps = C;
	if (nd->rec < 0 || nd->rec >= NNA_MAXREC)
		ERR("conv record %d (the W cache holds %d)", nd->rec, NNA_MAXREC);
	if (!wcache[nd->rec] && !nd->wq)	/* a caller may drop wq once the W is cached */
		ERR("conv record %d: no weights", nd->rec);
	if (nd->stem)
		return stem(be, in, H, W, nd, S, rq, OH, OW);
	if (kh > 3 || kw > 3 || P > 16)
		ERR("conv %dx%d with %d planes does not fit", kh, kw, P);
	/* F window: TR x TC outputs (2x4 MAC tiles) written once, the MACs placed in it by their
	 * word; chosen for the fewest F writes over the layer within F RAM (pitch <= 60, a plane
	 * <= 508 pixels, all planes <= 1024). TR = 2, TC = 4 is one tile per window. */
	int TR = 2, TC = 4, rows = st + kh;
	long best = -1;

	pitch = ((3 * st + kw) + 3) & ~3;
	for (int tr = 2; tr <= 32; tr += 2)
		for (int tc = 4; tc <= 64; tc += 4) {
			int r = (tr - 1) * st + kh, pt = (((tc - 1) * st + kw) + 3) & ~3;
			long cost;

			if (pt > 60 || r * pt > 508 || P * r * pt > 1024 || (tr > 2 && tr - 2 >= OH) ||
			    (tc > 4 && tc - 4 >= OW))
				continue;
			/* F writes, plus a little per window for the bookkeeping */
			cost = (long)((OH + tr - 1) / tr) * ((OW + tc - 1) / tc) * (P * r * pt / 2 + 8);
			if (best < 0 || cost < best)
				best = cost, TR = tr, TC = tc, rows = r, pitch = pt;
		}
	if (getenv("ONNA_TILE1"))			/* the old one tile per window, for comparison */
		TR = 2, TC = 4, rows = st + kh, pitch = ((3 * st + kw) + 3) & ~3;
	plsz = rows * pitch;			/* data pixels per plane = 0x37 (lines per plane) */
	uint8_t padpair[64] __attribute__((aligned(64)));

	memset(padpair, nd->padval, sizeof(padpair));

	if (pitch > 60 || plsz > 508 || P * plsz > 1024 || (nd->dw ? 1 : P) * ntaps * 4 > 288)
		ERR("tile does not fit (pitch %d, plane %d, planes %d)", pitch, plsz, P);
	for (int g = 0; g < 9; g++)
		if (snake[g][0] < kh && snake[g][1] < kw) {
			taps |= 1u << (snake[g][0] * 3 + snake[g][1]);
			order[nt++] = g;
		}
	common_regs(be, nd->padval, W - 1, H - 1, 0);
	be->rwr(be->ctx, R_MACTL, 0x12);		/* 2 rows x 4 pixels */
	be->rwr(be->ctx, R_TAPS, taps);
	be->rwr(be->ctx, R_NPLANE, (uint32_t)(nd->dw ? 0 : P - 1));	/* depthwise: block b MACs plane b */
	be->rwr(be->ctx, R_STRIDE, st == 2 ? 0x11 : 0);
	be->rwr(be->ctx, R_PITCH, (uint32_t)pitch);
	be->rwr(be->ctx, R_PLANE, (uint32_t)plsz);
	int nob = (nd->oc + 31) / 32, gper = nd->dw ? nt : P * nt;	/* groups per 32-oc block */

	if (nd->dw && nob != P)
		ERR("depthwise conv with %d channels in, %d out", C, nd->oc);
	if (rq && rq->bt) {
		if (nob > 64)
			ERR("%d BT blocks", nob);
		be->rwr(be->ctx, R_QSH, 0);		/* venus's values: shift 0, 8-bit outputs */
		be->rwr(be->ctx, R_OBIT, 3);
	}

	if (!wcache[nd->rec]) {
		uint8_t *e = wcache[nd->rec] = malloc((size_t)nob * gper * 1024);

		if (!e)
			ERR("out of memory");

		for (int ob = 0; ob < nob; ob++)
			for (int k = 0; k < P; k++)
				for (int i = 0; i < nt; i++, e += 1024) {
					int ky = snake[order[i]][0], kx = snake[order[i]][1];

					memset(w, 0, sizeof(w));
					if (nd->dw) {		/* one plane: the block's own, a diagonal */
						if (k)
							break;
						for (int o = 0; o < 32 && 32 * ob + o < nd->oc; o++)
							w[o][o] = nd->wq[((long)ky * kw + kx) * nd->oc + 32 * ob + o];
					} else {
						for (int o = 0; o < 32 && 32 * ob + o < nd->oc; o++)
							for (int c = 0; c < 32 && 32 * k + c < C; c++)
								w[o][c] = nd->wq[(((long)ky * kw + kx) * nd->oc + 32 * ob + o) * nd->ic + 32 * k + c];
					}
					encode_group(w, e);
				}
	}
	/* as many 32-oc blocks' weights as W RAM holds stay loaded together; each tile's F is written
	 * once and every block in the chunk MACs over it. W RAM on a T41NQ holds 288 entries (72 KB):
	 * venus never loads more at once (its 1152-entry layer goes in 4 loads) and a 512-entry load
	 * reads wrong weights on silicon (QEMU's model takes 1024) */
	enum { W_ENTRIES = 288 };
	int per = W_ENTRIES / (gper * 4);

	if (per < 1)
		ERR("one 32-oc block needs %d W entries", gper * 4);
	for (int ob0 = 0; ob0 < nob; ob0 += per) {
		int nb = nob - ob0 < per ? nob - ob0 : per;

		be->rwr(be->ctx, R_WWR, 0);
		put_entries(be, wcache[nd->rec] + (long)ob0 * gper * 1024, nb * gper * 4);
		arm(be);
		if (rq && rq->bt) {	/* as venus: after the W and NNCMD 0x60, from BT row 0 (0x5e) */
			be->rwr(be->ctx, R_BTWR, 0);
			for (int ob = 0; ob < nob; ob++)
				for (int i = 0; i < 4; i++)
					be->dwr(be->ctx, 0x40, (const uint8_t *)(rq->bt + 64 * ob) + 64 * i);
		}
		for (int wy = 0; wy < OH; wy += TR)
			for (int wx = 0; wx < OW; wx += TC) {
				int y0 = wy * st - pd, x0 = wx * st - pd;

				/* F, plane by plane from line k*plsz, 2 pixels per NNDWR: pairs inside the
				 * input straight from the tensor, pairs at a border through `pair`, rows
				 * outside the input as one padding vector */
				for (int k = 0; k < P && !(prof() & 1); k++) {
					int n = C - 32 * k < 32 ? C - 32 * k : 32;

					RWR(be, R_FLINE, (uint32_t)(k * plsz));
					for (int r = 0; r < rows; r++) {
						int y = y0 + r;
						const uint8_t *row = &in[((long)y * W + x0) * ps + 32 * k];

						if (y < 0 || y >= H) {
							put_same(be, padpair, pitch);
							continue;
						}
						for (int cx = 0; cx < pitch; cx += 2, row += 2L * ps) {
							int x = x0 + cx;

							if (x >= 0 && x + 1 < W) {
#ifdef ONNA_MXU3
								if (be->direct && !(ps & 31)) {
									VPR0_DWR2(row, row + ps);
									continue;
								}
#endif
								memcpy(pair, row, (size_t)n);
								memcpy(pair + 32, row + ps, (size_t)n);
							} else {
								for (int h = 0; h < 2; h++) {
									uint8_t *d = pair + 32 * h;

									if (x + h < 0 || x + h >= W)
										memset(d, nd->padval, 32);
									else
										memcpy(d, row + h * ps, (size_t)n);
								}
							}
							if (n < 32) {
								memset(pair + n, 0, (size_t)(32 - n));
								memset(pair + 32 + n, 0, (size_t)(32 - n));
							}
							put_px(be, pair, 2);
						}
					}
				}
				/* the window's 2x4 tiles; each MAC's word places it in F */
				for (int ty = 0; ty < TR && wy + ty < OH; ty += 2)
					for (int tx = 0; tx < TC && wx + tx < OW; tx += 4) {
						int oy = wy + ty, ox0 = wx + tx;
						uint32_t word = (uint32_t)(ty * st * pitch + tx * st);

						/* the W read pointer: 0 once, then each MAC (immediate bit 0)
						 * steps it past the weights it used onto the next block's, as
						 * venus does. Silicon does not take larger 0x30 values (64 and
						 * up read the wrong weights; venus only ever writes 0) */
						RWR(be, 0x31, 0);
						RWR(be, R_WRD, 0);
						for (int b = 0; b < nb; b++) {
							RWR(be, R_FY, (uint32_t)(oy * st - pd) & 0xfff);
							RWR(be, R_FX, (uint32_t)(ox0 * st - pd) & 0xfff);
							if (!(prof() & 8))
								MAC(be, 1, nd->dw ? word + 4u * (uint32_t)(plsz * (ob0 + b)) : word);
							read_sums(be, S, rq, OH, OW, nd->oc, oy, ox0, ob0 + b, 1);
						}
					}
			}
	}
	return 0;
}
