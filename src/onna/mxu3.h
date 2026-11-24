/*
 * mxu3.h: the MXU3 and NN instructions onna uses, as .word encodings (T41, XBurst2).
 * Encodings from gtxaspec/qemu target/mips/tcg/mxu3.decode (SPECIAL2 major opcode 011100):
 *   LAO   011100 base:5 off:5(x32, signed) n:2 011 vr:5 010001   load octword n of VPR vr
 *   SAO   011100 base:5 off:5(x32, signed) vr:5 n:2 011 010101   store octword n of VPR vr
 *   MFCPUW 011100 rs:5 11011 00000 vwd:5 000011                   GPR rs -> VWR word vwd
 *   MTCPUW 011100 11010 00000 vwp:5 rd:5 101110                   VWR word vwp -> GPR rd
 *   NNRWR 011100 00000 imm[9:5] vr:5 imm[4:0] 111010   engine register imm = VWR word vr
 *   NNDWR 011100 00010 imm[9:5] vr:5 imm[4:0] 111010   64 bytes of VPR vr to W/F/BT RAM
 *   NNDRD 011100 00011 imm:10 vr:5 111010              64 bytes into VPR vr
 *   NNMAC 011100 00100 imm[9:5] vr:5 imm[4:0] 111010   MAC; the pixel word is VWR word vr
 *   NNCMD 011100 00101 imm:15 111010
 * The NN instructions run on CPU0 only (the T41's NN unit is wired to core 0).
 * Operands are pinned: data pointer in $9 (t1), scalars in $8 (t0); VPR 0 for data, VWR word 0.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: MIT
 */
#ifndef MXU3_H
#define MXU3_H

#define MXU3_STR_(x) #x
#define MXU3_STR(x) MXU3_STR_(x)

#define LAO_W(base, off, n, vr)  (0x70000000 | ((base) << 21) | (((off) & 31) << 16) | ((n) << 14) | (3 << 11) | ((vr) << 6) | 0x11)
#define SAO_W(base, off, n, vr)  (0x70000000 | ((base) << 21) | (((off) & 31) << 16) | ((vr) << 11) | ((n) << 9) | (3 << 6) | 0x15)
#define MFCPUW_W(rs, vwd)        (0x70000000 | ((rs) << 21) | (0x1b << 16) | ((vwd) << 6) | 0x03)
#define MTCPUW_W(vwp, rd)        (0x70000000 | (0x1a << 21) | ((vwp) << 11) | ((rd) << 6) | 0x2e)
#define NNW_W(minor, imm, vr)    (0x70000000 | ((minor) << 21) | ((((imm) >> 5) & 31) << 16) | ((vr) << 11) | (((imm) & 31) << 6) | 0x3a)
#define NNR_W(minor, imm, vr)    (0x70000000 | ((minor) << 21) | (((imm) & 0x3ff) << 11) | ((vr) << 6) | 0x3a)
#define NNCMD_W(imm)             (0x70000000 | (5 << 21) | (((imm) & 0x7fff) << 6) | 0x3a)

#define MXU3_WORD(w) ".word " MXU3_STR(w) "\n\t"

/* load / store all 64 bytes of VPR 0 from / to p ($9) */
#define VPR0_LOAD(p) do { register const void *_p __asm__("$9") = (p); \
	__asm__ volatile(MXU3_WORD(LAO_W(9, 0, 0, 0)) MXU3_WORD(LAO_W(9, 1, 1, 0)) :: "r"(_p) : "memory"); } while (0)
#define VPR0_STORE(p) do { register void *_p __asm__("$9") = (p); \
	__asm__ volatile(MXU3_WORD(SAO_W(9, 0, 0, 0)) MXU3_WORD(SAO_W(9, 1, 1, 0)) :: "r"(_p) : "memory"); } while (0)
/* VPR 0 <- the 32 bytes at p0 (half 0) and at p1 (half 1), then NNDWR 0x20: two pixels of one
 * 32-channel plane straight from an HWC tensor (both 32-byte aligned) */
#define VPR0_DWR2(p0, p1) do { register const void *_a __asm__("$9") = (p0); \
	register const void *_b __asm__("$10") = (p1); \
	__asm__ volatile(MXU3_WORD(LAO_W(9, 0, 0, 0)) MXU3_WORD(LAO_W(10, 0, 1, 0)) \
			 MXU3_WORD(NNW_W(2, 0x20, 0)) :: "r"(_a), "r"(_b) : "memory"); } while (0)
/* VWR word 0 <- v ($8) */
#define VWR0_SET(v) do { register unsigned _v __asm__("$8") = (v); \
	__asm__ volatile(MXU3_WORD(MFCPUW_W(8, 0)) :: "r"(_v)); } while (0)
#define VWR0_GET(v) do { register unsigned _v __asm__("$8"); \
	__asm__ volatile(MXU3_WORD(MTCPUW_W(0, 8)) : "=r"(_v)); (v) = _v; } while (0)

#define NNRWR(imm)  __asm__ volatile(MXU3_WORD(NNW_W(0, imm, 0)))
#define NNDWR(imm)  __asm__ volatile(MXU3_WORD(NNW_W(2, imm, 0)) ::: "memory")
#define NNDRD(imm)  __asm__ volatile(MXU3_WORD(NNR_W(3, imm, 0)) ::: "memory")
#define NNMAC(imm)  __asm__ volatile(MXU3_WORD(NNW_W(4, imm, 0)))
#define NNCMD(imm)  __asm__ volatile(MXU3_WORD(NNCMD_W(imm)))

/* vector float32 requant helpers (COP2 major 010010; @3r: vs 20:16, vp 15:11, vd 10:6, df 1:0):
 *   ADDW  010010 10100 vs vp vd 0000 10        word add (df 2 = word)
 *   FMULW 010010 10011 vs vp vd 1000 11        float32 multiply
 *   FCVT  011100 00 sel:3 vs 00000 vd 101110   sel 2: int32 -> float32, 7: float32 -> int32
 *                                              in the current rounding mode (nearest even = rint)
 */
#define ADDW_W(vd, vs, vp)  (0x48000000 | (0x14 << 21) | ((vs) << 16) | ((vp) << 11) | ((vd) << 6) | 0x02)
#define FMULW_W(vd, vs, vp) (0x48000000 | (0x13 << 21) | ((vs) << 16) | ((vp) << 11) | ((vd) << 6) | 0x23)
#define FCVT_W(sel, vd, vs) (0x70000000 | ((sel) << 21) | ((vs) << 16) | ((vd) << 6) | 0x2e)
/* more vector ops for the SiLU requant (mxu3.decode; word elements, df = 2) */
#define MXU3_3R(minor, fn, vd, vs, vp) (0x48000000 | ((minor) << 21) | ((vs) << 16) | ((vp) << 11) | ((vd) << 6) | (fn))
#define MINSW_W(vd, vs, vp)  MXU3_3R(0x10, (0x5 << 2) | 2, vd, vs, vp)	/* signed min */
#define MAXSW_W(vd, vs, vp)  MXU3_3R(0x10, (0x7 << 2) | 2, vd, vs, vp)	/* signed max */
#define SLLIW_W(vd, vs, imm) MXU3_3R(0x15, (0x8 << 2) | 2, vd, vs, imm)
#define SRAIW_W(vd, vs, imm) MXU3_3R(0x15, (0xe << 2) | 2, vd, vs, imm)
#define ANDV_W(vd, vs, vp)   MXU3_3R(0x13, 0x02, vd, vs, vp)
/* signed words saturated to 0..255 (SATSU, tgt 2 = 8 bits, df 2 = words) */
#define SATSU8W_W(vd, vs)    (0x70000000 | (1 << 25) | (2 << 23) | (2 << 21) | ((vs) << 16) | ((vd) << 6) | 0x2f)
#define MAXUB_W(vd, vs, vp)  MXU3_3R(0x10, 0x08, vd, vs, vp)	/* unsigned byte max */
#define SUBW_W(vd, vs, vp)   MXU3_3R(0x14, (0x2 << 2) | 2, vd, vs, vp)	/* vs - vp */
#define MULW_W(vd, vs, vp)   MXU3_3R(0x13, (0x8 << 2) | 2, vd, vs, vp)	/* low 32 bits */
#define CLTSW_W(vd, vs, vp)  MXU3_3R(0x10, (0xd << 2) | 2, vd, vs, vp)	/* vs < vp: all ones */
#define CLESW_W(vd, vs, vp)  MXU3_3R(0x10, (0xf << 2) | 2, vd, vs, vp)	/* vs <= vp: all ones */
#define ORV_W(vd, vs, vp)    MXU3_3R(0x13, 0x04, vd, vs, vp)
#define FADDW_W(vd, vs, vp)  MXU3_3R(0x14, 0x03, vd, vs, vp)
/* byte i of vd = byte (vs byte i) & 63 of vp, or 0 when that selector byte has bit 7 set */
/* byte ops with an 8-bit immediate (decodetree %imm8: imm[7:5] in bits 2:0, imm[4:0] in 15:11) */
#define MXU3_IB(op, vd, vs, imm) (0x48000000 | (0x1b << 21) | ((vs) << 16) | (((imm) & 31) << 11) | ((vd) << 6) | \
				  ((op) << 3) | (((imm) >> 5) & 7))
#define ANDIB_W(vd, vs, imm) MXU3_IB(1, vd, vs, imm)
#define XORIB_W(vd, vs, imm) MXU3_IB(3, vd, vs, imm)
/* 256-entry byte table in VPRs t..t+3 (64 entries each): VPR d = table[VPR x], bytewise. Chunk k:
 * y = x ^ 64k is 0..63 only for x in chunk k; y | 2 (y & 0x40) sets bit 7 for the others, which
 * GSHUFVB turns into 0; the 4 gathers are ORed. Scratch: VPRs s, s+1, s+2 */
#define LUT256_CHUNK(d, x, t, k, s) MXU3_WORD(XORIB_W(s, x, 64 * (k))) MXU3_WORD(ANDIB_W((s) + 1, s, 0x40)) \
	MXU3_WORD(ADDW_W((s) + 1, (s) + 1, (s) + 1)) MXU3_WORD(ORV_W(s, s, (s) + 1)) \
	MXU3_WORD(GSHUFVB_W((s) + 2, s, (t) + (k))) MXU3_WORD(ORV_W(d, d, (s) + 2))
#define LUT256(d, x, t, s) MXU3_WORD(ANDIB_W((s) + 1, x, 0x40)) MXU3_WORD(ADDW_W((s) + 1, (s) + 1, (s) + 1)) \
	MXU3_WORD(ORV_W(s, x, (s) + 1)) MXU3_WORD(GSHUFVB_W(d, s, t)) \
	LUT256_CHUNK(d, x, t, 1, s) LUT256_CHUNK(d, x, t, 2, s) LUT256_CHUNK(d, x, t, 3, s)
#define GSHUFVB_W(vd, vs, vp) (0x70000000 | (0x1c << 21) | ((vs) << 16) | ((vp) << 11) | ((vd) << 6) | 0x2e)

/* load / store all 64 bytes of VPR vr from / to p ($9) */
#define VLOAD(vr, p) do { register const void *_p __asm__("$9") = (p); \
	__asm__ volatile(MXU3_WORD(LAO_W(9, 0, 0, vr)) MXU3_WORD(LAO_W(9, 1, 1, vr)) :: "r"(_p) : "memory"); } while (0)
#define VSTORE(vr, p) do { register void *_p __asm__("$9") = (p); \
	__asm__ volatile(MXU3_WORD(SAO_W(9, 0, 0, vr)) MXU3_WORD(SAO_W(9, 1, 1, vr)) :: "r"(_p) : "memory"); } while (0)

/* B and Fl of a 16-channel block into VPRs 2 and 3, then any number of
 * t[16] = rint(float32(S[16] + B) * Fl) through VPRs 1 and 4 */
#define VRQ_SETUP(B, Fl) do { VLOAD(2, B); VLOAD(3, Fl); } while (0)
#define VRQ_ONE(S, t) do { VLOAD(1, S); \
	__asm__ volatile(MXU3_WORD(ADDW_W(4, 1, 2)) MXU3_WORD(FCVT_W(2, 4, 4)) \
			 MXU3_WORD(FMULW_W(4, 4, 3)) MXU3_WORD(FCVT_W(7, 4, 4)) ::: "memory"); \
	VSTORE(4, t); } while (0)

/* t[16] = rint(float32(S[16] + B[16]) * Fl[16]) in VPRs 1..4 (VPR 0 stays the NN data register) */
#define VREQUANT16(S, B, Fl, t) do { \
	VLOAD(1, S); VLOAD(2, B); VLOAD(3, Fl); \
	__asm__ volatile(MXU3_WORD(ADDW_W(4, 1, 2)) MXU3_WORD(FCVT_W(2, 4, 4)) \
			 MXU3_WORD(FMULW_W(4, 4, 3)) MXU3_WORD(FCVT_W(7, 4, 4)) ::: "memory"); \
	VSTORE(4, t); } while (0)

#endif
