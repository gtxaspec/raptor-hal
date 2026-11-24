/* MXU2 (Ingenic XBurst2 128-bit SIMD, T31) instruction encodings as .word
 * strings for inline asm. gcc/binutils have no MXU2 support.
 *
 * Encodings: QEMU target/mips/tcg/mxu2.decode (gtxaspec fork 6802b7bc46),
 * which cites "XBurst ISA MXU2 Programming Manual rev 2.16". Semantics we rely
 * on (same source, mxu2_helper.c / mxu_helper.c), checked by mxu2test.c on
 * QEMU and on the v3:
 *   DOTPS.h / DADDS.h  d[i] (=|+=) s[2i]*t[2i] + s[2i+1]*t[2i+1], s,t int8, d uint16 (wraps)
 *   ADD.h / SUB.h      d = s +/- t, wrapping
 *   MULQ.h             d = sat16((s*t) >> 15)                     (floor)
 *   SRAR.h             d = s >> n + bit (n-1) of s, n = t & 15     (round half up)
 *   DOTPS.w            d[i] = s.h[2i]*t.h[2i] + s.h[2i+1]*t.h[2i+1] (int32)
 *   MULQ.w             d = sat32((s*t) >> 31)                     (floor)
 *   SRAR.w             as SRAR.h with n = t & 31
 *   ORV                d = s | t
 *   XORIB imm          d.b = s.b ^ imm
 *   SRAI n / SRARI n   arithmetic shift right by n / rounding (as SRAR)
 *   MAXS/MINS          signed max/min
 *   SATS imm           clamp to imm+1 signed bits
 *   SHUFV              cat[2i] = t[i], cat[2i+1] = s[i]; d[i] = r[i] & 0x80 ? 0 : cat[r[i] & 31]
 *   REPI.h imm         d[*] = s.h[imm]
 *   LI imm             d[*] = imm (sign-extended 15-bit)
 *   LU1Q / SU1Q        128-bit load/store, any alignment, byte offset s10
 *
 * Vector registers are named by number 0..31. GPR operands must be fixed
 * registers: bind the C variable with `register T *p __asm__("$8")` and pass
 * it to the asm statement as an "r" input so gcc keeps it there.
 */
#ifndef OJZDL_MXU2_H
#define OJZDL_MXU2_H

#define MX_S_(x) #x
#define MX_S(x) MX_S_(x)

/* data formats */
#define DF_B 0
#define DF_H 1
#define DF_W 2
#define DF_D 3

/* COP2 3R: 010010 | grp:5 | vt | vs | vd | minor:4 | df:2 */
#define MX_3R(grp, minor, vd, vs, vt, df) \
	".word (0x12<<26)|(" MX_S(grp) "<<21)|((" MX_S(vt) ")<<16)|((" MX_S(vs) ")<<11)|((" MX_S(vd) ")<<6)|(" MX_S(minor) "<<2)|(" MX_S(df) ")\n\t"

/* 3RINT-0 (grp 0x10) */
#define MAXS(vd, vs, vt, df)  MX_3R(0x10, 0x2, vd, vs, vt, df)
#define MINS(vd, vs, vt, df)  MX_3R(0x10, 0x3, vd, vs, vt, df)
#define SRAR(vd, vs, vt, df)  MX_3R(0x10, 0x8, vd, vs, vt, df)
/* 3RINT-1 (grp 0x11) */
#define ADD(vd, vs, vt, df)   MX_3R(0x11, 0x8, vd, vs, vt, df)
#define SUB(vd, vs, vt, df)   MX_3R(0x11, 0xb, vd, vs, vt, df)
/* 3RINT-2 (grp 0x12) */
#define DOTPS(vd, vs, vt, df) MX_3R(0x12, 0x8, vd, vs, vt, df)
#define DADDS(vd, vs, vt, df) MX_3R(0x12, 0x9, vd, vs, vt, df)

/* 3RFP Q-format (grp 0x18): minor5 = 10100 (MULQ), low bit = 0 for .h, 1 for .w */
#define MULQ_H(vd, vs, vt) \
	".word (0x12<<26)|(0x18<<21)|((" MX_S(vt) ")<<16)|((" MX_S(vs) ")<<11)|((" MX_S(vd) ")<<6)|(0x14<<1)|0\n\t"
#define MULQ_W(vd, vs, vt) \
	".word (0x12<<26)|(0x18<<21)|((" MX_S(vt) ")<<16)|((" MX_S(vs) ")<<11)|((" MX_S(vd) ")<<6)|(0x14<<1)|1\n\t"

/* 3RVEC: 010010 10110 vt vs vd 111010 */
#define ORV(vd, vs, vt) \
	".word (0x12<<26)|(0x16<<21)|((" MX_S(vt) ")<<16)|((" MX_S(vs) ")<<11)|((" MX_S(vd) ")<<6)|0x3a\n\t"

/* SPECIAL2 forms: 011100 ... */
/* SHUFV: 011100 | vr | vt | vs | vd | 011000 */
#define SHUFV(vd, vs, vt, vr) \
	".word (0x1c<<26)|((" MX_S(vr) ")<<21)|((" MX_S(vt) ")<<16)|((" MX_S(vs) ")<<11)|((" MX_S(vd) ")<<6)|0x18\n\t"
/* LU1Q / SU1Q: 011100 | base | offset:s10 | vd | 010100 / 011100 */
#define LU1Q(vd, off, base) \
	".word (0x1c<<26)|((" MX_S(base) ")<<21)|(((" MX_S(off) ")&0x3ff)<<11)|((" MX_S(vd) ")<<6)|0x14\n\t"
#define SU1Q(vd, off, base) \
	".word (0x1c<<26)|((" MX_S(base) ")<<21)|(((" MX_S(off) ")&0x3ff)<<11)|((" MX_S(vd) ")<<6)|0x1c\n\t"
/* LU1QX: 011100 | base | index | 00000 | vd | 000111 ; address = base + index */
#define LU1QX(vd, base, index) \
	".word (0x1c<<26)|((" MX_S(base) ")<<21)|((" MX_S(index) ")<<16)|((" MX_S(vd) ")<<6)|0x07\n\t"
/* REPI: 011100 | df:2 | imm:8 | vs | vd | 110101 */
#define REPI(vd, vs, imm, df) \
	".word (0x1c<<26)|((" MX_S(df) ")<<24)|((" MX_S(imm) ")<<16)|((" MX_S(vs) ")<<11)|((" MX_S(vd) ")<<6)|0x35\n\t"
/* SATS: 011100 | df:2 | 00 | imm:6 | vs | vd | 111000 ; clamps to imm+1 bits */
#define SATS(vd, vs, bits, df) \
	".word (0x1c<<26)|((" MX_S(df) ")<<24)|(((" MX_S(bits) ")-1)<<16)|((" MX_S(vs) ")<<11)|((" MX_S(vd) ")<<6)|0x38\n\t"
/* XORIB: 011100 | 11 | imm:8 | vs | vd | 110000 ; d.b = s.b ^ imm */
#define XORIB(vd, vs, imm) \
	".word (0x1c<<26)|(0x3<<24)|((" MX_S(imm) ")<<16)|((" MX_S(vs) ")<<11)|((" MX_S(vd) ")<<6)|0x30\n\t"
/* SRAI / SRARI: 011100 | df:2 | 00 / 01 | imm:6 | vs | vd | 111001 */
#define SRAI(vd, vs, n, df) \
	".word (0x1c<<26)|((" MX_S(df) ")<<24)|((" MX_S(n) ")<<16)|((" MX_S(vs) ")<<11)|((" MX_S(vd) ")<<6)|0x39\n\t"
#define SRARI(vd, vs, n, df) \
	".word (0x1c<<26)|((" MX_S(df) ")<<24)|(1<<22)|((" MX_S(n) ")<<16)|((" MX_S(vs) ")<<11)|((" MX_S(vd) ")<<6)|0x39\n\t"
/* LI: 011100 | imm:s15 | vd | 0011 | df */
#define LI(vd, imm, df) \
	".word (0x1c<<26)|(((" MX_S(imm) ")&0x7fff)<<11)|((" MX_S(vd) ")<<6)|(0x3<<2)|(" MX_S(df) ")\n\t"

/* GPR numbers used as fixed base registers by the kernels ($t0..$t7) */
#define GP0 8
#define GP1 9
#define GP2 10
#define GP3 11
#define GP4 12
#define GP5 13
#define GP6 14
#define GP7 15

#endif
