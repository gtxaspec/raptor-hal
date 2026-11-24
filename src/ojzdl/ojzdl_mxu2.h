/* ojzdl MXU2 kernels: internal interface between ojzdl.c and ojzdl_mxu2.c.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: MIT
 */
#ifndef OJZDL_MXU2_KERNELS_H
#define OJZDL_MXU2_KERNELS_H

#include <stddef.h>
#include <stdint.h>

#if defined(__mips__) && defined(OJZDL_MXU2)
#define OJZDL_HAVE_MXU2 1
#else
#define OJZDL_HAVE_MXU2 0
#endif

#if OJZDL_HAVE_MXU2
/* 1x1 conv on whole blocks of 4 pixels; returns the pixels done (npx & ~3).
 * Needs ci % 16 == 0, co % 16 == 0, every shift in 0..15. Weights: the
 * MXU2-blocked blob as stored in the model; bias/mult/shift: raw int16 blobs. */
size_t ojzdl_pw_mxu2(const int8_t *x, size_t npx, int ci, const int8_t *wp, int co,
                     const uint8_t *bias, const uint8_t *mult, const uint8_t *shift,
                     int bits, int8_t *dst);

/* Depthwise 3x3, stride 1 or 2 (both axes), on a padded input (rows of wp x c
 * bytes, the pads already in place). Needs c % 16 == 0, shifts 0..15, pad-slot
 * weight 0. wt: v5 block layout. */
void ojzdl_dw_mxu2(const int8_t *xp, int wp, int ho, int wo, int c, int stride, const int8_t *wt,
                   const uint8_t *bias, const uint8_t *mult, const uint8_t *shift,
                   int bits, int8_t *dst);

/* First conv (3x3, cin 3), one output row, whole blocks of 4 pixels; returns
 * pixels done. row0: padded image at the row's top-left tap; rs: padded row
 * stride; pstride: bytes between output pixels (sw * cin); qtab: per output
 * group 12 x int32[4] = bias, mult, shift for outputs {0,2,4,6}, {1,3,5,7},
 * {8,..,14}, {9,..,15}; dst: the output row. Needs slot-9 weights 0,
 * |acc| <= 32767, |bias| < 2^31 - 2^15 and shifts 0..31 (checked by caller). */
size_t ojzdl_first_mxu2(const int8_t *row0, size_t rs, int wo, int pstride,
                        const int8_t *wt, int per_group, int co, const int32_t *qtab,
                        int bits, int8_t *dst);

/* 2x2/2 max-pool of input rows a, b (w x c) into one output row; c % 16 == 0. */
void ojzdl_pool2_mxu2(const int8_t *a, const int8_t *b, int w, int c, int8_t *dst);

/* Head dot products, exact int32: acc[px][o] = sum x[px][c] * w[o][c].
 * Needs ci % 16 == 0 and every pair sum within int16 (checked by caller). */
void ojzdl_head_mxu2(const int8_t *x, size_t npx, int ci, const int8_t *w, int co, int32_t *acc);

/* dst = rgb - 128 for the first n & ~15 bytes; returns bytes done. */
size_t ojzdl_input_mxu2(const uint8_t *rgb, size_t n, int8_t *dst);

/* resize vertical pass, 16 bytes at a time; returns bytes done (n & ~15) */
size_t ojzdl_vresize_mxu2(const int16_t *r0, const int16_t *r1, int b0, int b1, uint8_t *dst, size_t n);
#endif

#endif
