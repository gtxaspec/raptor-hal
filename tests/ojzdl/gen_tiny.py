#!/usr/bin/env python3
# Copyright (C) 2026 Thingino Project
# SPDX-License-Identifier: MIT
"""Write tiny.bin and tiny.rgb, the ojzdl host test's model and input.

tiny.bin is a JZDL version-4 model (the layout thingino-ai's own models use) with seeded random
weights: every layer type of those models (input, first conv, depthwise, pointwise, split,
eltwise, concat, upsample, two heads, output pack) at 16-32 channels on a 32x48 input. It
contains no trained or third-party weights.

tiny.ref.f32 (the expected output) is NOT made here: it is libjzdl's output for this model and
input (jzdlrun under QEMU, thingino-ai docs/jzdl-format.md "Offline runtime"), so the test
checks ojzdl against the vendor runtime, not against itself.

  python3 gen_tiny.py OUTDIR
"""
import os, random, struct, sys

H, W = 32, 48
rng = random.Random(20261128)


def i8s(n, lo=-7, hi=7):
    return bytes((rng.randint(lo, hi) & 0xff) for _ in range(n))


def i16s(vals):
    return struct.pack('<%dh' % len(vals), *vals)


def i32s(vals):
    return struct.pack('<%di' % len(vals), *vals)


def req16(c, sh_lo=4, sh_hi=10):
    """int16 bias, mult (Q15), shift, unused: the 53/73 requant blobs"""
    return [i16s([rng.randint(-200, 200) for _ in range(c)]),
            i16s([rng.randint(8000, 30000) for _ in range(c)]),
            i16s([rng.randint(sh_lo, sh_hi) for _ in range(c)]),
            i16s([0] * c)]


layers = []   # (type, bottoms, tops, params, blobs)


def first(b, t, co):          # t46: 3x3/2, pads 0/1, 4-bit weights, 8-bit in, 5-bit out
    p = [co, 3, 3, 1, 1, 2, 2, -233, 0, 1, 0, 1, co * 27, 0, 4, 8, 5, 20]
    blobs = [i8s(co * 27), i32s([rng.randint(-3000, 3000) for _ in range(co)]),
             i32s([rng.randint(1 << 29, 1 << 30) for _ in range(co)]),
             i32s([rng.randint(6, 10) for _ in range(co)])]
    layers.append((46, [b], [t], p, blobs))


def dw(b, t, c, stride):     # t73: depthwise 3x3, stride 1 (pad 1) or 2 (pads 0/1)
    pads = [1, 1, 1, 1] if stride == 1 else [0, 1, 0, 1]
    p = [c, 3, 3, 1, 1, stride, stride, 0] + pads + [c * 9, 0, 5, 5, 20]
    layers.append((73, [b], [t], p, [i8s(c * 9)] + req16(c)))


def pw(b, t, ci, co):        # t53: 1x1
    p = [co, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, ci * co, 0, 5, 5, 20]
    layers.append((53, [b], [t], p, [i8s(ci * co)] + req16(co)))


def head(b, t, ci, co):      # t51: 1x1 to float: (acc + bias) * scale
    p = [co, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, ci * co, 5, 32, 21]
    blobs = [i8s(ci * co), i32s([rng.randint(-500, 500) for _ in range(co)]),
             struct.pack('<%df' % co, *[rng.uniform(0.001, 0.05) for _ in range(co)])]
    layers.append((51, [b], [t], p, blobs))


layers.append((16, [], [0], [3, 67, 67], []))
first(0, 1, 16)                                   # 16x24x16
dw(1, 2, 16, 1)
pw(2, 3, 16, 32)                                  # 16x24x32
layers.append((33, [3], [4, 5, 6], [20], []))     # split
pw(4, 7, 32, 32)
layers.append((78, [5, 7], [8], [5, 20], []))     # eltwise (mean)
dw(8, 9, 32, 2)                                   # 8x12x32
pw(9, 10, 32, 32)
layers.append((33, [10], [11, 12], [20], []))
layers.append((69, [11], [13], [20, 5, 2, 2, 1], []))   # nearest x2 -> 16x24x32
layers.append((71, [13, 6], [14], [20], []))      # concat -> 16x24x64
head(14, 15, 64, 27)
head(12, 16, 32, 27)
layers.append((76, [15, 16], [17], [], []))       # output pack
NBLOBS = 18

words = [0, 0, NBLOBS - 1, H, W, 3, len(layers) | (4 << 24), NBLOBS]
for t, b, tp, p, _ in layers:
    words += [t, len(b), len(tp)] + b + tp + p
words[0] = 4 * len(words) - 24
data = struct.pack('<%dI' % len(words), *[x & 0xffffffff for x in words])
for *_, blobs in layers:
    data += b''.join(blobs)

out = sys.argv[1] if len(sys.argv) > 1 else '.'
with open(os.path.join(out, 'tiny.bin'), 'wb') as f:
    f.write(data)
with open(os.path.join(out, 'tiny.rgb'), 'wb') as f:
    f.write(bytes(rng.randint(0, 255) for _ in range(H * W * 3)))
print('tiny.bin %d bytes, %d layers; tiny.rgb %dx%dx3' % (len(data), len(layers), W, H))
