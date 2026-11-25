#!/usr/bin/env python3
"""
yolo_ref.py -- reference YOLO decode for tests/test_yolo.c

Builds small YOLOv5-style head tensors (HWC, 3 anchors x (5 + 3 classes),
values before the sigmoid), decodes them with the YOLOv5 formulas written
out here independently of src/hal_ivs_yolo.c, and writes the slots that differ
from the background and the expected boxes as tests/yolo_vectors.h:

    python3 tests/yolo_ref.py > tests/yolo_vectors.h

Decode: score = sigmoid(obj) * sigmoid(best class); kept when score >= the
class's threshold (else conf); xy = (2 sigmoid(t) - 0.5 + cell) * stride;
wh = (2 sigmoid(t))^2 * anchor. NMS: by score, greedy, per class, a box
dropped when its IoU with a kept box of its class is above nms. To the
frame: (model - off) * scale, truncated, clamped to the frame.
"""

import math

NC = 3  # classes
NA = 3  # anchors per head
PER = 5 + NC
C = NA * PER
BG = -9.0  # background logit: sigmoid ~1.2e-4

STOCK = {
    8: [10, 13, 16, 30, 33, 23],
    16: [30, 61, 62, 45, 59, 119],
    32: [116, 90, 156, 198, 373, 326],
}


def sig(v):
    return 1.0 / (1.0 + math.exp(-v))


def logit(p):
    return math.log(p / (1.0 - p))


def new_head(gh, gw):
    return [[[BG] * C for _ in range(gw)] for _ in range(gh)]


def put(h, gy, gx, a, txy, twh, obj, cls, score):
    """one anchor slot: obj = sigmoid(objectness), class cls at score/obj"""
    cl = [BG] * NC
    cl[cls] = logit(score / obj)
    h[gy][gx][a * PER:(a + 1) * PER] = [txy[0], txy[1], twh[0], twh[1], logit(obj)] + cl


def decode(heads, anchors, conf, cls_thr):
    """heads: [(stride, head)] in output order; anchors: {stride: 6 floats}"""
    out = []
    for stride, h in heads:
        anc = anchors[stride]
        for gy, row in enumerate(h):
            for gx, cell in enumerate(row):
                for a in range(NA):
                    p = cell[a * PER:(a + 1) * PER]
                    best = max(range(NC), key=lambda c: (p[5 + c], -c))
                    score = sig(p[4]) * sig(p[5 + best])
                    t = cls_thr[best] if cls_thr[best] > 0 else conf
                    if score < t:
                        continue
                    cx = (2 * sig(p[0]) - 0.5 + gx) * stride
                    cy = (2 * sig(p[1]) - 0.5 + gy) * stride
                    w = (2 * sig(p[2])) ** 2 * anc[2 * a]
                    hh = (2 * sig(p[3])) ** 2 * anc[2 * a + 1]
                    out.append((cx - w / 2, cy - hh / 2, cx + w / 2, cy + hh / 2, score, best))
    return out


def iou(a, b):
    iw = min(a[2], b[2]) - max(a[0], b[0])
    ih = min(a[3], b[3]) - max(a[1], b[1])
    if iw <= 0 or ih <= 0:
        return 0.0
    inter = iw * ih
    return inter / ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter)


def nms(boxes, thr):
    kept = []
    for b in sorted(boxes, key=lambda b: -b[4]):
        if all(k[5] != b[5] or iou(k, b) <= thr for k in kept):
            kept.append(b)
    return kept


def emit(boxes, sx, sy, ox, oy, fw, fh):
    res = []
    for b in boxes[:20]:
        v = [(b[0] - ox) * sx, (b[1] - oy) * sy, (b[2] - ox) * sx, (b[3] - oy) * sy]
        for x in v:  # float32 in C: keep clear of an integer edge
            assert abs(x - round(x)) > 1e-3, x
        x0, y0, x1, y1 = (int(x) for x in v)
        res.append((max(x0, 0), max(y0, 0), min(x1, fw - 1), min(y1, fh - 1)))
    return res


def stock_with(over):
    a = dict(STOCK)
    a.update(over)
    return a


# Two heads like persondet: strides 32 then 16, input 128x64 (w x h).
TWO_W, TWO_H = 128, 64
two32 = new_head(TWO_H // 32, TWO_W // 32)
two16 = new_head(TWO_H // 16, TWO_W // 16)
put(two32, 1, 2, 0, (0.3, -0.2), (0.1, -0.3), 0.97, 0, 0.88)    # D1 kept
put(two16, 2, 5, 1, (-0.4, 0.25), (0.2, 0.15), 0.93, 2, 0.70)   # D2 kept
put(two16, 2, 6, 1, (-0.4, 0.25), (0.2, 0.15), 0.91, 2, 0.60)   # D3 same class, IoU 0.6 with D2: NMS
put(two16, 1, 5, 1, (-0.4, 0.3), (0.2, 0.15), 0.90, 1, 0.65)    # D4 other class over D2: kept
put(two32, 0, 0, 2, (0.1, 0.1), (-0.5, -0.5), 0.60, 0, 0.40)    # D5 under conf
put(two32, 0, 1, 1, (0.0, 0.0), (0.0, 0.0), 0.95, 1, 0.43)      # D6 high obj, low class: dropped
put(two32, 0, 3, 0, (0.2, -0.1), (-0.2, 0.1), 0.80, 2, 0.35)    # D7 kept only at class 2's 0.3
TWO = [(32, two32), (16, two16)]

# Three heads like YOLOv5: strides 8, 16, 32, input 96x64.
THREE_W, THREE_H = 96, 64
three8 = new_head(THREE_H // 8, THREE_W // 8)
three16 = new_head(THREE_H // 16, THREE_W // 16)
three32 = new_head(THREE_H // 32, THREE_W // 32)
put(three8, 3, 4, 0, (0.15, -0.35), (0.3, 0.05), 0.96, 1, 0.85)   # E1
put(three8, 3, 5, 0, (0.15, -0.35), (0.3, 0.05), 0.80, 1, 0.55)   # E2 beside E1, IoU 0.1: kept
put(three16, 1, 2, 2, (-0.25, 0.45), (-0.1, -0.2), 0.92, 0, 0.75)  # E3
put(three32, 0, 1, 1, (0.05, 0.2), (-0.3, 0.25), 0.88, 2, 0.62)   # E4
THREE = [(8, three8), (16, three16), (32, three32)]

PARAM32 = [20, 40, 50, 30, 80, 60]  # caller's anchors for stride 32 only

NOTHR = [0.0] * NC
CLSTHR = [0.9, 0.6, 0.3]  # class 0 above D1, class 1 under D4, class 2 under D7

CASES = {
    # name: (heads, anchors, conf, nms, class thresholds, emit args)
    "two": (TWO, STOCK, 0.5, 0.45, NOTHR, (640 / 128, 360 / 64, 0, 0, 640, 360)),
    "two_param": (TWO, stock_with({32: PARAM32}), 0.5, 0.45, NOTHR,
                  (640 / 128, 360 / 64, 0, 0, 640, 360)),
    "two_clsthr": (TWO, STOCK, 0.5, 0.45, CLSTHR, (640 / 128, 360 / 64, 0, 0, 640, 360)),
    "three": (THREE, STOCK, 0.5, 0.45, NOTHR, (640 / 96, 360 / 64, 0, 0, 640, 360)),
    # letterbox of a 1920x1080 frame into 96x64: scale 0.05, pad_y 5
    "three_lb": (THREE, STOCK, 0.5, 0.45, NOTHR, (20.0, 20.0, 0.0, 5.0, 1920, 1080)),
}


def fmt(v):
    s = "%.9g" % v
    return s + ("f" if ("." in s or "e" in s) else ".0f")


def c_slots(name, h):
    """the slots that are not background: gy, gx, anchor, then the PER values"""
    rows = []
    for gy, row in enumerate(h):
        for gx, cell in enumerate(row):
            for a in range(NA):
                p = cell[a * PER:(a + 1) * PER]
                if any(v != BG for v in p):
                    rows.append("    {%d, %d, %d, {%s}}," % (gy, gx, a, ", ".join(fmt(v) for v in p)))
    return "static const yv_slot_t %s[] = {\n%s\n};\n" % (name, "\n".join(rows))


def main():
    print("/* Generated by tests/yolo_ref.py; do not edit. */")
    print("/* clang-format off */")
    print()
    print("#define YV_NC %d" % NC)
    print("#define YV_C %d" % C)
    print("#define YV_TWO_W %d" % TWO_W)
    print("#define YV_TWO_H %d" % TWO_H)
    print("#define YV_THREE_W %d" % THREE_W)
    print("#define YV_THREE_H %d" % THREE_H)
    print()
    print("#define YV_PER %d" % PER)
    print("#define YV_BG %s" % fmt(BG))
    print()
    print("typedef struct {")
    print("    int gy, gx, a;")
    print("    float v[YV_PER]; /* x, y, w, h, objectness, classes: before the sigmoid */")
    print("} yv_slot_t;")
    print()
    for name, h in (("two32", two32), ("two16", two16), ("three8", three8),
                    ("three16", three16), ("three32", three32)):
        print(c_slots("yv_" + name, h))
    print("static const float yv_param32[6] = {%s};" % ", ".join(fmt(v) for v in PARAM32))
    print("static const float yv_clsthr[YV_NC] = {%s};" % ", ".join(fmt(v) for v in CLSTHR))
    print()
    print("typedef struct {")
    print("    float x1, y1, x2, y2, score; /* model input pixels */")
    print("    int cls;")
    print("    int fx0, fy0, fx1, fy1; /* frame pixels */")
    print("} yv_box_t;")
    for name, (heads, anchors, conf, thr, cls_thr, em) in CASES.items():
        kept = nms(decode(heads, anchors, conf, cls_thr), thr)
        fr = emit(kept, *em)
        print()
        print("#define YV_%s_N %d" % (name.upper(), len(kept)))
        print("static const yv_box_t yv_%s[] = {" % name)
        for b, f in zip(kept, fr):
            print("    {%s, %d, %d, %d, %d, %d}," % (", ".join(fmt(v) for v in b[:5]), b[5], *f))
        print("};")


if __name__ == "__main__":
    main()
