/*
 * onnaq as a library: one QSUM model (venus .bin + onnaq text graph) per process, run on the T41
 * NNA or on the CPU. Build onnaq.c with -DONNAQ_LIB (no main). Not thread safe: one caller.
 *
 * On the NNA every onnaq_run must come from a thread pinned to CPU0 (the NN instructions are
 * reserved instructions on CPU1). onnaq_open(..., 1) also pins the ISP firmware thread to CPU0,
 * which avoids the T41 dual-core deadlock (docs/nna-runtime.md, "Stability").
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: MIT
 */
#ifndef ONNAQ_H
#define ONNAQ_H
#include <stdint.h>

/* load the model; nna: 1 = the real NNA (T41 build), 0 = CPU. 0 on success, else -1 and
 * onnaq_error() says why */
int onnaq_open(const char *bin_path, const char *graph_path, int nna);
const char *onnaq_error(void);
/* network input size (RGB bytes, H x W x 3; a 4-channel model such as yolov5n too: onnaq_run adds
 * alpha 0) */
void onnaq_input_size(int *w, int *h);
/* one inference on H x W x 3 RGB bytes; 0 on success */
int onnaq_run(const uint8_t *rgb);
/* outputs, in the graph's output order: NHWC floats (valid until the next onnaq_run) */
int onnaq_num_heads(void);
const float *onnaq_head(int k, int *h, int *w, int *c);
/* the graph's anchors for a head of this stride (3 pairs w, h in input pixels), or YOLOv5's
 * defaults for strides 8/16/32 when the graph names none; 0 if found */
int onnaq_anchors(int stride, float wh[6]);
void onnaq_close(void);

#endif
