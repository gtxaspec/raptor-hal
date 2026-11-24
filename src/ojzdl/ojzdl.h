/* ojzdl: open runtime for JZDL v5 models (Ingenic libjzdl 5.4.0 format, T31).
 *
 * Plain C99, no dependencies. Output is meant to be bit-identical to libjzdl
 * and to tools/jzdl/jzdl_ref.py. Format and arithmetic: ../../jzdl-format.md.
 *
 * Usage:
 *   ojzdl_net *n = ojzdl_load_file("model.bin", &err);
 *   ojzdl_input_rgb(n, rgb);              // in_h x in_w x 3 uint8, HWC
 *   ojzdl_forward(n);
 *   const float *o = ojzdl_output(n, &count);
 *   ojzdl_free(n);
 */
#ifndef OJZDL_H
#define OJZDL_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ojzdl_net ojzdl_net;

/* Load from memory. The buffer is copied where needed; caller may free it after. */
ojzdl_net *ojzdl_load(const uint8_t *buf, size_t len, const char **err);
ojzdl_net *ojzdl_load_file(const char *path, const char **err);
void ojzdl_free(ojzdl_net *n);

/* Input shape (from the model header). */
int ojzdl_in_h(const ojzdl_net *n);
int ojzdl_in_w(const ojzdl_net *n);
int ojzdl_in_c(const ojzdl_net *n);

/* Feed one frame: uint8 HWC, in_h * in_w * in_c bytes. Does image_sub(128). */
int ojzdl_input_rgb(ojzdl_net *n, const uint8_t *rgb);

/* Run every layer. Returns 0 on success. */
int ojzdl_forward(ojzdl_net *n);

/* Packed float output of the output blob (type 76: heads + trailer). */
const float *ojzdl_output(const ojzdl_net *n, size_t *count);

/* Any blob after forward: int8 activations (dtype 20) or floats (heads).
 * Returns 0 if the blob exists; fills shape and exactly one of i8 / f32. */
int ojzdl_blob(const ojzdl_net *n, int blob, int *h, int *w, int *c, int *bits,
               const int8_t **i8, const float **f32);

/* Per-layer timing of the last forward (ns), filled when timing is enabled. */
void ojzdl_set_timing(ojzdl_net *n, int on);
/* Keep every blob valid after forward (for probing); default reuses memory
 * after a blob's last reader. Re-plans memory; returns 0 on success. */
int ojzdl_keep_blobs(ojzdl_net *n, int on);
/* Bytes of activation memory (arena) and scratch; returns the sum. */
size_t ojzdl_memory(const ojzdl_net *n, size_t *arena, size_t *scratch);
/* Use the MXU2 kernels where they apply (default on in MXU2 builds). */
int ojzdl_set_simd(ojzdl_net *n, int on);
int ojzdl_layer_count(const ojzdl_net *n);
int ojzdl_layer_info(const ojzdl_net *n, int i, int *type, uint64_t *ns);

/* Bilinear RGB resize identical to libjzdl's jzdl::resize (INTER_LINEAR) for
 * the sizes in ojzdl_resize.c; use it to make the model input from a frame. */
typedef struct ojzdl_resize ojzdl_resize;
ojzdl_resize *ojzdl_resize_create(int src_w, int src_h, int dst_w, int dst_h);
void ojzdl_resize_free(ojzdl_resize *r);
/* dst: dst_w x dst_h x 3. src: src_w x src_h x 3, packed. */
int ojzdl_resize_rgb(ojzdl_resize *r, const uint8_t *src, uint8_t *dst);
/* Same, pulling source rows on demand (src_w x 3 RGB bytes each); a row is
 * used before the next call, so get_row may reuse one buffer. Each source
 * row is requested at most once per call, in increasing order. */
int ojzdl_resize_rows(ojzdl_resize *r, const uint8_t *(*get_row)(void *ctx, int y), void *ctx,
                      uint8_t *dst);

#ifdef __cplusplus
}
#endif
#endif
