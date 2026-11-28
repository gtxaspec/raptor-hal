/*
 * test_ojzdl.c -- ojzdl (OJZDL=1) against libjzdl's own output
 *
 * ojzdl/tiny.bin: a 15-layer JZDL model with random weights (ojzdl/gen_tiny.py) covering the
 * layer types of thingino-ai's models. ojzdl/tiny.ref.f32: libjzdl 1.7.0's output for it on
 * ojzdl/tiny.rgb (jzdlrun under QEMU; int32 w, h, c, then the floats). The host build is the
 * plain C path; the T31's MXU2 kernels give the same bytes (thingino-ai tools/ojzdl/gate.sh).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "greatest.h"
#include "ojzdl/ojzdl.h"

#define DIR "ojzdl/"

static uint8_t *slurp(const char *path, size_t *len)
{
    FILE *f = fopen(path, "rb");
    uint8_t *b = NULL;
    long n;

    if (!f)
        return NULL;
    if (fseek(f, 0, SEEK_END) == 0 && (n = ftell(f)) > 0 && fseek(f, 0, SEEK_SET) == 0) {
        b = malloc((size_t)n);
        if (b && fread(b, 1, (size_t)n, f) != (size_t)n) {
            free(b);
            b = NULL;
        }
        *len = (size_t)n;
    }
    fclose(f);
    return b;
}

TEST tiny_model_output_is_bit_identical_to_libjzdl(void)
{
    size_t rlen = 0, ilen = 0, n = 0;
    const char *err = NULL;
    uint8_t *ref = slurp(DIR "tiny.ref.f32", &rlen);
    uint8_t *rgb = slurp(DIR "tiny.rgb", &ilen);
    ojzdl_net *net = ojzdl_load_file(DIR "tiny.bin", &err);

    ASSERT(ref && rgb);
    ASSERTm(err ? err : "load failed", net != NULL);
    ASSERT_EQ((size_t)ojzdl_in_h(net) * ojzdl_in_w(net) * ojzdl_in_c(net), ilen);
    ASSERT_EQ(0, ojzdl_input_rgb(net, rgb));
    ASSERT_EQ(0, ojzdl_forward(net));
    const float *out = ojzdl_output(net, &n);
    ASSERT(out != NULL);
    ASSERT_EQ(rlen, 12 + n * sizeof(float));
    ASSERT_MEM_EQ(ref + 12, out, n * sizeof(float));

    /* a second forward on the same input gives the same bytes (no state left behind) */
    ASSERT_EQ(0, ojzdl_input_rgb(net, rgb));
    ASSERT_EQ(0, ojzdl_forward(net));
    ASSERT_MEM_EQ(ref + 12, ojzdl_output(net, &n), n * sizeof(float));

    ojzdl_free(net);
    free(ref);
    free(rgb);
    PASS();
}

/* the first conv's int32 shift blob: after the param section, its 432 weight bytes and the
 * 16-entry int32 bias and mult blobs */
static size_t first_shift_offset(const uint8_t *b)
{
    uint32_t param_bytes =
        (uint32_t)b[0] | (uint32_t)b[1] << 8 | (uint32_t)b[2] << 16 | (uint32_t)b[3] << 24;

    return 24 + (size_t)param_bytes + 432 + 2 * 16 * 4;
}

TEST requant_shift_out_of_range_is_refused(void)
{
    static const int32_t bad[] = {32, 70, -1};
    size_t len = 0;
    uint8_t *b = slurp(DIR "tiny.bin", &len);

    ASSERT(b != NULL);
    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
        const char *err = NULL;
        size_t off = first_shift_offset(b);
        uint8_t save[4];

        memcpy(save, b + off, 4);
        ASSERT(save[0] >= 6 && save[0] <= 10 && !save[1] && !save[2] && !save[3]); /* gen_tiny.py */
        for (int k = 0; k < 4; k++)
            b[off + k] = (uint8_t)((uint32_t)bad[i] >> (8 * k));
        ASSERT_EQ(NULL, ojzdl_load(b, len, &err));
        ASSERT(err && strstr(err, "shift") != NULL);
        memcpy(b + off, save, 4);
    }
    free(b);
    PASS();
}

TEST truncated_model_is_refused(void)
{
    size_t len = 0;
    const char *err = NULL;
    uint8_t *b = slurp(DIR "tiny.bin", &len);

    ASSERT(b != NULL);
    ASSERT_EQ(NULL, ojzdl_load(b, len - 1, &err));
    ASSERT(err != NULL);
    free(b);
    PASS();
}

SUITE(ojzdl_suite)
{
    RUN_TEST(tiny_model_output_is_bit_identical_to_libjzdl);
    RUN_TEST(requant_shift_out_of_range_is_refused);
    RUN_TEST(truncated_model_is_refused);
}
