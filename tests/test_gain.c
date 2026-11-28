/*
 * test_gain.c -- TotalGainDb (log2 Q16) to the linear [24.8] gain
 */

#include <math.h>
#include <stdint.h>

#include "greatest.h"
#include "hal_gain.h"

TEST zero_is_unity(void)
{
    ASSERT_EQ_FMT(256u, hal_gain_log2q16_to_linear_q8(0), "%u");
    PASS();
}

TEST whole_stops_double(void)
{
    ASSERT_EQ_FMT(512u, hal_gain_log2q16_to_linear_q8(1u << 16), "%u");
    ASSERT_EQ_FMT(256u << 10, hal_gain_log2q16_to_linear_q8(10u << 16), "%u");
    PASS();
}

/* Two points measured on a T41 (os04d10), 5.16 and 6.73 stops */
TEST measured_t41_points(void)
{
    uint32_t a = hal_gain_log2q16_to_linear_q8(338004);
    uint32_t b = hal_gain_log2q16_to_linear_q8(441001);
    ASSERT_IN_RANGE(9137, a, 2);
    ASSERT_IN_RANGE(27159, b, 2);
    PASS();
}

TEST saturates_instead_of_wrapping(void)
{
    ASSERT_EQ_FMT(UINT32_MAX, hal_gain_log2q16_to_linear_q8(24u << 16), "%u");
    ASSERT_EQ_FMT(UINT32_MAX, hal_gain_log2q16_to_linear_q8(UINT32_MAX), "%u");
    /* 23.99999 stops is the largest input that still fits in 32 bits */
    ASSERT(hal_gain_log2q16_to_linear_q8((23u << 16) | 0xffffu) > (1u << 31));
    PASS();
}

TEST tracks_exp2_across_the_range(void)
{
    uint32_t prev = 0;
    for (uint32_t v = 0; v < (20u << 16); v += 977) {
        uint32_t got = hal_gain_log2q16_to_linear_q8(v);
        double want = 256.0 * exp2((double)v / 65536.0);
        ASSERT(got >= prev);
        ASSERT(fabs((double)got - want) <= 1.0 + want * 1e-6);
        prev = got;
    }
    PASS();
}

SUITE(gain_suite)
{
    RUN_TEST(zero_is_unity);
    RUN_TEST(whole_stops_double);
    RUN_TEST(measured_t41_points);
    RUN_TEST(saturates_instead_of_wrapping);
    RUN_TEST(tracks_exp2_across_the_range);
}
