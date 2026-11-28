/*
 * hal_gain.c -- ISP gain unit conversions (host-tested, see hal_gain.h)
 */

#include "hal_gain.h"

uint32_t hal_gain_log2q16_to_linear_q8(uint32_t log2q16)
{
    /* 2^(2^-k) in Q30, k = 1..16 */
    static const uint32_t pow2_frac_q30[16] = {
        1518500250u, 1276901417u, 1170923762u, 1121280436u, 1097253708u, 1085434106u,
        1079572136u, 1076653033u, 1075196443u, 1074468888u, 1074105294u, 1073923544u,
        1073832680u, 1073787251u, 1073764537u, 1073753181u,
    };
    uint32_t ip = log2q16 >> 16;
    uint32_t fp = log2q16 & 0xffffu;
    uint64_t r = 1ull << 30; /* 1.0 in Q30 */

    if (ip >= 24)
        return UINT32_MAX;
    for (int k = 0; k < 16; k++) {
        if (fp & (0x8000u >> k))
            r = (r * pow2_frac_q30[k] + (1ull << 29)) >> 30;
    }
    /* scale by the integer stops (< 2^54, fits), then Q30 -> Q8 */
    r = ((r << ip) + (1ull << 21)) >> 22;
    return r > UINT32_MAX ? UINT32_MAX : (uint32_t)r;
}
