/*
 * hal_gain.h -- ISP gain unit conversions
 */

#ifndef HAL_GAIN_H
#define HAL_GAIN_H

#include <stdint.h>

/*
 * T40/T41 report TotalGainDb as log2 of the linear total gain in 16.16
 * fixed point, whatever the SDK calls it (a T41 with an os04d10 matches
 * log2(ExposureValue / AeIntegrationTime) * 65536 to within 3 counts).
 * Every other platform reports the linear [24.8] value, which is what the
 * IR-cut thresholds (night_gain, day_gain_pct, probe_gain_pct) assume.
 *
 * Returns 256 * 2^(log2q16 / 65536), saturating at UINT32_MAX. Integer
 * only: no libm on the target.
 */
uint32_t hal_gain_log2q16_to_linear_q8(uint32_t log2q16);

#endif
