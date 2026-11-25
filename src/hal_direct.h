/*
 * hal_direct.h -- ISP direct mode (IVDC) as the ISP driver was loaded
 */

#ifndef HAL_DIRECT_H
#define HAL_DIRECT_H

#include "raptor_hal.h"

/* Number of sensors whose main channel the driver feeds straight to the
 * encoder, read from a direct_mode module parameter file. RSS_ERR_NOTSUP
 * when the file does not exist, RSS_ERR_IO when it holds no count; *count
 * is left alone on any error. */
int hal_direct_sensors_from_file(const char *path, int *count);

#endif
