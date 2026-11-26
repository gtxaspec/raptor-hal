/*
 * hal_mode.h -- judging an ISP running-mode switch
 */

#ifndef HAL_MODE_H
#define HAL_MODE_H

#include "raptor_hal.h"

/* set_ret: SetISPRunningMode's result. get_ret/now: the readback. The
 * readback decides (T41 returns -1 from a switch that took); without
 * one, set_ret stands. RSS_ERR_IO, with a warning, when the ISP stayed. */
int hal_mode_switch_result(int set_ret, int get_ret, rss_isp_mode_t now, rss_isp_mode_t want);

#endif
