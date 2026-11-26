/*
 * hal_mode.c -- judging an ISP running-mode switch
 */

#include "hal_log.h"
#include "hal_mode.h"

int hal_mode_switch_result(int set_ret, int get_ret, rss_isp_mode_t now, rss_isp_mode_t want)
{
    if (get_ret != RSS_OK)
        return set_ret;
    if (now != want) {
        HAL_LOG_WARN("ISP did not switch to %s mode (set returned %d; no tuning bin for it?)",
                     want == RSS_ISP_NIGHT ? "night" : "day", set_ret);
        return RSS_ERR_IO;
    }
    return RSS_OK;
}
