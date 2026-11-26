/*
 * log_capture.c -- the HAL log sink for host tests: keeps the last message
 */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "hal_log.h"
#include "log_capture.h"

log_capture_t logged;

static void capture(int level, const char *file, int line, const char *fmt, ...)
{
    (void)file;
    (void)line;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(logged.msg, sizeof(logged.msg), fmt, ap);
    va_end(ap);
    logged.level = level;
    if (level == HAL_LOG_LVL_WARN)
        logged.warns++;
}

rss_hal_log_func_t rss_hal_log_fn = capture;

void log_capture_reset(void)
{
    memset(&logged, 0, sizeof(logged));
}
