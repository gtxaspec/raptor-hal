/*
 * hal_log.h -- HAL logging, free of vendor SDK headers so host-tested
 * modules can log
 */

#ifndef HAL_LOG_H
#define HAL_LOG_H

#include "raptor_hal.h"

/* Calls through a function pointer (default: fprintf to stderr).
 * Daemons call rss_hal_set_log_func() at init to redirect to syslog. */

/* Log levels matching RSS: 0=fatal, 1=error, 2=warn, 3=info, 4=debug */
#define HAL_LOG_LVL_ERR 1
#define HAL_LOG_LVL_WARN 2
#define HAL_LOG_LVL_INFO 3
#define HAL_LOG_LVL_DBG 4

extern rss_hal_log_func_t rss_hal_log_fn;

#define HAL_LOG_ERR(fmt, ...)                                                                      \
    rss_hal_log_fn(HAL_LOG_LVL_ERR, __FILE__, __LINE__, fmt, ##__VA_ARGS__)
#define HAL_LOG_WARN(fmt, ...)                                                                     \
    rss_hal_log_fn(HAL_LOG_LVL_WARN, __FILE__, __LINE__, fmt, ##__VA_ARGS__)
#define HAL_LOG_INFO(fmt, ...)                                                                     \
    rss_hal_log_fn(HAL_LOG_LVL_INFO, __FILE__, __LINE__, fmt, ##__VA_ARGS__)

#ifdef HAL_DEBUG
#define HAL_LOG_DBG(fmt, ...)                                                                      \
    rss_hal_log_fn(HAL_LOG_LVL_DBG, __FILE__, __LINE__, fmt, ##__VA_ARGS__)
#else
#define HAL_LOG_DBG(fmt, ...) ((void)0)
#endif

#endif /* HAL_LOG_H */
