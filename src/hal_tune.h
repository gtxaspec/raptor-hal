/*
 * hal_tune.h -- ISP tuning values a running-mode switch must preserve
 *
 * IMP_ISP_Tuning_SetISPRunningMode reloads the tuning bin's block for the
 * new mode, dropping values set at runtime; which ones depends on the SoC
 * (T41/T32: anti-flicker; T31: max gains, backlight comp, highlight
 * depress). The HAL records every tuning value set through it and replays
 * them after a mode change, so a switch changes the mode and nothing else.
 */

#ifndef HAL_TUNE_H
#define HAL_TUNE_H

#include <stdbool.h>
#include <stdint.h>

#include "raptor_hal.h"

/* Replay order: enables before their strengths, WB before its coefficients */
typedef enum {
    HAL_TUNE_BRIGHTNESS,
    HAL_TUNE_CONTRAST,
    HAL_TUNE_SATURATION,
    HAL_TUNE_SHARPNESS,
    HAL_TUNE_HUE,
    HAL_TUNE_SINTER,
    HAL_TUNE_TEMPER,
    HAL_TUNE_AE_COMP,
    HAL_TUNE_MAX_AGAIN,
    HAL_TUNE_MAX_DGAIN,
    HAL_TUNE_DPC,
    HAL_TUNE_DRC,
    HAL_TUNE_HIGHLIGHT,
    HAL_TUNE_BACKLIGHT,
    HAL_TUNE_DEFOG,
    HAL_TUNE_DEFOG_STRENGTH,
    HAL_TUNE_ANTIFLICKER,
    HAL_TUNE_WB,
    HAL_TUNE_AWB_COEFFT,
    HAL_TUNE_COUNT
} hal_tune_key_t;

typedef struct {
    uint32_t set; /* bit per key that has been recorded */
    int val[HAL_TUNE_COUNT];
    rss_wb_config_t wb;
    uint16_t coefft[3];
    rss_isp_mode_t mode; /* the mode last set; libimp starts in day */
} hal_tune_t;

typedef struct {
    int (*set_int)(void *ctx, int sensor, hal_tune_key_t key, int val);
    int (*set_wb)(void *ctx, int sensor, const rss_wb_config_t *wb);
    int (*set_coefft)(void *ctx, int sensor, uint16_t r, uint16_t g, uint16_t b);
} hal_tune_ops_t;

void hal_tune_record(hal_tune_t *t, hal_tune_key_t key, int val);
void hal_tune_record_wb(hal_tune_t *t, const rss_wb_config_t *wb);
void hal_tune_record_coefft(hal_tune_t *t, uint16_t r, uint16_t g, uint16_t b);

/* Note a successful switch to mode; true if it differs from the last one. */
bool hal_tune_mode_changed(hal_tune_t *t, rss_isp_mode_t mode);

/* Replay the recorded values in key order. Returns the number of setters
 * that failed; unsupported ones are not counted. */
int hal_tune_replay(const hal_tune_t *t, const hal_tune_ops_t *ops, void *ctx, int sensor);

/* hal_tune_replay after a switch to mode, warning when values were lost */
int hal_tune_replay_switch(const hal_tune_t *t, const hal_tune_ops_t *ops, void *ctx, int sensor,
                           rss_isp_mode_t mode);

#endif /* HAL_TUNE_H */
