#include "hal_tune.h"

void hal_tune_record(hal_tune_t *t, hal_tune_key_t key, int val)
{
    if (key < 0 || key >= HAL_TUNE_COUNT || key == HAL_TUNE_WB || key == HAL_TUNE_AWB_COEFFT)
        return;
    t->val[key] = val;
    t->set |= 1u << key;
}

void hal_tune_record_wb(hal_tune_t *t, const rss_wb_config_t *wb)
{
    t->wb = *wb;
    t->set |= 1u << HAL_TUNE_WB;
}

void hal_tune_record_coefft(hal_tune_t *t, uint16_t r, uint16_t g, uint16_t b)
{
    t->coefft[0] = r;
    t->coefft[1] = g;
    t->coefft[2] = b;
    t->set |= 1u << HAL_TUNE_AWB_COEFFT;
}

bool hal_tune_mode_changed(hal_tune_t *t, rss_isp_mode_t mode)
{
    bool changed = mode != t->mode;
    t->mode = mode;
    return changed;
}

int hal_tune_replay(const hal_tune_t *t, const hal_tune_ops_t *ops, void *ctx, int sensor)
{
    int failed = 0;

    for (int k = 0; k < HAL_TUNE_COUNT; k++) {
        if (!(t->set & (1u << k)))
            continue;
        int ret;
        if (k == HAL_TUNE_WB)
            ret = ops->set_wb(ctx, sensor, &t->wb);
        else if (k == HAL_TUNE_AWB_COEFFT)
            ret = ops->set_coefft(ctx, sensor, t->coefft[0], t->coefft[1], t->coefft[2]);
        else
            ret = ops->set_int(ctx, sensor, (hal_tune_key_t)k, t->val[k]);
        if (ret != RSS_OK && ret != RSS_ERR_NOTSUP)
            failed++;
    }
    return failed;
}
