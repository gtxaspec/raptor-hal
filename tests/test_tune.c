/*
 * test_tune.c -- the tuning values a running-mode switch must preserve
 */

#include <errno.h>
#include <string.h>

#include "greatest.h"
#include "hal_tune.h"
#include "log_capture.h"

#define MAX_CALLS 64

static struct {
    int n;
    int sensor[MAX_CALLS];
    int key[MAX_CALLS];
    int val[MAX_CALLS];
    rss_wb_config_t wb;
    uint16_t coefft[3];
    int ret_for[HAL_TUNE_COUNT];
} calls;

static int fake_int(void *ctx, int sensor, hal_tune_key_t key, int val)
{
    (void)ctx;
    calls.sensor[calls.n] = sensor;
    calls.key[calls.n] = key;
    calls.val[calls.n] = val;
    calls.n++;
    return calls.ret_for[key];
}

static int fake_wb(void *ctx, int sensor, const rss_wb_config_t *wb)
{
    (void)ctx;
    calls.sensor[calls.n] = sensor;
    calls.key[calls.n] = HAL_TUNE_WB;
    calls.n++;
    calls.wb = *wb;
    return calls.ret_for[HAL_TUNE_WB];
}

static int fake_coefft(void *ctx, int sensor, uint16_t r, uint16_t g, uint16_t b)
{
    (void)ctx;
    calls.sensor[calls.n] = sensor;
    calls.key[calls.n] = HAL_TUNE_AWB_COEFFT;
    calls.n++;
    calls.coefft[0] = r;
    calls.coefft[1] = g;
    calls.coefft[2] = b;
    return calls.ret_for[HAL_TUNE_AWB_COEFFT];
}

static const hal_tune_ops_t ops = {fake_int, fake_wb, fake_coefft};

static void reset(hal_tune_t *t)
{
    memset(&calls, 0, sizeof(calls));
    memset(t, 0, sizeof(*t));
}

TEST nothing_recorded_replays_nothing(void)
{
    hal_tune_t t;
    reset(&t);
    ASSERT_EQ(0, hal_tune_replay(&t, &ops, NULL, 0));
    ASSERT_EQ(0, calls.n);
    PASS();
}

TEST the_last_value_set_is_replayed_once(void)
{
    hal_tune_t t;
    reset(&t);
    hal_tune_record(&t, HAL_TUNE_ANTIFLICKER, RSS_ANTIFLICKER_60HZ);
    hal_tune_record(&t, HAL_TUNE_ANTIFLICKER, RSS_ANTIFLICKER_50HZ);
    hal_tune_replay(&t, &ops, NULL, 0);
    ASSERT_EQ(1, calls.n);
    ASSERT_EQ(HAL_TUNE_ANTIFLICKER, calls.key[0]);
    ASSERT_EQ(RSS_ANTIFLICKER_50HZ, calls.val[0]);
    PASS();
}

TEST replay_follows_key_order_not_set_order(void)
{
    hal_tune_t t;
    reset(&t);
    hal_tune_record(&t, HAL_TUNE_DEFOG_STRENGTH, 150);
    hal_tune_record(&t, HAL_TUNE_ANTIFLICKER, RSS_ANTIFLICKER_50HZ);
    hal_tune_record(&t, HAL_TUNE_DEFOG, 1);
    hal_tune_record(&t, HAL_TUNE_MAX_AGAIN, 120);
    hal_tune_replay(&t, &ops, NULL, 0);
    ASSERT_EQ(4, calls.n);
    ASSERT_EQ(HAL_TUNE_MAX_AGAIN, calls.key[0]);
    ASSERT_EQ(HAL_TUNE_DEFOG, calls.key[1]); /* the enable before its strength */
    ASSERT_EQ(HAL_TUNE_DEFOG_STRENGTH, calls.key[2]);
    ASSERT_EQ(HAL_TUNE_ANTIFLICKER, calls.key[3]);
    PASS();
}

TEST white_balance_replays_whole(void)
{
    hal_tune_t t;
    reset(&t);
    rss_wb_config_t wb = {.mode = RSS_WB_MANUAL, .r_gain = 300, .g_gain = 256, .b_gain = 200};
    hal_tune_record_coefft(&t, 1100, 1024, 900);
    hal_tune_record_wb(&t, &wb);
    hal_tune_replay(&t, &ops, NULL, 0);
    ASSERT_EQ(2, calls.n);
    ASSERT_EQ(HAL_TUNE_WB, calls.key[0]); /* the mode before its coefficients */
    ASSERT_EQ(HAL_TUNE_AWB_COEFFT, calls.key[1]);
    ASSERT_EQ(RSS_WB_MANUAL, calls.wb.mode);
    ASSERT_EQ(300, calls.wb.r_gain);
    ASSERT_EQ(200, calls.wb.b_gain);
    ASSERT_EQ(1100, calls.coefft[0]);
    ASSERT_EQ(900, calls.coefft[2]);
    PASS();
}

TEST struct_keys_are_not_recorded_as_ints(void)
{
    hal_tune_t t;
    reset(&t);
    hal_tune_record(&t, HAL_TUNE_WB, 1);
    hal_tune_record(&t, HAL_TUNE_AWB_COEFFT, 1);
    hal_tune_record(&t, HAL_TUNE_COUNT, 1);
    hal_tune_record(&t, (hal_tune_key_t)-1, 1);
    hal_tune_replay(&t, &ops, NULL, 0);
    ASSERT_EQ(0, calls.n);
    PASS();
}

TEST failures_are_counted_unsupported_is_not(void)
{
    hal_tune_t t;
    reset(&t);
    hal_tune_record(&t, HAL_TUNE_BRIGHTNESS, 150);
    hal_tune_record(&t, HAL_TUNE_SINTER, 160);
    hal_tune_record(&t, HAL_TUNE_MAX_AGAIN, 120);
    calls.ret_for[HAL_TUNE_BRIGHTNESS] = -EIO; /* first in replay order */
    calls.ret_for[HAL_TUNE_SINTER] = RSS_ERR_NOTSUP;
    ASSERT_EQ(1, hal_tune_replay(&t, &ops, NULL, 0));
    ASSERT_EQ(3, calls.n); /* a failure does not stop the rest */
    PASS();
}

TEST replay_targets_the_given_sensor(void)
{
    hal_tune_t t;
    reset(&t);
    hal_tune_record(&t, HAL_TUNE_HUE, 140);
    hal_tune_replay(&t, &ops, NULL, 1);
    ASSERT_EQ(1, calls.n);
    ASSERT_EQ(1, calls.sensor[0]);
    PASS();
}

TEST mode_change_is_tracked_from_day(void)
{
    hal_tune_t t;
    reset(&t);
    ASSERT_FALSE(hal_tune_mode_changed(&t, RSS_ISP_DAY)); /* libimp starts in day */
    ASSERT(hal_tune_mode_changed(&t, RSS_ISP_NIGHT));
    ASSERT_FALSE(hal_tune_mode_changed(&t, RSS_ISP_NIGHT));
    ASSERT(hal_tune_mode_changed(&t, RSS_ISP_DAY));
    PASS();
}

TEST a_switch_replay_with_failures_warns_once(void)
{
    hal_tune_t t;
    reset(&t);
    log_capture_reset();
    hal_tune_record(&t, HAL_TUNE_BRIGHTNESS, 150);
    hal_tune_record(&t, HAL_TUNE_MAX_AGAIN, 120);
    calls.ret_for[HAL_TUNE_BRIGHTNESS] = -EIO;
    calls.ret_for[HAL_TUNE_MAX_AGAIN] = -EIO;
    ASSERT_EQ(2, hal_tune_replay_switch(&t, &ops, NULL, 1, RSS_ISP_NIGHT));
    ASSERT_EQ(1, logged.warns);
    ASSERT(strstr(logged.msg, "sensor 1: 2 tuning value(s) not restored after the night switch"));
    PASS();
}

TEST a_clean_switch_replay_is_silent(void)
{
    hal_tune_t t;
    reset(&t);
    log_capture_reset();
    hal_tune_record(&t, HAL_TUNE_BRIGHTNESS, 150);
    calls.ret_for[HAL_TUNE_SINTER] = RSS_ERR_NOTSUP;
    ASSERT_EQ(0, hal_tune_replay_switch(&t, &ops, NULL, 0, RSS_ISP_DAY));
    ASSERT_EQ(1, calls.n);
    ASSERT_EQ(0, logged.warns);
    PASS();
}

SUITE(tune_suite)
{
    RUN_TEST(nothing_recorded_replays_nothing);
    RUN_TEST(the_last_value_set_is_replayed_once);
    RUN_TEST(replay_follows_key_order_not_set_order);
    RUN_TEST(white_balance_replays_whole);
    RUN_TEST(struct_keys_are_not_recorded_as_ints);
    RUN_TEST(failures_are_counted_unsupported_is_not);
    RUN_TEST(replay_targets_the_given_sensor);
    RUN_TEST(mode_change_is_tracked_from_day);
    RUN_TEST(a_switch_replay_with_failures_warns_once);
    RUN_TEST(a_clean_switch_replay_is_silent);
}
