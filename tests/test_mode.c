/*
 * test_mode.c -- judging a running-mode switch by the mode the ISP reports
 */

#include <string.h>

#include "greatest.h"
#include "hal_mode.h"
#include "log_capture.h"

TEST a_switch_that_took_is_ok_whatever_set_returned(void)
{
    log_capture_reset();
    /* T41 IMP-1.2.6 returns -1 from a switch that took */
    ASSERT_EQ(RSS_OK, hal_mode_switch_result(-1, RSS_OK, RSS_ISP_NIGHT, RSS_ISP_NIGHT));
    ASSERT_EQ(0, logged.warns);
    PASS();
}

TEST a_switch_that_did_not_take_is_an_io_error_and_a_warning(void)
{
    log_capture_reset();
    /* T32 with no night bin stays in day */
    ASSERT_EQ(RSS_ERR_IO, hal_mode_switch_result(0, RSS_OK, RSS_ISP_DAY, RSS_ISP_NIGHT));
    ASSERT_EQ(1, logged.warns);
    ASSERT(strstr(logged.msg, "did not switch to night mode") != NULL);
    PASS();
}

TEST the_warning_names_the_mode_asked_for(void)
{
    log_capture_reset();
    ASSERT_EQ(RSS_ERR_IO, hal_mode_switch_result(0, RSS_OK, RSS_ISP_NIGHT, RSS_ISP_DAY));
    ASSERT(strstr(logged.msg, "did not switch to day mode") != NULL);
    PASS();
}

TEST without_a_readback_the_set_result_stands(void)
{
    log_capture_reset();
    ASSERT_EQ(-4090, hal_mode_switch_result(-4090, RSS_ERR_NOTSUP, RSS_ISP_DAY, RSS_ISP_NIGHT));
    ASSERT_EQ(RSS_OK, hal_mode_switch_result(RSS_OK, RSS_ERR_IO, RSS_ISP_DAY, RSS_ISP_NIGHT));
    ASSERT_EQ(0, logged.warns);
    PASS();
}

SUITE(mode_suite)
{
    RUN_TEST(a_switch_that_took_is_ok_whatever_set_returned);
    RUN_TEST(a_switch_that_did_not_take_is_an_io_error_and_a_warning);
    RUN_TEST(the_warning_names_the_mode_asked_for);
    RUN_TEST(without_a_readback_the_set_result_stands);
}
