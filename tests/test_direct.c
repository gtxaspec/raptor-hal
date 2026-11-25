/*
 * test_direct.c -- reading the ISP driver's direct_mode parameter
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "greatest.h"
#include "hal_direct.h"

static char path[64];

static void write_param(const char *text)
{
    snprintf(path, sizeof(path), "/tmp/hal_direct_XXXXXX");
    int fd = mkstemp(path);
    if (fd < 0)
        abort();
    if (write(fd, text, strlen(text)) != (ssize_t)strlen(text))
        abort();
    close(fd);
}

static int read_param(const char *text, int *count)
{
    write_param(text);
    int ret = hal_direct_sensors_from_file(path, count);
    unlink(path);
    return ret;
}

TEST direct_one_sensor(void)
{
    int n = -1;
    ASSERT_EQ(RSS_OK, read_param("1\n", &n));
    ASSERT_EQ(1, n);
    PASS();
}

TEST direct_off(void)
{
    int n = -1;
    ASSERT_EQ(RSS_OK, read_param("0\n", &n));
    ASSERT_EQ(0, n);
    PASS();
}

TEST direct_four_sensors(void)
{
    int n = -1;
    ASSERT_EQ(RSS_OK, read_param("4", &n));
    ASSERT_EQ(4, n);
    PASS();
}

TEST no_parameter_is_unsupported(void)
{
    int n = 7;
    ASSERT_EQ(RSS_ERR_NOTSUP, hal_direct_sensors_from_file("/nonexistent/direct_mode", &n));
    ASSERT_EQ(7, n);
    PASS();
}

TEST garbage_is_an_error(void)
{
    int n = 7;
    ASSERT_EQ(RSS_ERR_IO, read_param("yes\n", &n));
    ASSERT_EQ(7, n);
    PASS();
}

TEST negative_is_an_error(void)
{
    int n = 7;
    ASSERT_EQ(RSS_ERR_IO, read_param("-1\n", &n));
    ASSERT_EQ(7, n);
    PASS();
}

TEST empty_is_an_error(void)
{
    int n = 7;
    ASSERT_EQ(RSS_ERR_IO, read_param("", &n));
    ASSERT_EQ(7, n);
    PASS();
}

TEST bare_newline_is_an_error(void)
{
    int n = 7;
    ASSERT_EQ(RSS_ERR_IO, read_param("\n", &n));
    ASSERT_EQ(7, n);
    PASS();
}

TEST null_count_is_invalid(void)
{
    ASSERT_EQ(RSS_ERR_INVAL, hal_direct_sensors_from_file("/nonexistent/direct_mode", NULL));
    PASS();
}

SUITE(direct_suite)
{
    RUN_TEST(direct_one_sensor);
    RUN_TEST(direct_off);
    RUN_TEST(direct_four_sensors);
    RUN_TEST(no_parameter_is_unsupported);
    RUN_TEST(garbage_is_an_error);
    RUN_TEST(negative_is_an_error);
    RUN_TEST(empty_is_an_error);
    RUN_TEST(bare_newline_is_an_error);
    RUN_TEST(null_count_is_invalid);
}
