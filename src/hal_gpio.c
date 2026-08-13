/*
 * hal_gpio.c -- Raptor HAL GPIO and IR-cut implementation
 *
 * Simple sysfs GPIO operations.  No vendor SDK dependency.
 *
 * GPIO access is done through /sys/class/gpio/gpio{pin}/value,
 * exporting the pin first if the system has not already.
 *
 * A pin this file exports is a pin nothing else configured, and a fresh
 * export leaves the line an input, so hal_gpio_set sets the direction
 * itself or the value write fails with EPERM.  It reads the direction
 * first and writes "out" only on a mismatch: an unconditional write
 * resets the line low on most drivers, and a pin deliberately configured
 * elsewhere stays as it was.
 *
 * hal_gpio_get does not force "in".  Turning an output round to read it
 * would release whatever it holds -- on an IR-cut driver, the filter --
 * and a freshly exported pin is already an input.
 *
 * IR-cut control is board-specific and requires configuration
 * that maps logical state (day/night) to physical GPIO pins.
 * For now, ircut_set is a stub that logs and returns success.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: MIT
 */

#include "hal_internal.h"

#include <fcntl.h>
#include <unistd.h>

/* Maximum path length for sysfs GPIO files */
#define GPIO_PATH_MAX 64

/* ================================================================
 * GPIO EXPORT HELPER
 *
 * Ensures a GPIO pin is exported via sysfs before use.
 * If already exported, the write to /sys/class/gpio/export
 * will fail with EBUSY -- we silently ignore that.
 * ================================================================ */

static int gpio_export(int pin)
{
    int fd;
    char buf[16];
    int len;

    fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd < 0)
        return RSS_ERR_IO;

    len = snprintf(buf, sizeof(buf), "%d", pin);
    if (write(fd, buf, len) < 0) {
        /* EBUSY or EINVAL means already exported or invalid pin */
        int err = errno;
        close(fd);
        if (err == EBUSY)
            return RSS_OK; /* already exported */
        return RSS_ERR_IO;
    }

    close(fd);
    return RSS_OK;
}

/* ================================================================
 * DIRECTION HELPER
 *
 * Make the pin an output, writing "out" only on a mismatch.
 * ================================================================ */

static int gpio_set_output(int pin)
{
    char path[GPIO_PATH_MAX];
    char buf[8];
    int fd;
    ssize_t n;

    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", pin);

    fd = open(path, O_RDONLY);
    if (fd >= 0) {
        memset(buf, 0, sizeof(buf));
        n = read(fd, buf, sizeof(buf) - 1);
        close(fd);
        if (n > 0 && strncmp(buf, "out", 3) == 0)
            return RSS_OK;
    }

    fd = open(path, O_WRONLY);
    if (fd < 0) {
        /* Some drivers export a fixed-function line with no direction
         * attribute; leave the value write to succeed or fail on its own. */
        return RSS_ERR_IO;
    }

    if (write(fd, "out", 3) != 3) {
        HAL_LOG_ERR("gpio_set: direction out on pin %d failed", pin);
        close(fd);
        return RSS_ERR_IO;
    }

    close(fd);
    return RSS_OK;
}

/* ================================================================
 * GPIO SET
 *
 * Write a value (0 or 1) to /sys/class/gpio/gpio{pin}/value.
 * Exports the pin and makes it an output first if needed.
 * ================================================================ */

int hal_gpio_set(void *ctx, int pin, int value)
{
    (void)ctx;
    char path[GPIO_PATH_MAX];
    int fd;

    if (pin < 0)
        return RSS_ERR_INVAL;

    /* Ensure pin is exported and driving */
    gpio_export(pin);
    gpio_set_output(pin);

    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", pin);

    fd = open(path, O_WRONLY);
    if (fd < 0) {
        HAL_LOG_ERR("gpio_set: open %s failed", path);
        return RSS_ERR_IO;
    }

    if (write(fd, value ? "1" : "0", 1) != 1) {
        HAL_LOG_ERR("gpio_set: write to %s failed", path);
        close(fd);
        return RSS_ERR_IO;
    }

    close(fd);
    return RSS_OK;
}

/* ================================================================
 * GPIO GET
 *
 * Read a value (0 or 1) from /sys/class/gpio/gpio{pin}/value.
 * Exports the pin if needed but does not touch its direction.
 * ================================================================ */

int hal_gpio_get(void *ctx, int pin, int *value)
{
    (void)ctx;
    char path[GPIO_PATH_MAX];
    char buf[4];
    int fd;

    if (pin < 0 || !value)
        return RSS_ERR_INVAL;

    /* Ensure pin is exported */
    gpio_export(pin);

    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", pin);

    fd = open(path, O_RDONLY);
    if (fd < 0) {
        HAL_LOG_ERR("gpio_get: open %s failed", path);
        return RSS_ERR_IO;
    }

    memset(buf, 0, sizeof(buf));
    if (read(fd, buf, sizeof(buf) - 1) < 0) {
        HAL_LOG_ERR("gpio_get: read from %s failed", path);
        close(fd);
        return RSS_ERR_IO;
    }

    close(fd);

    *value = (buf[0] == '1') ? 1 : 0;
    return RSS_OK;
}

/* ================================================================
 * IR-CUT CONTROL
 *
 * IR-cut filter control is board-specific: different boards use
 * different GPIO pins, polarities, and timing.  The mapping from
 * logical state (0=day/closed, 1=night/open) to physical GPIO
 * toggling must come from board configuration.
 *
 * For now this is a stub.  A real implementation would:
 *   1. Read ircut GPIO pin numbers from board config
 *   2. Pulse the appropriate pin(s) to engage/disengage the filter
 *   3. Some boards use a single GPIO, others use two (for H-bridge)
 * ================================================================ */

int hal_ircut_set(void *ctx, int state)
{
    (void)ctx;

    HAL_LOG_INFO("ircut_set: state=%d (stub, board-specific config needed)", state);

    /* TODO: implement board-specific IR-cut GPIO control.
     * This requires reading pin configuration from a config file
     * or device tree overlay:
     *
     *   int ircut_gpio1 = board_cfg->ircut_pin1;
     *   int ircut_gpio2 = board_cfg->ircut_pin2;
     *
     *   if (state) {  // night mode: open IR-cut filter
     *       hal_gpio_set(ctx, ircut_gpio1, 0);
     *       hal_gpio_set(ctx, ircut_gpio2, 1);
     *   } else {      // day mode: close IR-cut filter
     *       hal_gpio_set(ctx, ircut_gpio1, 1);
     *       hal_gpio_set(ctx, ircut_gpio2, 0);
     *   }
     *   usleep(100000);  // 100ms pulse
     *   hal_gpio_set(ctx, ircut_gpio1, 0);
     *   hal_gpio_set(ctx, ircut_gpio2, 0);
     */

    return RSS_OK;
}
