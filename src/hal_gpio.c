/*
 * hal_gpio.c -- Raptor HAL GPIO and IR-cut implementation
 *
 * Simple sysfs GPIO operations.  No vendor SDK dependency.
 *
 * GPIO access is done through /sys/class/gpio/gpio{pin}/value.
 * The pin is assumed to be already exported and direction set
 * by the system init scripts or device tree.
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
 * GPIO SET
 *
 * Write a value (0 or 1) to /sys/class/gpio/gpio{pin}/value.
 * Exports the pin first if needed.
 * ================================================================ */

int hal_gpio_set(void *ctx, int pin, int value)
{
    (void)ctx;
    char path[GPIO_PATH_MAX];
    int fd;

    if (pin < 0)
        return RSS_ERR_INVAL;

    /* Ensure pin is exported */
    gpio_export(pin);

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
