/*
 * hal_gpio.c -- Raptor HAL GPIO and IR-cut implementation
 *
 * Simple sysfs GPIO operations.  No vendor SDK dependency.
 *
 * GPIO access is done through /sys/class/gpio/gpio{pin}/value,
 * exporting the pin first if the system has not already.
 *
 * ================================================================
 * DIRECTION IS THIS FILE'S JOB, AND ONLY ON WRITES
 *
 * These functions used to assume "the pin is already exported and
 * direction set by the system init scripts or device tree", and export
 * it themselves if not.  Those two halves contradict each other: a pin
 * this file had to export is a pin nothing else configured, and a fresh
 * sysfs export leaves the line an *input*.  Writing to value then fails
 * with EPERM, so hal_gpio_set could not drive a pin that was not already
 * set up elsewhere -- which on an OpenIPC image is every pin.
 *
 * So hal_gpio_set now ensures the direction is "out", and does it by
 * reading the current direction first and writing only on a mismatch.
 * Two reasons, both about not disturbing a working board: writing
 * "out" resets the line low on most drivers, so an unconditional write
 * would pulse a pin low on every set, and a pin somebody else has
 * deliberately configured stays as they configured it.
 *
 * hal_gpio_get deliberately does *not* force "in".  Reading back a pin
 * this process drives is a legitimate use, and switching an output to
 * input to read it would release whatever it holds -- on an IR-cut
 * driver, that means letting go of the filter.  A freshly exported pin
 * is already an input, which is what a photosensor needs.
 * ================================================================
 *
 * IR-cut control is board-specific: pin numbers, polarity, whether the
 * filter is one line or an H-bridge pair.  That configuration lives in
 * raptor.conf's [ircut] section, which ric reads -- so ric drives the
 * filter directly (ric_daynight.c) and ircut_set stays a stub here.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: MIT
 */

#include "hal_internal.h"

#include <fcntl.h>
#include <unistd.h>

/* Maximum path length for sysfs GPIO files */
#define GPIO_PATH_MAX 128

/*
 * The sysfs root, overridable at compile time so the host tests can point
 * it at a temporary directory. The export-then-direction-then-value
 * sequence is the whole of the logic here and there is no way to observe
 * it off-target otherwise; a camera is a poor place to discover that a
 * write went to the wrong file.
 */
#ifndef GPIO_SYSFS_ROOT
#define GPIO_SYSFS_ROOT "/sys/class/gpio"
#endif

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

    fd = open(GPIO_SYSFS_ROOT "/export", O_WRONLY);
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
 * Make the pin an output, writing "out" only when it is not one
 * already -- see the header comment for why the read comes first.
 * ================================================================ */

static int gpio_set_output(int pin)
{
    char path[GPIO_PATH_MAX];
    char buf[8];
    int fd;
    ssize_t n;

    snprintf(path, sizeof(path), GPIO_SYSFS_ROOT "/gpio%d/direction", pin);

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
        /* No direction attribute at all: some drivers export a
         * fixed-function line without one. Leave it to the value
         * write to succeed or fail on its own. */
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

    snprintf(path, sizeof(path), GPIO_SYSFS_ROOT "/gpio%d/value", pin);

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

    snprintf(path, sizeof(path), GPIO_SYSFS_ROOT "/gpio%d/value", pin);

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
 * A stub, on every platform, and not a gap waiting to be filled.
 *
 * Driving an IR-cut filter needs the pin numbers, the polarity, whether
 * it is one line or an H-bridge pair, and the pulse width -- board
 * configuration, none of which the HAL has or should acquire.  raptor
 * keeps it in raptor.conf's [ircut] section, and ric reads that and
 * pulses the pins itself through the same sysfs interface this file uses
 * (ric_daynight.c).  Nothing in the tree calls this op.
 *
 * It stays in the vtable because removing an op from an ABI to delete
 * three lines is a worse trade than leaving it returning RSS_OK, and
 * because a backend where the filter really is behind a vendor call
 * (rather than a GPIO) would want exactly this signature.
 * ================================================================ */

int hal_ircut_set(void *ctx, int state)
{
    (void)ctx;

    HAL_LOG_INFO("ircut_set: state=%d ignored -- ric drives the filter from [ircut] config",
                 state);

    return RSS_OK;
}
