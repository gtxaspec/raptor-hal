/*
 * Host-side test of src/hal_gpio.c.
 *
 * Not a SigmaStar file -- hal_gpio.c is plain sysfs and builds for every
 * platform -- but it is the file phase 6 depends on, and its logic is
 * entirely about which sysfs file gets written in which order. That is
 * observable on a host as long as the sysfs root can be moved, which is
 * what GPIO_SYSFS_ROOT is for.
 *
 * The regression these tests exist for: hal_gpio_set used to export a pin
 * and then write to its value without ever setting the direction, so it
 * could only drive pins something else had already configured as outputs.
 * A fresh sysfs export is an input, and writing value on an input fails.
 */

#include <assert.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

/* Must precede the include: hal_gpio.c only defines it if nobody has. */
#define GPIO_SYSFS_ROOT "/dev/shm/hal_gpio_test"

#include "hal_gpio.c"

static void quiet_log(int level, const char *file, int line, const char *fmt, ...)
{
    (void)level;
    (void)file;
    (void)line;
    (void)fmt;
}

rss_hal_log_func_t rss_hal_log_fn = quiet_log;

static int failures;

#define CHECK(cond, fmt, ...)                                                                      \
    do {                                                                                           \
        if (!(cond)) {                                                                             \
            printf("FAIL %s:%d: " fmt "\n", __func__, __LINE__, ##__VA_ARGS__);                    \
            failures++;                                                                            \
        }                                                                                          \
    } while (0)

/* ── fake sysfs ── */

static void write_file(const char *path, const char *content)
{
    int fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0644);

    assert(fd >= 0);
    assert(write(fd, content, strlen(content)) == (ssize_t)strlen(content));
    close(fd);
}

static void read_file(const char *path, char *buf, size_t len)
{
    int fd = open(path, O_RDONLY);
    ssize_t n;

    memset(buf, 0, len);
    if (fd < 0)
        return;
    n = read(fd, buf, len - 1);
    if (n < 0)
        n = 0;
    buf[n] = '\0';
    close(fd);
}

/*
 * Build a sysfs root with the export file and, optionally, one pin
 * directory. Absent pin directory stands for "the kernel has not created
 * it", which is how a bad pin number presents.
 */
static void fake_sysfs(int pin, const char *direction)
{
    char path[256];

    if (system("rm -rf " GPIO_SYSFS_ROOT) != 0)
        assert(0);
    assert(mkdir(GPIO_SYSFS_ROOT, 0755) == 0);
    write_file(GPIO_SYSFS_ROOT "/export", "");

    if (pin < 0)
        return;

    snprintf(path, sizeof(path), GPIO_SYSFS_ROOT "/gpio%d", pin);
    assert(mkdir(path, 0755) == 0);

    if (direction) {
        snprintf(path, sizeof(path), GPIO_SYSFS_ROOT "/gpio%d/direction", pin);
        write_file(path, direction);
    }

    snprintf(path, sizeof(path), GPIO_SYSFS_ROOT "/gpio%d/value", pin);
    write_file(path, "0\n");
}

static void read_pin_file(int pin, const char *attr, char *buf, size_t len)
{
    char path[256];

    snprintf(path, sizeof(path), GPIO_SYSFS_ROOT "/gpio%d/%s", pin, attr);
    read_file(path, buf, len);
}

/* ── tests ── */

static void test_set_exports_and_drives(void)
{
    char buf[32];

    /* The state a fresh export leaves behind: an input. The value file
     * keeps its trailing newline because a 1-byte write does not truncate,
     * which real sysfs would; only the first byte is the reading. */
    fake_sysfs(23, "in\n");

    CHECK(hal_gpio_set(NULL, 23, 1) == RSS_OK, "driving an exported pin must succeed");

    read_file(GPIO_SYSFS_ROOT "/export", buf, sizeof(buf));
    CHECK(strcmp(buf, "23") == 0, "the pin number must reach export, got \"%s\"", buf);

    read_pin_file(23, "direction", buf, sizeof(buf));
    CHECK(strcmp(buf, "out") == 0, "an input must be turned round before the write, got \"%s\"",
          buf);

    read_pin_file(23, "value", buf, sizeof(buf));
    CHECK(buf[0] == '1', "the value must be written, got \"%s\"", buf);

    CHECK(hal_gpio_set(NULL, 23, 0) == RSS_OK, "clearing must succeed");
    read_pin_file(23, "value", buf, sizeof(buf));
    CHECK(buf[0] == '0', "the cleared value must be written, got \"%s\"", buf);
}

static void test_set_leaves_an_output_alone(void)
{
    char buf[32];

    /*
     * The sentinel is the test. If the direction is already "out" nothing
     * should touch the file, so anything past "out" survives; a rewrite
     * would truncate it. Writing "out" unconditionally pulses the line low
     * on most drivers, which on an IR-cut pin is a visible glitch on every
     * poll.
     */
    fake_sysfs(24, "out-sentinel");

    CHECK(hal_gpio_set(NULL, 24, 1) == RSS_OK, "driving an output must succeed");

    read_pin_file(24, "direction", buf, sizeof(buf));
    CHECK(strcmp(buf, "out-sentinel") == 0,
          "an output's direction must not be rewritten, got \"%s\"", buf);

    read_pin_file(24, "value", buf, sizeof(buf));
    CHECK(buf[0] == '1', "the value must still be written, got \"%s\"", buf);
}

static void test_get_reads_without_changing_direction(void)
{
    char buf[32];
    char path[256];
    int value = -1;

    /* An output being read back: switching it to input to sample it would
     * release whatever it is holding. */
    fake_sysfs(60, "out");
    snprintf(path, sizeof(path), GPIO_SYSFS_ROOT "/gpio60/value");
    write_file(path, "1\n");

    CHECK(hal_gpio_get(NULL, 60, &value) == RSS_OK, "reading a pin must succeed");
    CHECK(value == 1, "a high pin must read 1, got %d", value);

    read_pin_file(60, "direction", buf, sizeof(buf));
    CHECK(strcmp(buf, "out") == 0, "a read must not change the direction, got \"%s\"", buf);

    write_file(path, "0\n");
    CHECK(hal_gpio_get(NULL, 60, &value) == RSS_OK, "reading a low pin must succeed");
    CHECK(value == 0, "a low pin must read 0, got %d", value);
}

static void test_missing_pin_and_bad_arguments(void)
{
    int value = -1;

    /* Export accepted the write but no directory appeared: an invalid pin
     * on this SoC. Both directions must report it rather than pretend. */
    fake_sysfs(-1, NULL);
    CHECK(hal_gpio_set(NULL, 99, 1) == RSS_ERR_IO, "a pin with no sysfs entry must fail");
    CHECK(hal_gpio_get(NULL, 99, &value) == RSS_ERR_IO, "reading a missing pin must fail");

    CHECK(hal_gpio_set(NULL, -1, 1) == RSS_ERR_INVAL, "a negative pin must be rejected");
    CHECK(hal_gpio_get(NULL, -1, &value) == RSS_ERR_INVAL, "a negative pin must be rejected");
    CHECK(hal_gpio_get(NULL, 23, NULL) == RSS_ERR_INVAL, "a NULL out pointer must be rejected");
}

/*
 * A pin with no direction attribute at all -- some drivers export
 * fixed-function lines that way. The value write is then the only thing
 * that can succeed or fail, and it must still be attempted.
 */
static void test_pin_without_a_direction_attribute(void)
{
    char buf[32];

    fake_sysfs(59, NULL);

    CHECK(hal_gpio_set(NULL, 59, 1) == RSS_OK, "a directionless pin must still be written");
    read_pin_file(59, "value", buf, sizeof(buf));
    CHECK(buf[0] == '1', "the value must be written, got \"%s\"", buf);
}

int main(void)
{
    test_set_exports_and_drives();
    test_set_leaves_an_output_alone();
    test_get_reads_without_changing_direction();
    test_missing_pin_and_bad_arguments();
    test_pin_without_a_direction_attribute();

    if (system("rm -rf " GPIO_SYSFS_ROOT) != 0)
        return 1;

    if (failures) {
        printf("\n%d check(s) failed\n", failures);
        return 1;
    }

    printf("all hal_gpio logic tests passed\n");
    return 0;
}
