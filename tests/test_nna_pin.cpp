/*
 * test_nna_pin.cpp -- host tests of the task lookup in
 * src/hal_ivs_nna_pin.h, on a fake proc tree and on the host's /proc.
 *
 *   make -C tests
 */

#include "hal_ivs_nna_pin.h"

#include <cstdio>
#include <cstdlib>
#include <string>
#include <sys/stat.h>
#include <unistd.h>

static int failures;

#define CHECK(cond)                                                                                \
    do {                                                                                           \
        if (!(cond)) {                                                                             \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                                 \
            failures++;                                                                            \
        }                                                                                          \
    } while (0)

static std::string root;

/* <root>/<dir>/comm holding text; no comm file when text is NULL. */
static void task(const char *dir, const char *text)
{
    std::string d = root + "/" + dir;
    mkdir(d.c_str(), 0755);
    if (!text)
        return;
    FILE *f = fopen((d + "/comm").c_str(), "w");
    if (!f) {
        perror(d.c_str());
        exit(1);
    }
    fputs(text, f);
    fclose(f);
}

int main(void)
{
    char tmpl[] = "/tmp/test_nna_pin.XXXXXX";
    if (!mkdtemp(tmpl)) {
        perror("mkdtemp");
        return 1;
    }
    root = tmpl;

    /* no proc tree at all */
    CHECK(nna_find_comm((root + "/none").c_str(), "isp_fw_process") == -1);

    /* empty tree: nothing found */
    CHECK(nna_find_comm(root.c_str(), "isp_fw_process") == 0);

    /* not tasks: names that do not start with 1-9, a task without comm */
    task("self", "isp_fw_process\n");
    task("0", "isp_fw_process\n");
    task("sys", "isp_fw_process\n");
    task("12", NULL);
    CHECK(nna_find_comm(root.c_str(), "isp_fw_process") == 0);

    /* a longer or shorter name is not a match */
    task("40", "isp_fw_process2\n");
    task("41", "isp_fw\n");
    task("42", "kworker/0:1\n");
    CHECK(nna_find_comm(root.c_str(), "isp_fw_process") == 0);

    /* the task, with the kernel's trailing newline */
    task("377", "isp_fw_process\n");
    CHECK(nna_find_comm(root.c_str(), "isp_fw_process") == 377);
    CHECK(nna_find_comm(root.c_str(), "kworker/0:1") == 42);

    /* without the newline too */
    task("1024", "rvd");
    CHECK(nna_find_comm(root.c_str(), "rvd") == 1024);

    /* the host's own /proc finds this process by its name */
    char self[32] = "";
    FILE *f = fopen("/proc/self/comm", "r");
    if (f) {
        if (fgets(self, sizeof(self), f))
            self[strcspn(self, "\n")] = '\0';
        fclose(f);
        CHECK(nna_find_comm("/proc", self) > 0);
    }

    if (system(("rm -rf " + root).c_str()) != 0)
        printf("warning: %s left behind\n", root.c_str());

    if (failures) {
        printf("test_nna_pin: %d failure(s)\n", failures);
        return 1;
    }
    printf("test_nna_pin: ok\n");
    return 0;
}
