/*
 * hal_ivs_nna_pin.h -- keep the T41's ISP firmware thread off core 1
 * while the NNA runs
 *
 * On a T41 the NN instructions run on core 0 only. When core 0 runs them
 * while core 1 runs the ISP firmware's in-kernel MXU3 code (tisp_tmo_fpga,
 * in the isp_fw_process kernel thread), both cores can stall until the
 * watchdog resets the SoC. Pinning isp_fw_process to core 0 as well means
 * the two never run at the same time. The approach, and the finding, are
 * from the onna backend (gtxaspec/raptor-hal#25).
 *
 * nna_find_comm() is separate from the pin so the host tests can run it
 * on a fake proc tree.
 */

#ifndef HAL_IVS_NNA_PIN_H
#define HAL_IVS_NNA_PIN_H

#include <dirent.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>

/*
 * The pid of the first task under proc_root whose comm is comm exactly;
 * 0 if there is none, -1 if proc_root cannot be read.
 */
static inline pid_t nna_find_comm(const char *proc_root, const char *comm)
{
    DIR *d = opendir(proc_root);
    if (!d)
        return -1;

    pid_t pid = 0;
    struct dirent *e;
    while (!pid && (e = readdir(d)) != NULL) {
        if (e->d_name[0] < '1' || e->d_name[0] > '9')
            continue;
        char path[300], name[32] = "";
        if ((size_t)snprintf(path, sizeof(path), "%s/%s/comm", proc_root, e->d_name) >=
            sizeof(path))
            continue;
        FILE *f = fopen(path, "r");
        if (!f)
            continue;
        if (fgets(name, sizeof(name), f)) {
            name[strcspn(name, "\n")] = '\0';
            if (!strcmp(name, comm))
                pid = (pid_t)atoi(e->d_name);
        }
        fclose(f);
    }
    closedir(d);
    return pid;
}

/*
 * Pin isp_fw_process to core 0. Returns its pid, 0 if there is no such
 * thread (no ISP loaded, or an ISP driver that names it otherwise), -1 if
 * it is there but cannot be moved.
 */
static inline pid_t nna_pin_isp_core0(void)
{
    pid_t pid = nna_find_comm("/proc", "isp_fw_process");
    if (pid <= 0)
        return 0;

    cpu_set_t m;
    CPU_ZERO(&m);
    CPU_SET(0, &m);
    if (sched_setaffinity(pid, sizeof(m), &m) != 0)
        return -1;
    return pid;
}

#endif /* HAL_IVS_NNA_PIN_H */
