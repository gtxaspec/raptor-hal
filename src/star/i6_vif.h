/*
 * star/i6_vif.h -- MI_VIF bindings, Infinity6E
 *
 * Vendored from OpenIPC divinus, src/hal/star/i6_vif.h. See i6_common.h for
 * why these headers are vendored and for the four adaptations applied.
 *
 * VIF is the sensor-facing capture interface: a device carries the interface
 * type and sync polarity, its channel ports carry crop and destination
 * geometry. It has no Ingenic counterpart -- IMP folds this into
 * IMP_ISP_AddSensor -- which is the main reason the MI backend is a separate
 * translation unit rather than more #ifdefs in src/hal_common.c.
 *
 * Copyright (c) 2024 OpenIPC
 * SPDX-License-Identifier: MIT
 */

#ifndef STAR_I6_VIF_H
#define STAR_I6_VIF_H

#include "i6_common.h"

typedef enum {
    I6_VIF_FRATE_FULL,
    I6_VIF_FRATE_HALF,
    I6_VIF_FRATE_QUART,
    I6_VIF_FRATE_OCTANT,
    I6_VIF_FRATE_3QUARTS,
    I6_VIF_FRATE_END
} i6_vif_frate;

typedef enum {
    I6_VIF_WORK_1MULTIPLEX,
    I6_VIF_WORK_2MULTIPLEX,
    I6_VIF_WORK_4MULTIPLEX,
    I6_VIF_WORK_RGB_REALTIME,
    I6_VIF_WORK_RGB_FRAME,
    I6_VIF_WORK_END
} i6_vif_work;

typedef struct {
    i6_common_intf intf;
    i6_vif_work work;
    i6_common_hdr hdr;
    i6_common_edge edge;
    i6_common_input input;
    char bitswap;
    i6_common_sync sync;
    /*
     * The vendor's MI_VIF_DevAttr_t ends in a u32MultiDevMap, and it must be
     * declared here: MI_VIF_SetDevAttr block-copies 3 x 16 bytes plus one
     * trailing word -- 52, not 48 -- out of the pointer it is given, then
     * sends an ioctl whose hardcoded payload length is 56, i.e. 4 bytes of
     * device id followed by those 52. (Both numbers read off a disassembly of
     * libmi_vif.so.) So the field sits at exactly +48 and sizeof must be 52.
     *
     * Omitting it does not fail loudly. The vendor reads a word of the
     * caller's stack frame and passes it to the driver as a device bitmap,
     * deterministically for a given binary -- one stack layout leaves the 1
     * the driver wants, another leaves a fragment of the sensor-name string
     * (0x36346367, "gc46"), and VIF then never syncs: dmesg loops on
     * _MI_VIF_EnqueueOutputTaskDev "layout type 2, bindmode 4 not sync err"
     * and no stream is produced. Hence the asserts below rather than trust.
     */
    unsigned int multidevmap;
} i6_vif_dev;

_Static_assert(sizeof(i6_vif_dev) == 52, "i6_vif_dev must match the 52 bytes MI_VIF_SetDevAttr copies");
_Static_assert(offsetof(i6_vif_dev, multidevmap) == 48, "u32MultiDevMap sits at +48; see the comment above");

typedef struct {
    i6_common_rect capt;
    i6_common_dim dest;
    // Values 0-3 correspond to No, Top, Bottom, Both
    int field;
    int interlaceOn;
    i6_common_pixfmt pixFmt;
    i6_vif_frate frate;
    unsigned int frameLineCnt;
} i6_vif_port;

/* Checked the same way as i6_vif_dev above, since a short struct here would
 * fail identically: MI_VIF_SetChnPortAttr copies 2 x 16 bytes and its ioctl
 * payload is 40, i.e. 8 bytes of channel and port plus these 32. This one was
 * already the right size -- the assert is here so it stays that way. */
_Static_assert(sizeof(i6_vif_port) == 32, "i6_vif_port must match the 32 bytes MI_VIF_SetChnPortAttr copies");

typedef struct {
    void *handle;

    int (*fnDisableDevice)(int device);
    int (*fnEnableDevice)(int device);
    int (*fnSetDeviceConfig)(int device, i6_vif_dev *config);

    int (*fnDisablePort)(int channel, int port);
    int (*fnEnablePort)(int channel, int port);
    int (*fnSetPortConfig)(int channel, int port, i6_vif_port *config);
} i6_vif_impl;

static inline int i6_vif_load(i6_vif_impl *vif_lib)
{
    if (!(vif_lib->handle = dlopen("libmi_vif.so", RTLD_LAZY | RTLD_GLOBAL))) {
        HAL_LOG_ERR("i6_vif: failed to load library: %s", dlerror());
        return RSS_ERR_NOENT;
    }

    if (!(vif_lib->fnDisableDevice = (int(*)(int device))
        hal_symbol_load("i6_vif", vif_lib->handle, "MI_VIF_DisableDev")))
        return RSS_ERR_NOTSUP;

    if (!(vif_lib->fnEnableDevice = (int(*)(int device))
        hal_symbol_load("i6_vif", vif_lib->handle, "MI_VIF_EnableDev")))
        return RSS_ERR_NOTSUP;

    if (!(vif_lib->fnSetDeviceConfig = (int(*)(int device, i6_vif_dev *config))
        hal_symbol_load("i6_vif", vif_lib->handle, "MI_VIF_SetDevAttr")))
        return RSS_ERR_NOTSUP;

    if (!(vif_lib->fnDisablePort = (int(*)(int channel, int port))
        hal_symbol_load("i6_vif", vif_lib->handle, "MI_VIF_DisableChnPort")))
        return RSS_ERR_NOTSUP;

    if (!(vif_lib->fnEnablePort = (int(*)(int channel, int port))
        hal_symbol_load("i6_vif", vif_lib->handle, "MI_VIF_EnableChnPort")))
        return RSS_ERR_NOTSUP;

    if (!(vif_lib->fnSetPortConfig = (int(*)(int channel, int port, i6_vif_port *config))
        hal_symbol_load("i6_vif", vif_lib->handle, "MI_VIF_SetChnPortAttr")))
        return RSS_ERR_NOTSUP;

    return RSS_OK;
}

static inline void i6_vif_unload(i6_vif_impl *vif_lib)
{
    if (vif_lib->handle)
        dlclose(vif_lib->handle);
    vif_lib->handle = NULL;
    memset(vif_lib, 0, sizeof(*vif_lib));
}

#endif /* STAR_I6_VIF_H */
