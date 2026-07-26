/*
 * star/i6_isp.h -- MI_ISP bindings, Infinity6E
 *
 * Based on OpenIPC divinus, src/hal/star/i6_isp.h. See i6_common.h for why
 * these headers are vendored and for the four adaptations applied to all of
 * them. This one carries more than divinus's original: divinus binds six
 * MI_ISP entry points, and phase 3 needs the readiness probe and generic
 * IQ-command access as well (see below).
 *
 * ================================================================
 * TWO SHAPES OF MI_ISP CALL
 *
 * MI_ISP splits into a handful of *typed* lifecycle calls -- load an IQ
 * binary, enable CUS3A, read the AE exposure limits -- and roughly 340
 * MI_ISP_{IQ,AE,AWB,AF}_{Get,Set}<Module> calls that are all the same
 * shape: (int channel, void *payload), with a per-module payload whose
 * size the userspace wrapper hardcodes.
 *
 * The typed calls get real prototypes here. The per-module ones do not,
 * because writing 340 structs to poke one field each is not a sensible
 * trade -- hal_isp.c drives them from a descriptor table instead, and
 * only needs `handle` plus the i6_isp_cmd_fn signature below. The
 * layout convention that makes that possible is documented in hal_isp.c.
 * ================================================================
 *
 * Copyright (c) 2024 OpenIPC
 * SPDX-License-Identifier: MIT
 */

#ifndef STAR_I6_ISP_H
#define STAR_I6_ISP_H

#include "i6_common.h"

/*
 * AE exposure limits. MI_ISP_AE_{Get,Set}ExposureLimit, command 0x1409.
 *
 * The 32-byte payload size the wrapper hardcodes matches these eight
 * unsigned ints exactly (checked by disassembling the board's own
 * libmi_isp.so -- see the objdump technique in i6_sys.h). Note the field
 * order: the two gain minima precede the two maxima, which is not the
 * min/max pairing the shutter and aperture fields above them use.
 */
typedef struct {
    unsigned int minShutterUs;
    unsigned int maxShutterUs;
    unsigned int minApertX10;
    unsigned int maxApertX10;
    unsigned int minSensorGain;
    unsigned int minIspGain;
    unsigned int maxSensorGain;
    unsigned int maxIspGain;
} i6_isp_exp;

/*
 * CUS3A enable parameters. 13 ints, of which only the first three are
 * ever set by either reference: {1, 0, 0} then {1, 1, 0}, called in that
 * order. Neither project documents the fields, and the pair-of-calls
 * sequence is reproduced rather than explained -- see star_isp_enable_3a.
 */
typedef struct {
    int params[13];
} i6_isp_p3a;

/*
 * IQ parameter-init status. MI_ISP_IQ_GetParaInitStatus, command 0x1002,
 * payload 4 bytes -- a single flag, which is why the nested-struct form
 * the vendor headers use collapses to this.
 *
 * This is the one binding neither divinus nor an ISP tuning guide would
 * suggest is necessary, and it is the difference between a reliable IQ
 * load and an intermittent one. The ISP channel initialises
 * asynchronously *after* MI_VPE_CreateChannel returns, so a bin load
 * issued immediately gets "IspApiGet channel not created" from the
 * kernel driver. divinus papers over this with sleep(1) before its load
 * (media.c:827); polling this flag is the same wait without the guess.
 */
typedef struct {
    int ready;
} i6_isp_parainit;

/*
 * Generic per-module IQ/AE/AWB command. Every
 * MI_ISP_{IQ,AE,AWB}_{Get,Set}<Module> entry point has this signature;
 * the payload is module-specific and its size is fixed in the wrapper.
 */
typedef int (*i6_isp_cmd_fn)(int channel, void *payload);

typedef struct {
    void *handle, *handleCus3a, *handleIspAlgo;

    int (*fnDisableUserspace3A)(int channel);
    int (*fnEnableUserspace3A)(int channel, i6_isp_p3a *params);
    int (*fnLoadChannelConfig)(int channel, char *path, unsigned int key);
    int (*fnGetParaInitStatus)(int channel, i6_isp_parainit *status);
    int (*fnGetExposureLimit)(int channel, i6_isp_exp *config);
    int (*fnSetExposureLimit)(int channel, i6_isp_exp *config);
} i6_isp_impl;

static inline int i6_isp_load(i6_isp_impl *isp_lib)
{
    /*
     * Same chain, same order, same best-effort policy as i6_vpe_load --
     * see the long comment there for why DT_NEEDED cannot be relied on
     * and why RTLD_GLOBAL is required. VPE has almost certainly opened
     * all three already by the time the ISP comes up; dlopen refcounts,
     * so opening them again is correct rather than merely harmless, as
     * it keeps this module's teardown independent of VPE's.
     */
    isp_lib->handleIspAlgo = dlopen("libispalgo.so", RTLD_LAZY | RTLD_GLOBAL);
    if (!isp_lib->handleIspAlgo)
        HAL_LOG_WARN("i6_isp: libispalgo.so not loaded (%s) -- ISP algorithms may fail",
                     dlerror());

    isp_lib->handleCus3a = dlopen("libcus3a.so", RTLD_LAZY | RTLD_GLOBAL);
    if (!isp_lib->handleCus3a)
        HAL_LOG_WARN("i6_isp: libcus3a.so not loaded (%s) -- ISP algorithms may fail",
                     dlerror());

    if (!(isp_lib->handle = dlopen("libmi_isp.so", RTLD_LAZY | RTLD_GLOBAL))) {
        HAL_LOG_ERR("i6_isp: failed to load libmi_isp.so: %s", dlerror());
        return RSS_ERR_NOENT;
    }

    if (!(isp_lib->fnDisableUserspace3A =
              (int (*)(int channel))hal_symbol_load("i6_isp", isp_lib->handle,
                                                    "MI_ISP_DisableUserspace3A")))
        return RSS_ERR_NOTSUP;

    if (!(isp_lib->fnEnableUserspace3A =
              (int (*)(int channel, i6_isp_p3a *params))hal_symbol_load(
                  "i6_isp", isp_lib->handle, "MI_ISP_CUS3A_Enable")))
        return RSS_ERR_NOTSUP;

    if (!(isp_lib->fnLoadChannelConfig =
              (int (*)(int channel, char *path, unsigned int key))hal_symbol_load(
                  "i6_isp", isp_lib->handle, "MI_ISP_API_CmdLoadBinFile")))
        return RSS_ERR_NOTSUP;

    if (!(isp_lib->fnGetParaInitStatus =
              (int (*)(int channel, i6_isp_parainit *status))hal_symbol_load(
                  "i6_isp", isp_lib->handle, "MI_ISP_IQ_GetParaInitStatus")))
        return RSS_ERR_NOTSUP;

    if (!(isp_lib->fnGetExposureLimit =
              (int (*)(int channel, i6_isp_exp *config))hal_symbol_load(
                  "i6_isp", isp_lib->handle, "MI_ISP_AE_GetExposureLimit")))
        return RSS_ERR_NOTSUP;

    if (!(isp_lib->fnSetExposureLimit =
              (int (*)(int channel, i6_isp_exp *config))hal_symbol_load(
                  "i6_isp", isp_lib->handle, "MI_ISP_AE_SetExposureLimit")))
        return RSS_ERR_NOTSUP;

    return RSS_OK;
}

static inline void i6_isp_unload(i6_isp_impl *isp_lib)
{
    if (isp_lib->handle)
        dlclose(isp_lib->handle);
    if (isp_lib->handleCus3a)
        dlclose(isp_lib->handleCus3a);
    if (isp_lib->handleIspAlgo)
        dlclose(isp_lib->handleIspAlgo);
    memset(isp_lib, 0, sizeof(*isp_lib));
}

#endif /* STAR_I6_ISP_H */
