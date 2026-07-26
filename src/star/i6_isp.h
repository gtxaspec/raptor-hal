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
 * CUS3A enable flags: three MI_BOOLs, one byte each.
 *
 * The byte width is the entire point of this comment. Both references
 * declare the block as `int params[13]` (waybeam star6e_pipeline.c:274;
 * divinus binds the symbol and never calls it), and written that way
 * {1, 1, 0} lays down 01 00 00 00 01 00 00 00 -- byte 1 is zero, so AWB
 * is asked for and never enabled. That left auto white balance off after
 * every tuning load and gave the picture a magenta cast identical to
 * having loaded no tuning file at all (board-diagnosed 2026-07-26).
 *
 * MI_ISP_CUS3A_Enable in the board's own libmi_isp.so settles it -- three
 * byte loads, and a declared payload of 3:
 *
 *   6530: ldrb r3, [r3, #0]   @ ae
 *   6536: ldrb r3, [r3, #1]   @ awb
 *   653c: ldrb r3, [r3, #2]   @ af
 *   ...   _MI_ISP_SetIspApiData({20, 3, 0x2e08, channel, 0}, block)
 *
 * (Field 2 of that descriptor is the payload size, confirmed against
 * MI_ISP_IQ_GetBrightness, which declares the 76 bytes hal_isp.c's table
 * already knows it has.)
 *
 * The driver prints the flags it received, so the log is the check: see
 * star_isp_enable_3a. The tail is padding -- nothing reads past byte 2.
 */
typedef struct {
    unsigned char ae;
    unsigned char awb;
    unsigned char af;
    unsigned char pad[17];
} i6_isp_p3a;

_Static_assert(offsetof(i6_isp_p3a, awb) == 1, "CUS3A awb must be the byte the wrapper's ldrb #1 reads");
_Static_assert(offsetof(i6_isp_p3a, af) == 2, "CUS3A af must be the byte the wrapper's ldrb #2 reads");
_Static_assert(sizeof(i6_isp_p3a) == 20, "CUS3A block stays as wide as the descriptor's 20 bytes");

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
    /*
     * Named for the symbol it actually binds. MI_ISP_CUS3A_Enable is not
     * the inverse of MI_ISP_DisableUserspace3A -- MI_ISP_EnableUserspace3A
     * is a separate entry point that spawns the SDK's own 3A_Proc thread,
     * and waybeam's 6E path deliberately does not call it on the normal
     * (internal-AE) path. The old name read like the pairing it is not,
     * which is an easy way to "fix" the asymmetry by calling the wrong
     * function.
     */
    int (*fnCus3aEnable)(int channel, i6_isp_p3a *params);
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

    if (!(isp_lib->fnCus3aEnable =
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
