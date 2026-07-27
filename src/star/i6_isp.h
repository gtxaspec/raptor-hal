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
 * AE status. MI_ISP_CUS3A_GetAeStatus, command 0x2e05.
 *
 * The one MI call that answers "what did the AE converge on" -- shutter
 * in microseconds and the two gains -- which is what day/night detection
 * needs and what MI_ISP_AE_GetManualExpo (the manual *setting*) and
 * GetExposureLimit (the bounds) cannot give. Field order is waybeam's
 * (star6e_cus3a.c, "verified via hex dump" on Star6E); this file adds the
 * size.
 *
 * That size is the reason not to copy waybeam's struct as it stands. The
 * wrapper declares a 65-byte payload:
 *
 *   65d4: sub  sp, #32
 *   65e4: movs r3, #65        @ 0x41   -- payload size
 *   65e8: movw r3, #11781     @ 0x2e05 -- command
 *   65fe: blx  _MI_ISP_GetIspApiData
 *
 * and _MI_ISP_GetIspApiData copies all 65 bytes into the caller's buffer.
 * waybeam declares twelve u32s, 48 bytes, on the stack -- so the vendor
 * library writes 17 bytes past it on every call. The tail below exists to
 * hold that overrun, and the assert is what keeps it holding it. Anything
 * past ispGainHdrShort is unread, not unwritten.
 */
typedef struct {
    unsigned int reserved0[3];
    unsigned int avgBlkX;
    unsigned int avgBlkY;
    unsigned int reserved1;
    unsigned int shutterUs;
    unsigned int sensorGain;
    unsigned int ispGain;
    unsigned int shutterHdrShortUs;
    unsigned int sensorGainHdrShort;
    unsigned int ispGainHdrShort;
    unsigned char tail[20];
} i6_isp_ae_status;

_Static_assert(sizeof(i6_isp_ae_status) >= 65,
               "AE status must hold the 65 bytes the wrapper copies into it");
_Static_assert(offsetof(i6_isp_ae_status, shutterUs) == 24, "AE status shutter offset");
_Static_assert(offsetof(i6_isp_ae_status, sensorGain) == 28, "AE status sensor gain offset");

/*
 * AE exposure info. MI_ISP_AE_QueryExposureInfo, command 0x1402.
 *
 * The diagnostic counterpart to the AE status above: status says what the
 * AE converged on, this says whether it is done converging and what it was
 * aiming at. `reachBoundary` is the field worth having -- it distinguishes
 * "the AE chose this exposure" from "the AE wanted more and something
 * stopped it", which no combination of the other readings can.
 *
 * Unlike CUS3A_GetAeStatus, waybeam's layout for this one is *correct*:
 * the wrapper declares 572 bytes (`mov.w r3, #572 @ 0x23c` at
 * MI_ISP_AE_QueryExposureInfo+0x12, size field at [r7,#12]) and their
 * struct sums to exactly that. Checked by disassembling the board's own
 * libmi_isp.so with the technique in i6_sys.h.
 *
 * Both exposure values are carried because reading only one is a trap: if
 * the AE is running a long/short pair, the short one is not the exposure
 * that determines image brightness, and reporting it would understate the
 * shutter by whatever the HDR ratio is.
 */
typedef struct {
    unsigned int fNx10;
    unsigned int sensorGain;
    unsigned int ispGain;
    unsigned int us;
} i6_isp_ae_expo_value;

typedef struct {
    int stable;
    int reachBoundary;
    i6_isp_ae_expo_value expoLong;
    i6_isp_ae_expo_value expoShort;
    unsigned int histLumY;
    unsigned int histAvgY;
    unsigned int histHits[128];
    unsigned int lvX10;
    int bv;
    unsigned int sceneTarget;
} i6_isp_ae_expo_info;

_Static_assert(sizeof(i6_isp_ae_expo_info) == 572,
               "AE exposure info must match the 572 bytes the wrapper declares");

/*
 * Hardware AE average statistics. MI_ISP_AE_GetAeHwAvgStats, command
 * 0x2e01, payload 46088 bytes (0xb408 at that wrapper's `movw r3`).
 *
 * 46088 = 128 * 90 * 4 + 8, and the neighbouring
 * MI_ISP_AWB_GetAwbHwAvgStats declares 34568 = 128 * 90 * 3 + 8. Two
 * calls agreeing on a 128x90 grid at one byte per channel, plus the same
 * eight spare bytes, is what fixes the cell width at four bytes here --
 * so waybeam's `short r, g, b, y` (8 bytes a cell, a 92160-byte struct
 * against a 46088-byte payload) cannot be the layout, and its avgY log
 * line is averaging two cells per sample.
 *
 * What those eight spare bytes are, and whether they lead or trail, the
 * sizes cannot say. Hence the union: hal_isp.c tries both placements
 * against the grid dimensions MI_ISP_CUS3A_GetAeStatus reports and
 * accepts the one that matches, rather than picking one and averaging
 * whatever lands at that offset. Neither matching means no luma -- not a
 * plausible-looking number derived from the wrong bytes.
 *
 * Settled on an SSC30KQ + GC4653 board 2026-07-27: the probe reported
 * "AE grid 32x32, cells at offset 8", so the eight bytes LEAD and they
 * are the grid dimensions themselves. 128x90 is the payload's maximum,
 * not the live grid -- the buffer stays sized for the declared 46088
 * either way, and the grid actually averaged is whatever AE status
 * reports. The lane order is still only waybeam's word: that scene came
 * back r=46 g=46 b=44 y=46, which is consistent with r,g,b,y but too
 * neutral to distinguish the lanes from each other.
 */
#define I6_ISP_AE_BLK_X 128
#define I6_ISP_AE_BLK_Y 90
#define I6_ISP_AE_BLK_MAX (I6_ISP_AE_BLK_X * I6_ISP_AE_BLK_Y)
#define I6_ISP_AE_CELL_SZ 4

/* Byte lane within a cell. Order is waybeam's r, g, b, y. */
#define I6_ISP_AE_CELL_Y 3

typedef union {
    unsigned char raw[I6_ISP_AE_BLK_MAX * I6_ISP_AE_CELL_SZ + 8];
    struct {
        unsigned int blkX, blkY;
        unsigned char cell[I6_ISP_AE_BLK_MAX * I6_ISP_AE_CELL_SZ];
    } lead;
    struct {
        unsigned char cell[I6_ISP_AE_BLK_MAX * I6_ISP_AE_CELL_SZ];
        unsigned int blkX, blkY;
    } trail;
} i6_isp_ae_hw_stats;

_Static_assert(sizeof(i6_isp_ae_hw_stats) == 46088,
               "AE HW stats must match the 46088-byte payload the wrapper declares");

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

    /*
     * Optional -- may be NULL, unlike everything above. Both are only
     * wanted by isp_get_exposure, which is advisory (ric's day/night
     * detection); a library without them should still bring the ISP up
     * and stream, so the loader must not fail on their absence.
     */
    int (*fnGetAeStatus)(int channel, i6_isp_ae_status *status);
    int (*fnGetAeHwAvgStats)(int channel, i6_isp_ae_hw_stats *stats);
    int (*fnQueryExposureInfo)(int channel, i6_isp_ae_expo_info *info);
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

    /*
     * dlsym directly rather than hal_symbol_load: these two are optional
     * (see the impl struct), and hal_symbol_load logs at error level, so
     * routing an expected NULL through it would report a fault every boot
     * on a library that simply predates the symbol.
     */
    isp_lib->fnGetAeStatus = (int (*)(int channel, i6_isp_ae_status *status))dlsym(
        isp_lib->handle, "MI_ISP_CUS3A_GetAeStatus");
    isp_lib->fnGetAeHwAvgStats = (int (*)(int channel, i6_isp_ae_hw_stats *stats))dlsym(
        isp_lib->handle, "MI_ISP_AE_GetAeHwAvgStats");
    isp_lib->fnQueryExposureInfo = (int (*)(int channel, i6_isp_ae_expo_info *info))dlsym(
        isp_lib->handle, "MI_ISP_AE_QueryExposureInfo");
    if (!isp_lib->fnGetAeStatus)
        HAL_LOG_WARN("i6_isp: no MI_ISP_CUS3A_GetAeStatus -- "
                     "exposure readback unavailable, ric cannot detect day/night");

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
