/*
 * tools/star_probe.c -- throwaway ABI probe for the SigmaStar MI backend
 *
 * NOT shipped and NOT part of libraptor_hal_*. Build it with
 *   make PLATFORM=INFINITY6E CROSS_COMPILE=arm-openipc-linux-gnueabihf- star_probe
 * copy it to the board, insmod the sensor modules, and run it.
 *
 * Purpose: prove the vendored src/star/i6_*.h struct layouts against the real
 * vendor .so files before any backend code depends on them. A layout error
 * shows up here as a garbled version string or an absurd resolution -- cheap
 * to spot and cheap to fix -- rather than as memory corruption several tasks
 * into the port, which is the failure mode this port most needs to avoid.
 *
 * Deliberately stops short of streaming: it queries the sensor and never
 * calls MI_SNR_Enable or touches VIF/VPE/VENC, so it cannot leave hardware
 * running. MI_SNR_SetPlaneMode is the one state it writes, matching what
 * divinus does before querying (i6_hal.c:228).
 *
 * On SSC30KQ + GC4653 expect: one plane, 2560x1440, 10bpp precision,
 * Bayer GR, MIPI interface, 2 lanes, 5-30 fps.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "star/i6_snr.h"
#include "star/i6_sys.h"
#include "star/i6_venc.h"
#include "star/i6_vif.h"
#include "star/i6_vpe.h"

#include <stdarg.h>

/* Sanity bounds on counts MI reports back — see probe_sensor(). */
#define PROBE_MAX_RES 32
#define PROBE_MAX_PLANES 4

/*
 * The i6_*.h loaders log through rss_hal_log_fn, which the HAL archives
 * define in star/hal_common.c. Defining it here instead keeps the probe a
 * single translation unit that links against nothing but libdl -- if it
 * needed the archive, a broken archive would mean no probe.
 */
static void probe_log(int level, const char *file, int line, const char *fmt, ...)
{
    va_list ap;

    (void)level;
    (void)file;
    (void)line;

    fputs("  !! ", stdout);
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    fputc('\n', stdout);
}

rss_hal_log_func_t rss_hal_log_fn = probe_log;

static const char *bayer_name(i6_common_bayer b)
{
    static const char *names[] = {"RG", "GR", "BG", "GB", "R0", "G0",
                                  "B0", "G1", "G2", "I0", "G3", "I1"};

    return b < I6_BAYER_END ? names[b] : "?";
}

static const char *prec_name(i6_common_prec p)
{
    static const char *names[] = {"8bpp", "10bpp", "12bpp", "14bpp", "16bpp"};

    return p < I6_PREC_END ? names[p] : "?";
}

static const char *intf_name(i6_common_intf i)
{
    static const char *names[] = {"BT656", "DIGITAL_CAMERA", "BT1120_STANDARD",
                                  "BT1120_INTERLEAVED", "MIPI"};

    return i < I6_INTF_END ? names[i] : "?";
}

static const char *hdr_name(i6_common_hdr h)
{
    static const char *names[] = {"OFF", "VC", "DOL", "EMBED", "LI"};

    return h < I6_HDR_END ? names[h] : "?";
}

/*
 * Struct sizes are the fastest layout check available without hardware: if
 * these disagree with the vendor's own build, every field past the first
 * mismatch is wrong. Printed unconditionally so a bad run still records them.
 */
static void print_sizes(void)
{
    printf("struct sizes (compile-time):\n");
    printf("  i6_sys_bind      %zu\n", sizeof(i6_sys_bind));
    printf("  i6_sys_ver       %zu\n", sizeof(i6_sys_ver));
    printf("  i6_snr_pad       %zu\n", sizeof(i6_snr_pad));
    printf("  i6_snr_plane     %zu\n", sizeof(i6_snr_plane));
    printf("  i6_snr_res       %zu\n", sizeof(i6_snr_res));
    printf("  i6_vif_dev       %zu\n", sizeof(i6_vif_dev));
    printf("  i6_vif_port      %zu\n", sizeof(i6_vif_port));
    printf("  i6_vpe_chn       %zu\n", sizeof(i6_vpe_chn));
    printf("  i6e_vpe_chn      %zu\n", sizeof(i6e_vpe_chn));
    printf("  i6_vpe_para      %zu\n", sizeof(i6_vpe_para));
    printf("  i6e_vpe_para     %zu\n", sizeof(i6e_vpe_para));
    printf("  i6_vpe_port      %zu\n", sizeof(i6_vpe_port));
    printf("  i6_venc_chn      %zu\n", sizeof(i6_venc_chn));
    printf("  i6_venc_strm     %zu\n", sizeof(i6_venc_strm));
    printf("  i6_venc_pack     %zu\n", sizeof(i6_venc_pack));
    printf("\n");
}

static int probe_sensor(i6_snr_impl *snr, unsigned int index)
{
    i6_snr_pad pad;
    i6_snr_plane plane;
    i6_snr_res res;
    unsigned char cur = 0;
    unsigned int count = 0;
    unsigned int i;
    int ret;

    /* Single-plane mode, as divinus does before querying geometry. */
    ret = snr->fnSetPlaneMode(index, 0);
    if (ret)
        printf("MI_SNR_SetPlaneMode: %d (continuing)\n", ret);

    ret = snr->fnGetResolutionCount(index, &count);
    if (ret) {
        printf("MI_SNR_QueryResCount: %d -- is sensor_<name>_mipi.ko loaded?\n", ret);
        return ret;
    }
    printf("sensor %u: %u resolution(s)\n", index, count);

    /*
     * Clamp before looping. A struct-layout error is precisely the condition
     * this tool exists to detect, and it can make a count come back as
     * garbage -- so the loop bound must not be trusted, or the probe floods
     * the console instead of reporting the problem.
     */
    if (count > PROBE_MAX_RES) {
        printf("  !! count implausible, clamping to %u -- suspect a layout error\n",
               PROBE_MAX_RES);
        count = PROBE_MAX_RES;
    }

    for (i = 0; i < count; i++) {
        memset(&res, 0, sizeof(res));
        ret = snr->fnGetResolution(index, (unsigned char)i, &res);
        if (ret) {
            printf("  [%u] MI_SNR_GetRes: %d\n", i, ret);
            continue;
        }
        printf("  [%u] crop %ux%u+%u+%u  out %ux%u  fps %u-%u  \"%.32s\"\n", i,
               res.crop.width, res.crop.height, res.crop.x, res.crop.y, res.output.width,
               res.output.height, res.minFps, res.maxFps, res.desc);
    }

    memset(&res, 0, sizeof(res));
    ret = snr->fnCurrentResolution(index, &cur, &res);
    if (ret)
        printf("MI_SNR_GetCurRes: %d\n", ret);
    else
        printf("current: [%u] %ux%u fps %u-%u\n", cur, res.output.width, res.output.height,
               res.minFps, res.maxFps);

    memset(&pad, 0, sizeof(pad));
    ret = snr->fnGetPadInfo(index, &pad);
    if (ret) {
        printf("MI_SNR_GetPadInfo: %d\n", ret);
        return ret;
    }
    printf("pad: planes %u  intf %s(%d)  hdr %s(%d)  earlyInit %d\n", pad.planeCnt,
           intf_name(pad.intf), pad.intf, hdr_name(pad.hdr), pad.hdr, pad.earlyInit);
    if (pad.intf == I6_INTF_MIPI)
        printf("     mipi: lanes %u  rgbFmt %u  input %d  hsyncMode %u  hwHdr %d  virtChn %u\n",
               pad.intfAttr.mipi.laneCnt, pad.intfAttr.mipi.rgbFmtOn, pad.intfAttr.mipi.input,
               pad.intfAttr.mipi.hsyncMode, pad.intfAttr.mipi.hwHdr, pad.intfAttr.mipi.virtChn);

    if (pad.planeCnt > PROBE_MAX_PLANES) {
        printf("  !! planeCnt implausible, clamping to %u -- suspect a layout error\n",
               PROBE_MAX_PLANES);
        pad.planeCnt = PROBE_MAX_PLANES;
    }

    for (i = 0; i < pad.planeCnt; i++) {
        memset(&plane, 0, sizeof(plane));
        ret = snr->fnGetPlaneInfo(index, i, &plane);
        if (ret) {
            printf("  plane %u: MI_SNR_GetPlaneInfo: %d\n", i, ret);
            continue;
        }
        printf("  plane %u: id %u  \"%.32s\"  capt %ux%u+%u+%u\n", i, plane.planeId,
               plane.sensName, plane.capt.width, plane.capt.height, plane.capt.x, plane.capt.y);
        printf("           bayer %s(%d)  prec %s(%d)  pixFmt %d  hdrSrc %d\n",
               bayer_name(plane.bayer), plane.bayer, prec_name(plane.precision), plane.precision,
               plane.pixFmt, plane.hdrSrc);
        printf("           shutter %uus  sensGain %u/1024  compGain %u\n", plane.shutter,
               plane.sensGain, plane.compGain);
    }

    return 0;
}

int main(int argc, char **argv)
{
    i6_sys_impl sys;
    i6_snr_impl snr;
    i6_sys_ver ver;
    unsigned int index = 0;
    int ret;

    if (argc > 1)
        index = (unsigned int)strtoul(argv[1], NULL, 0);

    printf("star_probe -- %s, sensor index %u\n\n", HAL_PLATFORM_NAME, index);
    print_sizes();

    memset(&sys, 0, sizeof(sys));
    memset(&snr, 0, sizeof(snr));

    ret = i6_sys_load(&sys);
    if (ret) {
        printf("i6_sys_load: %d\n", ret);
        return 1;
    }
    if (!sys.handleCamOsWrapper)
        printf("note: libcam_os_wrapper.so not loaded -- MI symbols may not resolve\n");

    ret = i6_snr_load(&snr);
    if (ret) {
        printf("i6_snr_load: %d\n", ret);
        i6_sys_unload(&sys);
        return 1;
    }

    ret = sys.fnInit();
    if (ret) {
        printf("MI_SYS_Init: %d\n", ret);
        i6_snr_unload(&snr);
        i6_sys_unload(&sys);
        return 1;
    }

    memset(&ver, 0, sizeof(ver));
    ret = sys.fnGetVersion(&ver);
    if (ret)
        printf("MI_SYS_GetVersion: %d\n", ret);
    else
        printf("MI version: \"%.127s\"\n\n", (char *)ver.version);

    ret = probe_sensor(&snr, index);

    sys.fnExit();
    i6_snr_unload(&snr);
    i6_sys_unload(&sys);

    printf("\n%s\n", ret ? "FAILED" : "OK");
    return ret ? 1 : 0;
}
