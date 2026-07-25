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
 * Never touches VENC -- that is 2d. By default it only queries, writing no
 * state but MI_SNR_SetPlaneMode, as divinus does before querying
 * (i6_hal.c:228).
 *
 * With -e it additionally runs SetRes/SetFps/SetOrien/Enable first, because
 * the pad's intfAttr and the plane geometry stay zero until the sensor
 * driver's pCus_sensor_init has run, and MI_SNR_Enable is what triggers it --
 * which is why waybeam queries the pad after Enable (sensor_select.c:485).
 * Enable is always paired with Disable, so even -e leaves nothing streaming.
 *
 * With -f it brings VIF up on top of the sensor and reports what MI made of
 * the configuration, via /proc/mi_modules -- which costs nothing and cannot be
 * wrong about ABI. It then tries to drain the VIF output port, which is the
 * only thing here that exercises i6_sys_bufinfo.
 *
 * Do not expect frames from that drain, and do not read zero as a failure.
 * Both references run VIF in RGB_REALTIME and bind it to VPE with
 * I6_SYS_LINK_REALTIME (divinus i6_hal.c:269,355), which is a hardware
 * streaming link: VIF hands pixels to the ISP without writing them to DRAM, so
 * with no VPE bound there is nothing for the CPU to read and nothing to count.
 * The first -f run confirmed exactly that -- every VIF call returned 0, the
 * proc table showed the port programmed with our geometry, and GetTotalCnt
 * stayed 0. Frame flow becomes observable in 2c, once VPE exists.
 *
 * So what -f actually verifies is that MI accepts the descriptors 2b derives
 * from the sensor. -f implies -e.
 *
 * With -v it builds the 2c stage on top: the VPE channel, the VIF->VPE
 * realtime bind, a scan of which output ports MI accepts, and an NV12 dump
 * from port 0 (sensor resolution) and port 1 (640x360) into /tmp. This is
 * where frames finally become CPU-readable -- VPE output ports are framebase,
 * unlike the realtime link feeding them -- so -v is the run that either
 * produces a viewable image or does not. -v implies -f.
 *
 * On SSC30KQ + GC4653 expect: 2560x1440, 10bpp precision, Bayer GR, MIPI
 * interface, 2 lanes, 5-30 fps, and a single plane (pad.planeCnt reads 0 --
 * see probe_sensor).
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/* nanosleep() -- the HAL builds -std=c11, which hides the POSIX prototypes. */
#define _POSIX_C_SOURCE 200809L

#include "star/i6_snr.h"
#include "star/i6_sys.h"
#include "star/i6_venc.h"
#include "star/i6_vif.h"
#include "star/i6_vpe.h"

#include <fcntl.h>
#include <stdarg.h>
#include <sys/select.h>
#include <time.h>
#include <unistd.h>

/* Sanity bounds on counts MI reports back — see probe_sensor(). */
#define PROBE_MAX_RES 32
#define PROBE_MAX_PLANES 4

/* VIF and VPE topology, matching src/star/star_state.h's STAR_* constants. */
#define PROBE_VIF_DEV 0
#define PROBE_VIF_CHN 0
#define PROBE_VIF_PORT 0
#define PROBE_VPE_DEV 0
#define PROBE_VPE_CHN 0

/* Where -v writes the NV12 dumps. Board-side path, so /tmp. */
#define PROBE_DUMP_DIR "/tmp"

/* How long to wait for a frame. At the GC4653's 5 fps floor a frame is 200 ms,
 * so a second is generous without hanging a failed run for long. */
#define PROBE_FRAME_TIMEOUT_MS 1000

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

static int probe_sensor(i6_snr_impl *snr, unsigned int index, int do_enable)
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

    /*
     * Optional bring-up. Until the sensor driver's pCus_sensor_init has run,
     * the pad descriptor is only partly populated -- planeCnt and intfAttr
     * read back as zero. That init is triggered by MI_SNR_Enable, which is
     * why waybeam queries the pad *after* Enable (sensor_select.c:485) while
     * divinus queries before it and relies only on the fields that are
     * populated either way (i6_hal.c:257).
     *
     * Enable starts the sensor, so it is behind a flag rather than the
     * default, and it is always paired with Disable below.
     */
    if (do_enable) {
        printf("\nbring-up: SetRes(%u) -> SetFps(%u) -> SetOrien(0,0) -> Enable\n", cur,
               res.maxFps ? res.maxFps : 30);
        ret = snr->fnSetResolution(index, cur);
        if (ret)
            printf("MI_SNR_SetRes: %d (continuing)\n", ret);
        ret = snr->fnSetFramerate(index, res.maxFps ? res.maxFps : 30);
        if (ret)
            printf("MI_SNR_SetFps: %d (continuing)\n", ret);
        ret = snr->fnSetOrientation(index, 0, 0);
        if (ret)
            printf("MI_SNR_SetOrien: %d (continuing)\n", ret);
        ret = snr->fnEnable(index);
        if (ret) {
            printf("MI_SNR_Enable: %d\n", ret);
            return ret;
        }
        printf("enabled\n\n");
    }

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
    if (!pad.planeCnt)
        printf("     (planeCnt 0 is normal before bring-up; neither reference reads this\n"
               "      field -- both query plane 0 directly, as below)\n");

    /*
     * Plane 0 unconditionally, not a loop over planeCnt. Both references do
     * exactly this -- divinus i6_hal.c:259 and waybeam sensor_select.c:491
     * both hardcode index 0 and neither reads planeCnt anywhere -- and
     * looping over it instead means an unpopulated pad silently skips the
     * most important struct in this header set: i6_snr_plane is what feeds
     * VIF port geometry and pixel format in 2b.
     *
     * Extra planes only exist for hardware HDR, which GC4653 has disabled.
     * They are reported when claimed, but never gate plane 0.
     */
    ret = snr->fnGetPlaneInfo(index, 0, &plane);
    if (ret) {
        printf("plane 0: MI_SNR_GetPlaneInfo: %d\n", ret);
        printf("  (without this the i6_snr_plane layout is untested -- rerun with -e)\n");
        return ret;
    }
    printf("plane 0: id %u  \"%.32s\"  capt %ux%u+%u+%u\n", plane.planeId, plane.sensName,
           plane.capt.width, plane.capt.height, plane.capt.x, plane.capt.y);
    printf("         bayer %s(%d)  prec %s(%d)  pixFmt %d  hdrSrc %d\n", bayer_name(plane.bayer),
           plane.bayer, prec_name(plane.precision), plane.precision, plane.pixFmt, plane.hdrSrc);
    printf("         shutter %uus  sensGain %u/1024  compGain %u\n", plane.shutter,
           plane.sensGain, plane.compGain);

    if (pad.planeCnt > PROBE_MAX_PLANES) {
        printf("  !! planeCnt implausible, clamping to %u -- suspect a layout error\n",
               PROBE_MAX_PLANES);
        pad.planeCnt = PROBE_MAX_PLANES;
    }

    for (i = 1; i < pad.planeCnt; i++) {
        memset(&plane, 0, sizeof(plane));
        ret = snr->fnGetPlaneInfo(index, i, &plane);
        if (ret) {
            printf("plane %u: MI_SNR_GetPlaneInfo: %d\n", i, ret);
            continue;
        }
        printf("plane %u: id %u  \"%.32s\"  capt %ux%u  bayer %s  prec %s\n", i, plane.planeId,
               plane.sensName, plane.capt.width, plane.capt.height, bayer_name(plane.bayer),
               prec_name(plane.precision));
    }

    return 0;
}

/*
 * Print a line from a /proc/mi_modules file and the `after` lines that follow
 * it -- the vendor's proc tables put column headers on one line and values on
 * the next, which is why the SigmaStar debugging notes always reach for
 * `grep -A1` (waybeam documentation/PROC_MI_MODULES_REFERENCE.md).
 */
static int probe_proc_grep(const char *path, const char *needle, int after)
{
    char line[512];
    int remaining = 0;
    int hits = 0;
    FILE *f = fopen(path, "r");

    if (!f) {
        printf("  %s: unavailable\n", path);
        return 0;
    }

    while (fgets(line, sizeof(line), f)) {
        if (strstr(line, needle)) {
            remaining = after;
            hits++;
            printf("  | %s", line);
        } else if (remaining > 0) {
            remaining--;
            printf("  | %s", line);
        }
    }

    fclose(f);
    if (!hits)
        printf("  %s: no line matching \"%s\"\n", path, needle);
    return hits;
}

/*
 * VIF pixel format. Duplicated from star_vif_pixfmt() in
 * src/star/hal_common.c, which is the source of truth -- the probe is a single
 * translation unit on purpose (see probe_log) so it cannot call into the
 * archive.
 *
 * The derived value wins for bayer sensors, as in both references. The first
 * -f run settled why: with the driver's reported 41 programmed, MI's proc
 * table decoded it back as "I0_10BPP" -- I0 being index 9, exactly what 41
 * means under the formula -- so the driver's field selects an IR pattern that
 * contradicts its own "bayer GR" report. See star_vif_pixfmt() in
 * src/star/hal_common.c for the full reasoning.
 */
static i6_common_pixfmt probe_vif_pixfmt(const i6_snr_plane *plane)
{
    if (plane->bayer >= I6_BAYER_END)
        return plane->pixFmt;

    return (i6_common_pixfmt)(I6_PIXFMT_RGB_BAYER + plane->precision * I6_BAYER_END + plane->bayer);
}

/*
 * probe_vpe_ports -- which VPE output ports does this silicon accept?
 *
 * hal_caps.c advertises max_fs_channels = 4 on the strength of divinus
 * disabling ports 0..3 in teardown, which is documentation by use rather than
 * a fact about the hardware. MI_VPE_SetPortMode either takes a port or it does
 * not, so ask, and let the caps value quote a measurement instead.
 *
 * Ports are left configured; the caller reconfigures the two it dumps from.
 */
static void probe_vpe_ports(i6_vpe_impl *vpe)
{
    i6_vpe_port attr;
    int i;

    printf("\nVPE output ports (MI_VPE_SetPortMode at 640x360 NV12; 0 == accepted):\n ");
    for (i = 0; i < 4; i++) {
        memset(&attr, 0, sizeof(attr));
        attr.output.width = 640;
        attr.output.height = 360;
        attr.pixFmt = I6_PIXFMT_YUV420SP;
        attr.compress = I6_COMPR_NONE;
        printf("  port %d: %d", i, vpe->fnSetPortConfig(PROBE_VPE_CHN, i, &attr));
    }
    printf("\n");
}

/*
 * probe_vpe_dump -- configure one VPE port, drain frames, write NV12 to a file.
 *
 * This is 2c's verification, and the first point in the port where a frame is
 * CPU-readable at all: VIF->VPE is a realtime hardware link with no DRAM round
 * trip, but the VPE *output* ports are framebase.
 *
 * The write follows the vendor's own MI_SYS_ChnOutputPortGetBuf sample
 * (ref/sigmastar-docs, MI_SYS reference 2.4.7), which writes
 *   height * stride[0] + height * stride[1] / 2
 * bytes from pVirAddr[0] -- i.e. it assumes the two planes are contiguous.
 * That assumption is worth checking rather than inheriting, so the plane
 * pointers are printed and compared.
 */
static int probe_vpe_dump(i6_sys_impl *sys, i6_vpe_impl *vpe, int port_id, unsigned short width,
                          unsigned short height, int frames, const char *dir)
{
    i6_vpe_port attr;
    i6_sys_bind bind;
    char path[256];
    int fd = -1;
    int fout = -1;
    int captured = 0;
    int ret;

    memset(&attr, 0, sizeof(attr));
    attr.output.width = width;
    attr.output.height = height;
    attr.pixFmt = I6_PIXFMT_YUV420SP;
    attr.compress = I6_COMPR_NONE;

    ret = vpe->fnSetPortConfig(PROBE_VPE_CHN, port_id, &attr);
    if (ret) {
        printf("\nMI_VPE_SetPortMode(%d, %d) %ux%u: %d\n", PROBE_VPE_CHN, port_id, width, height,
               ret);
        return ret;
    }

    ret = vpe->fnEnablePort(PROBE_VPE_CHN, port_id);
    if (ret) {
        printf("\nMI_VPE_EnablePort(%d, %d): %d\n", PROBE_VPE_CHN, port_id, ret);
        return ret;
    }

    printf("\nVPE port %d: %ux%u NV12\n", port_id, width, height);

    memset(&bind, 0, sizeof(bind));
    bind.module = I6_SYS_MOD_VPE;
    bind.device = PROBE_VPE_DEV;
    bind.channel = PROBE_VPE_CHN;
    bind.port = port_id;

    /* (2, 3) is the vendor sample's own pairing -- user depth 2, queue 3. */
    ret = sys->fnSetOutputDepth(&bind, 2, 3);
    if (ret) {
        printf("  MI_SYS_SetChnOutputPortDepth: %d\n", ret);
        goto out;
    }

    if (sys->fnGetFd) {
        ret = sys->fnGetFd(&bind, &fd);
        if (ret) {
            printf("  MI_SYS_GetFd: %d (falling back to polling)\n", ret);
            fd = -1;
        }
    }

    snprintf(path, sizeof(path), "%s/vpe_port%d_%ux%u.nv12", dir, port_id, width, height);

    while (captured < frames) {
        i6_sys_bufinfo buf;
        int handle = -1;
        unsigned int size;

        if (fd >= 0) {
            fd_set rfds;
            struct timeval tv = {PROBE_FRAME_TIMEOUT_MS / 1000,
                                 (PROBE_FRAME_TIMEOUT_MS % 1000) * 1000};

            FD_ZERO(&rfds);
            FD_SET(fd, &rfds);
            ret = select(fd + 1, &rfds, NULL, NULL, &tv);
            if (ret < 0) {
                printf("  select: %s\n", strerror(errno));
                break;
            }
            if (!ret) {
                printf("  timeout after %d ms with no frame\n", PROBE_FRAME_TIMEOUT_MS);
                break;
            }
        }

        memset(&buf, 0, sizeof(buf));
        ret = sys->fnGetOutputBuf(&bind, &buf, &handle);
        if (ret) {
            printf("  MI_SYS_ChnOutputPortGetBuf: %d\n", ret);
            break;
        }

        size = buf.frame.height * buf.frame.stride[0] + buf.frame.height * buf.frame.stride[1] / 2;
        if (buf.frame.bufSize && size > buf.frame.bufSize)
            size = buf.frame.bufSize;

        printf("  [%d] pts %llu  type %d  seq %u  %ux%u  pixFmt %d  stride %u/%u  "
               "bufSize %u  write %u\n",
               captured, buf.pts, buf.bufType, buf.seqNum, buf.frame.width, buf.frame.height,
               buf.frame.pixFmt, buf.frame.stride[0], buf.frame.stride[1], buf.frame.bufSize, size);
        printf("       va %p/%p  pa 0x%llx  planes %s\n", buf.frame.virAddr[0],
               buf.frame.virAddr[1], buf.frame.phyAddr[0],
               (char *)buf.frame.virAddr[1] ==
                       (char *)buf.frame.virAddr[0] + buf.frame.height * buf.frame.stride[0]
                   ? "contiguous"
                   : "NOT contiguous -- the vendor sample's single write is wrong here");

        /* One file per port, first frame only: the point is to look at an
         * image, and later frames would just append. */
        if (fout < 0 && buf.frame.virAddr[0] && size) {
            fout = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
            if (fout < 0)
                printf("       open(%s): %s\n", path, strerror(errno));
            else if (write(fout, buf.frame.virAddr[0], size) != (ssize_t)size)
                printf("       write(%s): %s\n", path, strerror(errno));
            else
                printf("       wrote %s (%u bytes) -- ffplay -f rawvideo -pix_fmt nv12 "
                       "-video_size %ux%u %s\n",
                       path, size, buf.frame.width, buf.frame.height, path);
        }

        sys->fnPutOutputBuf(handle);
        captured++;
    }

    printf("  captured %d of %d frame(s)\n", captured, frames);
    ret = captured ? 0 : -1;

out:
    if (fout >= 0)
        close(fout);
    if (fd >= 0 && sys->fnCloseFd)
        sys->fnCloseFd(fd);
    vpe->fnDisablePort(PROBE_VPE_CHN, port_id);

    return ret;
}

/*
 * probe_vpe -- create the VPE channel, bind VIF to it, dump both ports.
 *
 * Mirrors star_vpe_bringup() in src/star/hal_common.c plus what
 * hal_framesource.c does per port; if they ever disagree, those are right and
 * this is stale.
 */
static int probe_vpe(i6_sys_impl *sys, i6_vpe_impl *vpe, const i6_snr_plane *plane,
                     unsigned int index, unsigned int fps, int frames, const char *dir)
{
    i6e_vpe_chn channel;
    i6e_vpe_para param;
    i6_sys_bind source, dest;
    int created = 0, started = 0, bound = 0;
    int ret, dret;

    memset(&channel, 0, sizeof(channel));
    channel.capt.width = plane->capt.width;
    channel.capt.height = plane->capt.height;
    channel.pixFmt = probe_vif_pixfmt(plane);
    channel.hdr = I6_HDR_OFF;
    /* i6_vpe_sens is 1-based: ID0 == 1. Both references pass index + 1. */
    channel.sensor = (i6_vpe_sens)(index + 1);
    channel.mode = I6_VPE_MODE_REALTIME;

    printf("\nVPE: chn %d, %ux%u in, pixFmt %d, sensor id %d, mode REALTIME(%d)\n", PROBE_VPE_CHN,
           channel.capt.width, channel.capt.height, channel.pixFmt, channel.sensor, channel.mode);

    /* i6e_ struct cast to the shorter declared type -- Infinity6E reads the
     * longer layout. See i6_vpe.h. */
    ret = vpe->fnCreateChannel(PROBE_VPE_CHN, (i6_vpe_chn *)&channel);
    if (ret) {
        printf("MI_VPE_CreateChannel: %d\n", ret);
        return ret;
    }
    created = 1;

    memset(&param, 0, sizeof(param));
    param.hdr = I6_HDR_OFF;
    param.level3DNR = 1;
    ret = vpe->fnSetChannelParam(PROBE_VPE_CHN, (i6_vpe_para *)&param);
    if (ret) {
        printf("MI_VPE_SetChannelParam: %d\n", ret);
        goto out;
    }

    ret = vpe->fnStartChannel(PROBE_VPE_CHN);
    if (ret) {
        printf("MI_VPE_StartChannel: %d\n", ret);
        goto out;
    }
    started = 1;

    memset(&source, 0, sizeof(source));
    source.module = I6_SYS_MOD_VIF;
    source.device = PROBE_VIF_DEV;
    source.channel = PROBE_VIF_CHN;
    source.port = PROBE_VIF_PORT;

    memset(&dest, 0, sizeof(dest));
    dest.module = I6_SYS_MOD_VPE;
    dest.device = PROBE_VPE_DEV;
    dest.channel = PROBE_VPE_CHN;
    dest.port = 0;

    ret = sys->fnBindExt(&source, &dest, fps, fps, I6_SYS_LINK_REALTIME, 0);
    if (ret) {
        printf("MI_SYS_BindChnPort2 VIF->VPE (realtime, %u fps): %d\n", fps, ret);
        goto out;
    }
    bound = 1;
    printf("     bound VIF -> VPE, realtime, %u fps\n", fps);

    probe_vpe_ports(vpe);

    /*
     * Let the ISP finish coming up. MI_VPE_CreateChannel returns before the
     * ISP channel has initialised (waybeam polls MI_ISP_IQ_GetParaInitStatus
     * for up to 2 s here), and the first frames off a cold ISP are not worth
     * looking at anyway.
     */
    {
        struct timespec nap = {1, 0};

        nanosleep(&nap, NULL);
    }

    /* Port 0 at sensor resolution, port 1 scaled -- the main/sub split the
     * backend will use, and the substream is where this board's known
     * trouble lives. */
    ret = probe_vpe_dump(sys, vpe, 0, plane->capt.width, plane->capt.height, frames, dir);
    dret = probe_vpe_dump(sys, vpe, 1, 640, 360, frames, dir);
    if (!ret)
        ret = dret;

out:
    if (bound)
        sys->fnUnbind(&source, &dest);
    if (started)
        vpe->fnStopChannel(PROBE_VPE_CHN);
    if (created)
        vpe->fnDestroyChannel(PROBE_VPE_CHN);

    return ret;
}

/*
 * Bring VIF up on an already-enabled sensor and confirm frames arrive. Mirrors
 * star_vif_bringup() in src/star/hal_common.c; if the two ever disagree,
 * hal_common.c is right and this is stale.
 *
 * With vpe non-NULL it goes on to build the VPE stage on top before tearing
 * anything down, since VPE only means anything with VIF live underneath it.
 */
static int probe_vif(i6_sys_impl *sys, i6_snr_impl *snr, i6_vif_impl *vif, i6_vpe_impl *vpe,
                     unsigned int index, int frames, const char *dir)
{
    i6_snr_pad pad;
    i6_snr_plane plane;
    i6_vif_dev device;
    i6_vif_port port;
    i6_sys_bind bind;
    int dev_enabled = 0;
    int port_enabled = 0;
    int fd = -1;
    int captured = 0;
    int ret;

    memset(&pad, 0, sizeof(pad));
    memset(&plane, 0, sizeof(plane));

    /* Both descriptors are read after MI_SNR_Enable, since the sensor driver's
     * init is what populates them -- see probe_sensor's comment. */
    ret = snr->fnGetPadInfo(index, &pad);
    if (ret) {
        printf("MI_SNR_GetPadInfo: %d\n", ret);
        return ret;
    }
    ret = snr->fnGetPlaneInfo(index, 0, &plane);
    if (ret) {
        printf("MI_SNR_GetPlaneInfo: %d\n", ret);
        return ret;
    }

    /*
     * Orientation timing check. probe_sensor calls MI_SNR_SetOrien(0, 0)
     * before Enable, exactly as divinus does (i6_hal.c:254 precedes its
     * fnEnable), yet the first -f run showed the sensor reporting bmirror 1
     * bflip 1 afterwards. The likely explanation is the same one that made us
     * move the descriptor reads after Enable: pCus_sensor_init runs on Enable
     * and applies the driver's own defaults over anything set earlier.
     *
     * Mirror and flip matter beyond framing -- they change the effective bayer
     * order, which is what the pixel format above encodes -- so re-apply after
     * Enable and print the sensor's own view either side of it. Whichever way
     * this reads, the answer belongs in the record before 2c programs a VPE
     * channel from the same descriptors.
     */
    printf("\norientation, as the sensor reports it after Enable:\n");
    probe_proc_grep("/proc/mi_modules/mi_sensor/mi_sensor0", "PadId", 1);
    ret = snr->fnSetOrientation(index, 0, 0);
    printf("re-applied MI_SNR_SetOrien(%u, 0, 0) after Enable: %d\n", index, ret);
    probe_proc_grep("/proc/mi_modules/mi_sensor/mi_sensor0", "PadId", 1);

    memset(&device, 0, sizeof(device));
    device.intf = pad.intf;
    device.work = device.intf == I6_INTF_BT656 ? I6_VIF_WORK_1MULTIPLEX : I6_VIF_WORK_RGB_REALTIME;
    device.hdr = I6_HDR_OFF;
    if (device.intf == I6_INTF_MIPI) {
        device.edge = I6_EDGE_DOUBLE;
        device.input = pad.intfAttr.mipi.input;
    } else if (device.intf == I6_INTF_BT656) {
        device.edge = pad.intfAttr.bt656.edge;
        device.sync = pad.intfAttr.bt656.sync;
        device.bitswap = (char)pad.intfAttr.bt656.bitswap;
    }

    printf("\nVIF: intf %s(%d) work %d edge %d input %d\n", intf_name(device.intf), device.intf,
           device.work, device.edge, device.input);

    ret = vif->fnSetDeviceConfig(PROBE_VIF_DEV, &device);
    if (ret) {
        printf("MI_VIF_SetDevAttr: %d\n", ret);
        return ret;
    }
    ret = vif->fnEnableDevice(PROBE_VIF_DEV);
    if (ret) {
        printf("MI_VIF_EnableDev: %d\n", ret);
        return ret;
    }
    dev_enabled = 1;

    memset(&port, 0, sizeof(port));
    port.capt = plane.capt;
    port.dest.width = plane.capt.width;
    port.dest.height = plane.capt.height;
    port.pixFmt = probe_vif_pixfmt(&plane);
    port.frate = I6_VIF_FRATE_FULL;

    printf("     port %ux%u+%u+%u -> %ux%u  pixFmt %d (sensor reported %d)\n", port.capt.width,
           port.capt.height, port.capt.x, port.capt.y, port.dest.width, port.dest.height,
           port.pixFmt, plane.pixFmt);

    ret = vif->fnSetPortConfig(PROBE_VIF_CHN, PROBE_VIF_PORT, &port);
    if (ret) {
        printf("MI_VIF_SetChnPortAttr: %d (pixFmt %d)\n", ret, port.pixFmt);
        goto out;
    }
    ret = vif->fnEnablePort(PROBE_VIF_CHN, PROBE_VIF_PORT);
    if (ret) {
        printf("MI_VIF_EnableChnPort: %d\n", ret);
        goto out;
    }
    port_enabled = 1;
    printf("     up\n");

    /*
     * Give the sensor and VIF a moment to produce frames before reading the
     * counters -- at 30 fps a few hundred ms is many frames, and reading
     * immediately would report zero for reasons that have nothing to do with
     * whether the bring-up worked.
     */
    {
        struct timespec nap = {0, 500 * 1000 * 1000};

        nanosleep(&nap, NULL);
    }

    printf("\nVIF as MI sees it (/proc/mi_modules -- no ABI involved). The Cap_size,\n"
           "Dest_size and Fmt columns are the descriptors above read back; the counters\n"
           "stay 0 until VPE is bound, since RGB_REALTIME never lands frames in DRAM:\n");
    probe_proc_grep("/proc/mi_modules/mi_vif/mi_vif0", "GetTotalCnt", 1);
    probe_proc_grep("/proc/mi_modules/mi_vif/mi_vif0", "OutCount", 1);

    /*
     * With -v, VPE is the observation point and the VIF drain below would only
     * spend a second confirming what the first two runs already established.
     * Hand over with VIF still up.
     */
    if (vpe) {
        i6_snr_res res;
        unsigned int fps = 30;

        memset(&res, 0, sizeof(res));
        if (!snr->fnGetResolution(index, 0, &res) && res.maxFps)
            fps = res.maxFps;

        ret = probe_vpe(sys, vpe, &plane, index, fps, frames, dir);
        goto out;
    }

    /*
     * The ABI-exercising part, kept because it costs one second and is the only
     * thing that would exercise i6_sys_bufinfo. It is expected to time out
     * here for the realtime-link reason in the file header; 2c is where this
     * check becomes meaningful, against a VPE port.
     */
    if (!frames)
        goto out;

    if (!sys->fnGetOutputBuf || !sys->fnPutOutputBuf) {
        printf("\nMI_SYS_ChnOutputPortGetBuf/PutBuf not exported -- skipping frame reads\n");
        goto out;
    }

    memset(&bind, 0, sizeof(bind));
    bind.module = I6_SYS_MOD_VIF;
    bind.device = PROBE_VIF_DEV;
    bind.channel = PROBE_VIF_CHN;
    bind.port = PROBE_VIF_PORT;

    /* usrDepth 2 / bufDepth 4 is divinus's own choice for a port userspace
     * drains (i6_hal.c:109). Without a user depth the port queues nothing for
     * the CPU and GetBuf always returns empty. */
    ret = sys->fnSetOutputDepth(&bind, 2, 4);
    if (ret) {
        printf("\nMI_SYS_SetChnOutputPortDepth: %d -- VIF output is not CPU-readable in this\n"
               "  configuration; the counters above are the verification that matters\n", ret);
        ret = 0;
        goto out;
    }

    if (sys->fnGetFd) {
        ret = sys->fnGetFd(&bind, &fd);
        if (ret) {
            printf("MI_SYS_GetFd: %d (falling back to polling)\n", ret);
            fd = -1;
        }
    }

    printf("\nframe reads from VIF chn %d port %d:\n", PROBE_VIF_CHN, PROBE_VIF_PORT);

    while (captured < frames) {
        i6_sys_bufinfo buf;
        int handle = -1;

        if (fd >= 0) {
            fd_set rfds;
            struct timeval tv = {PROBE_FRAME_TIMEOUT_MS / 1000,
                                 (PROBE_FRAME_TIMEOUT_MS % 1000) * 1000};

            FD_ZERO(&rfds);
            FD_SET(fd, &rfds);
            ret = select(fd + 1, &rfds, NULL, NULL, &tv);
            if (ret < 0) {
                printf("  select: %s\n", strerror(errno));
                break;
            }
            if (!ret) {
                printf("  timeout after %d ms with no frame\n", PROBE_FRAME_TIMEOUT_MS);
                break;
            }
        }

        memset(&buf, 0, sizeof(buf));
        ret = sys->fnGetOutputBuf(&bind, &buf, &handle);
        if (ret) {
            printf("  MI_SYS_ChnOutputPortGetBuf: %d\n", ret);
            if (fd < 0) {
                struct timespec nap = {0, 50 * 1000 * 1000};

                nanosleep(&nap, NULL);
                continue;
            }
            break;
        }

        printf("  [%d] pts %llu  type %d  seq %u  %ux%u  pixFmt %d  stride %u/%u  "
               "size %u  eos %d drop %d\n",
               captured, buf.pts, buf.bufType, buf.seqNum, buf.frame.width, buf.frame.height,
               buf.frame.pixFmt, buf.frame.stride[0], buf.frame.stride[1], buf.frame.bufSize,
               buf.endOfStream, buf.drop);
        printf("       va %p/%p  pa 0x%llx  crop %ux%u+%u+%u\n", buf.frame.virAddr[0],
               buf.frame.virAddr[1], buf.frame.phyAddr[0], buf.frame.crop.width,
               buf.frame.crop.height, buf.frame.crop.x, buf.frame.crop.y);

        sys->fnPutOutputBuf(handle);
        captured++;
    }

    printf("captured %d of %d frame(s)\n", captured, frames);
    if (!captured)
        printf("  (expected: VIF in RGB_REALTIME streams to the ISP without touching DRAM,\n"
               "   so an unbound VIF has nothing to hand the CPU. Not a failure, but it does\n"
               "   leave i6_sys_bufinfo unexercised until 2c reads a VPE port.)\n");
    ret = 0;

out:
    if (fd >= 0 && sys->fnCloseFd)
        sys->fnCloseFd(fd);
    if (port_enabled)
        vif->fnDisablePort(PROBE_VIF_CHN, PROBE_VIF_PORT);
    if (dev_enabled)
        vif->fnDisableDevice(PROBE_VIF_DEV);

    return ret;
}

int main(int argc, char **argv)
{
    i6_sys_impl sys;
    i6_snr_impl snr;
    i6_vif_impl vif;
    i6_vpe_impl vpe;
    i6_sys_ver ver;
    const char *dir = PROBE_DUMP_DIR;
    unsigned int index = 0;
    int do_enable = 0;
    int do_vif = 0;
    int do_vpe = 0;
    int frames = 3;
    int i;

    int ret;

    for (i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-e"))
            do_enable = 1;
        else if (!strcmp(argv[i], "-f")) {
            do_vif = 1;
            do_enable = 1;
        } else if (!strcmp(argv[i], "-v")) {
            do_vpe = 1;
            do_vif = 1;
            do_enable = 1;
        } else if (!strcmp(argv[i], "-o") && i + 1 < argc) {
            dir = argv[++i];
        } else if (!strcmp(argv[i], "-n") && i + 1 < argc) {
            frames = (int)strtol(argv[++i], NULL, 0);
            if (frames < 0)
                frames = 0;
        } else if (!strcmp(argv[i], "-h")) {
            printf("usage: star_probe [-e] [-f] [-v] [-n count] [-o dir] [sensor-index]\n"
                   "  -e  run the sensor bring-up sequence (SetRes/SetFps/SetOrien/Enable)\n"
                   "      before querying, then Disable. Needed to see pad intfAttr and\n"
                   "      plane geometry, since those are only populated once the sensor\n"
                   "      driver's init has run. Starts the sensor -- stop any streamer\n"
                   "      first.\n"
                   "  -f  additionally bring up VIF and check that frames arrive, via\n"
                   "      /proc/mi_modules counters and then by draining the VIF output\n"
                   "      port. Implies -e. Everything is torn down again before exit.\n"
                   "  -v  additionally create the VPE channel, bind VIF to it, report which\n"
                   "      output ports MI accepts, and dump one NV12 frame from port 0 (full\n"
                   "      resolution) and port 1 (640x360). Implies -f, and replaces -f's\n"
                   "      VIF drain -- a realtime-bound VIF port has nothing to hand the CPU.\n"
                   "  -n  how many frames to try to read per port (default %d)\n"
                   "  -o  where -v writes its dumps (default %s)\n",
                   frames, dir);
            return 0;
        } else
            index = (unsigned int)strtoul(argv[i], NULL, 0);
    }

    printf("star_probe -- %s, sensor index %u%s%s%s\n\n", HAL_PLATFORM_NAME, index,
           do_enable ? ", with bring-up" : "", do_vif ? " + VIF" : "", do_vpe ? " + VPE" : "");
    print_sizes();

    memset(&sys, 0, sizeof(sys));
    memset(&snr, 0, sizeof(snr));
    memset(&vif, 0, sizeof(vif));
    memset(&vpe, 0, sizeof(vpe));

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

    /*
     * Media clock. The MI_SYS_GetCurPts signature was read off libmi_sys.so's
     * disassembly rather than guessed (see i6_sys.h), and the arity differs by
     * SoC family -- so confirm it here before rvd's frame loop depends on it.
     * Two reads a moment apart: a single value could be anything, whereas a
     * value that advances by roughly the elapsed time is a real clock reached
     * through a correctly-shaped call.
     */
    if (sys.fnGetCurrentPts) {
        unsigned long long a = 0, b = 0;
        int r1 = sys.fnGetCurrentPts(&a);
        struct timespec nap = {0, 100 * 1000 * 1000};

        nanosleep(&nap, NULL);
        int r2 = sys.fnGetCurrentPts(&b);

        if (r1 || r2)
            printf("MI_SYS_GetCurPts: %d/%d\n", r1, r2);
        else
            printf("pts: %llu -> %llu (delta %llu us over ~100000 us)\n\n", a, b, b - a);
    } else
        printf("MI_SYS_GetCurPts not exported -- SEI timecodes unavailable\n\n");

    /*
     * The frame-buffer symbols are resolved optionally (see i6_sys_load), so
     * report what actually resolved before anything depends on it -- "not
     * exported" and "exported but misused" look identical from a failed call.
     */
    if (do_vif)
        printf("frame symbols: GetBuf %c PutBuf %c GetFd %c CloseFd %c Flush %c Va2Pa %c\n\n",
               sys.fnGetOutputBuf ? 'y' : 'n', sys.fnPutOutputBuf ? 'y' : 'n',
               sys.fnGetFd ? 'y' : 'n', sys.fnCloseFd ? 'y' : 'n',
               sys.fnFlushInvCache ? 'y' : 'n', sys.fnVa2Pa ? 'y' : 'n');

    ret = probe_sensor(&snr, index, do_enable);

    if (!ret && do_vif) {
        int vret = i6_vif_load(&vif);

        if (vret)
            printf("i6_vif_load: %d\n", vret);
        else if (do_vpe && (vret = i6_vpe_load(&vpe)))
            printf("i6_vpe_load: %d\n", vret);
        else
            vret = probe_vif(&sys, &snr, &vif, do_vpe ? &vpe : NULL, index, frames, dir);
        i6_vpe_unload(&vpe);
        i6_vif_unload(&vif);
        if (vret)
            ret = vret;
    }

    /*
     * Always pair Enable with Disable, including on the failure paths above --
     * a probe that leaves the sensor streaming would poison the next run and
     * confuse whoever restarts the real streamer.
     */
    if (do_enable)
        snr.fnDisable(index);

    sys.fnExit();
    i6_snr_unload(&snr);
    i6_sys_unload(&sys);

    printf("\n%s\n", ret ? "FAILED" : "OK");
    return ret ? 1 : 0;
}
