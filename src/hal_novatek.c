/*
 * hal_novatek.c -- Novatek HDAL backend
 *
 * One capture path feeds the ISP (videoproc); every encoder channel owns
 * a videoproc output port and a videoenc path bound to it. There is no
 * framesource surface: enc_create_channel builds the whole chain, as the
 * v4l2 backend does, so rvd takes its has_framesource == false path.
 *
 * Board facts (MIPI lanes, pinmux, I2C bus) are not sensor facts and
 * live in /etc/sensor/<sensor>.pinmux, key=value.
 */

#include <errno.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <pthread.h>

#include "hdal.h"
#include "hd_util.h"

#include "raptor_hal.h"

#define NVT_MAX_CHN 4
#define NVT_PLATFORM_NAME "NA51055"

/* Largest frame the sensor can deliver; pools are sized for it so any
 * sensor mode the driver auto-selects fits. */
#define NVT_MAX_W 3840
#define NVT_MAX_H 2160
#define NVT_RAW_FMT HD_VIDEO_PXLFMT_RAW12

static void hal_log_stderr(int level, const char *file, int line, const char *fmt, ...)
    __attribute__((format(printf, 4, 5)));
static rss_hal_log_func_t log_fn = hal_log_stderr;

#define LOG_ERR(fmt, ...) log_fn(1, __FILE__, __LINE__, fmt, ##__VA_ARGS__)
#define LOG_WARN(fmt, ...) log_fn(2, __FILE__, __LINE__, fmt, ##__VA_ARGS__)
#define LOG_INFO(fmt, ...) log_fn(3, __FILE__, __LINE__, fmt, ##__VA_ARGS__)

static void hal_log_stderr(int level, const char *file, int line, const char *fmt, ...)
{
    static const char *const tag[] = {"FATAL", "ERROR", "WARN", "INFO", "DEBUG"};
    va_list ap;

    fprintf(stderr, "[hal %s] %s:%d: ", tag[level < 0 || level > 4 ? 4 : level], file, line);
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputc('\n', stderr);
}

void rss_hal_set_log_func(rss_hal_log_func_t func)
{
    log_fn = func ? func : hal_log_stderr;
}

enum {
    NVT_IMG_BRIGHTNESS,
    NVT_IMG_CONTRAST,
    NVT_IMG_SATURATION,
    NVT_IMG_SHARPNESS,
    NVT_IMG_HUE,
    NVT_IMG_NR,
    NVT_IMG_3DNR,
    NVT_IMG_AE_COMP,
    NVT_IMG_ANTIFLICKER,
    NVT_IMG_NIGHT,
    NVT_IMG_MAX
};

typedef struct {
    bool open;
    bool started;
    HD_PATH_ID prc;
    HD_PATH_ID enc;
    rss_video_config_t cfg;
    uintptr_t bs_va;
    uint32_t bs_pa;
    uint32_t bs_size;
    HD_VIDEOENC_BS bs;
    bool held;
    rss_nal_unit_t nals[VENC_BS_PACK_MAX];
    uint32_t seq;
} nvt_chn_t;

struct rss_hal_ctx {
    const rss_hal_ops_t *ops;
    rss_hal_caps_t caps;
    pthread_mutex_t lock;

    bool inited;
    char sensor[32];
    HD_PATH_ID cap_ctrl;
    HD_PATH_ID cap_path;
    HD_PATH_ID prc_ctrl;
    bool cap_configured;
    int running;
    uint32_t cap_fps;
    int hflip, vflip;
    uint32_t base_mirror, base_flip;
    /* Image controls, raptor's 0-255 scale (128 = neutral); -1 = unset. */
    int img[NVT_IMG_MAX];
    bool tuned;

    nvt_chn_t chn[NVT_MAX_CHN];
};

/* ── board pin config ── */

typedef struct {
    uint32_t if_type;
    uint32_t sensor_pinmux;
    uint32_t serial_if_pinmux;
    uint32_t cmd_if_pinmux;
    uint32_t clk_lane_sel;
    uint32_t pin_map[HD_VIDEOCAP_SEN_SER_MAX_DATALANE];
    int lanes;
} nvt_pins_t;

static int load_pins(const char *sensor, nvt_pins_t *p)
{
    char path[96], line[160];
    FILE *f;

    memset(p, 0, sizeof(*p));
    p->if_type = HD_COMMON_VIDEO_IN_MIPI_CSI;
    for (int i = 0; i < HD_VIDEOCAP_SEN_SER_MAX_DATALANE; i++)
        p->pin_map[i] = HD_VIDEOCAP_SEN_IGNORE;

    snprintf(path, sizeof(path), "/etc/sensor/%s.pinmux", sensor);
    f = fopen(path, "r");
    if (!f) {
        LOG_ERR("%s: %s (board pin config is required)", path, strerror(errno));
        return RSS_ERR_NOENT;
    }
    while (fgets(line, sizeof(line), f)) {
        char *eq = strchr(line, '=');
        if (line[0] == '#' || !eq)
            continue;
        *eq++ = '\0';
        if (!strcmp(line, "if_type"))
            p->if_type = strtoul(eq, NULL, 0);
        else if (!strcmp(line, "sensor_pinmux"))
            p->sensor_pinmux = strtoul(eq, NULL, 0);
        else if (!strcmp(line, "serial_if_pinmux"))
            p->serial_if_pinmux = strtoul(eq, NULL, 0);
        else if (!strcmp(line, "cmd_if_pinmux"))
            p->cmd_if_pinmux = strtoul(eq, NULL, 0);
        else if (!strcmp(line, "clk_lane_sel"))
            p->clk_lane_sel = strtoul(eq, NULL, 0);
        else if (!strcmp(line, "pin_map")) {
            char *s = eq, *end;
            p->lanes = 0;
            while (p->lanes < HD_VIDEOCAP_SEN_SER_MAX_DATALANE) {
                unsigned long v = strtoul(s, &end, 0);
                if (end == s)
                    break;
                p->pin_map[p->lanes++] = (uint32_t)v;
                s = (*end == ',') ? end + 1 : end;
            }
        }
    }
    fclose(f);
    return p->lanes ? RSS_OK : RSS_ERR_INVAL;
}

/* The sensor kernel module is the only record of which sensor this board
 * carries when the config names none. */
static int detect_sensor(char *name, size_t len)
{
    DIR *d = opendir("/sys/module");
    struct dirent *e;
    int ret = RSS_ERR_NOENT;

    if (!d)
        return RSS_ERR_NOENT;
    while ((e = readdir(d))) {
        if (!strncmp(e->d_name, "nvt_sen_", 8) && strcmp(e->d_name, "nvt_sen_off")) {
            snprintf(name, len, "%s", e->d_name + 8);
            ret = RSS_OK;
            break;
        }
    }
    closedir(d);
    return ret;
}

/* ── lifecycle ── */

static int nvt_mem_init(void)
{
    HD_COMMON_MEM_INIT_CONFIG mem = {0};
    /* RAW12 frame plus the CA (32x32) and LA (32x32) statistics windows
     * and the 0x200 debug header the capture driver prepends. */
    uint32_t raw = 0x200 + ((NVT_MAX_W * 12 / 8 + 3) & ~3u) * NVT_MAX_H + ((32 * 32 << 3) << 1) +
                   ((32 * 32 << 1) << 1);
    uint32_t yuv = 0x200 + NVT_MAX_W * NVT_MAX_H * 3 / 2;
    uint32_t yuv_sub = 0x200 + 1920 * 1080 * 3 / 2;

    mem.pool_info[0].type = HD_COMMON_MEM_COMMON_POOL;
    mem.pool_info[0].blk_size = raw;
    mem.pool_info[0].blk_cnt = 2;
    mem.pool_info[0].ddr_id = DDR_ID0;
    mem.pool_info[1].type = HD_COMMON_MEM_COMMON_POOL;
    mem.pool_info[1].blk_size = yuv;
    mem.pool_info[1].blk_cnt = 3;
    mem.pool_info[1].ddr_id = DDR_ID0;
    mem.pool_info[2].type = HD_COMMON_MEM_COMMON_POOL;
    mem.pool_info[2].blk_size = yuv_sub;
    mem.pool_info[2].blk_cnt = 4;
    mem.pool_info[2].ddr_id = DDR_ID0;
    return hd_common_mem_init(&mem) == HD_OK ? RSS_OK : RSS_ERR_NOMEM;
}

static int nvt_init(void *vctx, const rss_multi_sensor_config_t *multi)
{
    rss_hal_ctx_t *ctx = vctx;
    HD_VIDEOCAP_DRV_CONFIG drv = {0};
    HD_VIDEOCAP_CTRL ctl = {0};
    HD_VIDEOPROC_DEV_CONFIG pdev = {0};
    HD_VIDEOPROC_CTRL pctl = {0};
    nvt_pins_t pins;
    int ret;

    if (ctx->inited)
        return RSS_OK;

    if (multi && multi->sensor_count > 0 && multi->sensors[0].name[0])
        snprintf(ctx->sensor, sizeof(ctx->sensor), "%s", multi->sensors[0].name);
    else if (detect_sensor(ctx->sensor, sizeof(ctx->sensor)) != RSS_OK) {
        LOG_ERR("no sensor configured and no nvt_sen_* module loaded");
        return RSS_ERR_NOENT;
    }
    ret = load_pins(ctx->sensor, &pins);
    if (ret != RSS_OK)
        return ret;

    if (hd_common_init(0) != HD_OK) {
        LOG_ERR("hd_common_init failed");
        return RSS_ERR_IO;
    }
    ret = nvt_mem_init();
    if (ret != RSS_OK) {
        LOG_ERR("hd_common_mem_init failed (debugfs mounted?)");
        hd_common_uninit();
        return ret;
    }
    if (hd_videocap_init() != HD_OK || hd_videoproc_init() != HD_OK ||
        hd_videoenc_init() != HD_OK) {
        LOG_ERR("HDAL module init failed");
        goto fail;
    }

    snprintf(drv.sen_cfg.sen_dev.driver_name, HD_VIDEOCAP_SEN_NAME_LEN - 1, "nvt_sen_%s",
             ctx->sensor);
    drv.sen_cfg.sen_dev.if_type = pins.if_type;
    drv.sen_cfg.sen_dev.pin_cfg.pinmux.sensor_pinmux = pins.sensor_pinmux;
    drv.sen_cfg.sen_dev.pin_cfg.pinmux.serial_if_pinmux = pins.serial_if_pinmux;
    drv.sen_cfg.sen_dev.pin_cfg.pinmux.cmd_if_pinmux = pins.cmd_if_pinmux;
    drv.sen_cfg.sen_dev.pin_cfg.clk_lane_sel = pins.clk_lane_sel;
    for (int i = 0; i < HD_VIDEOCAP_SEN_SER_MAX_DATALANE; i++)
        drv.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[i] = pins.pin_map[i];

    if (hd_videocap_open(0, (HD_OUT_ID)HD_VIDEOCAP_0_CTRL, &ctx->cap_ctrl) != HD_OK ||
        hd_videocap_set(ctx->cap_ctrl, HD_VIDEOCAP_PARAM_DRV_CONFIG, &drv) != HD_OK) {
        LOG_ERR("sensor %s: capture driver config rejected", drv.sen_cfg.sen_dev.driver_name);
        goto fail;
    }
    ctl.func = HD_VIDEOCAP_FUNC_AE | HD_VIDEOCAP_FUNC_AWB;
    hd_videocap_set(ctx->cap_ctrl, HD_VIDEOCAP_PARAM_CTRL, &ctl);

    if (hd_videoproc_open(0, (HD_OUT_ID)HD_VIDEOPROC_0_CTRL, &ctx->prc_ctrl) != HD_OK)
        goto fail;
    pdev.pipe = HD_VIDEOPROC_PIPE_RAWALL;
    pdev.isp_id = 0;
    pdev.in_max.dim.w = NVT_MAX_W;
    pdev.in_max.dim.h = NVT_MAX_H;
    pdev.in_max.pxlfmt = NVT_RAW_FMT;
    pdev.in_max.frc = HD_VIDEO_FRC_RATIO(1, 1);
    if (hd_videoproc_set(ctx->prc_ctrl, HD_VIDEOPROC_PARAM_DEV_CONFIG, &pdev) != HD_OK) {
        LOG_ERR("videoproc device config rejected");
        goto fail;
    }
    hd_videoproc_set(ctx->prc_ctrl, HD_VIDEOPROC_PARAM_CTRL, &pctl);

    if (hd_videocap_open(HD_VIDEOCAP_0_IN_0, HD_VIDEOCAP_0_OUT_0, &ctx->cap_path) != HD_OK)
        goto fail;
    if (hd_videocap_bind(HD_VIDEOCAP_0_OUT_0, HD_VIDEOPROC_0_IN_0) != HD_OK)
        goto fail;

    ctx->inited = true;
    LOG_INFO("HDAL up: sensor nvt_sen_%s, %d MIPI lanes", ctx->sensor, pins.lanes);
    return RSS_OK;

fail:
    hd_videoenc_uninit();
    hd_videoproc_uninit();
    hd_videocap_uninit();
    hd_common_mem_uninit();
    hd_common_uninit();
    return RSS_ERR_IO;
}

static int nvt_deinit(void *vctx)
{
    rss_hal_ctx_t *ctx = vctx;

    if (!ctx->inited)
        return RSS_OK;
    if (ctx->running) {
        hd_videocap_stop(ctx->cap_path);
        ctx->running = 0;
    }
    hd_videocap_unbind(HD_VIDEOCAP_0_OUT_0);
    hd_videocap_close(ctx->cap_path);
    hd_videoenc_uninit();
    hd_videoproc_uninit();
    hd_videocap_uninit();
    hd_common_mem_uninit();
    hd_common_uninit();
    ctx->inited = false;
    ctx->cap_configured = false;
    return RSS_OK;
}

static const rss_hal_caps_t *nvt_get_caps(void *vctx)
{
    return &((rss_hal_ctx_t *)vctx)->caps;
}

/* ── sensor tuning ── */

/*
 * AE/AWB/IQ tables are a flattened device tree, /etc/sensor/<sensor>_isp.dtb,
 * as the vendor app ships them: /isp/sensor@0 names each table's node. The
 * vendor modules only read such files themselves; their AWB reader fails on
 * this kernel, so AWB is set table by table through the per-item calls.
 * vendor_isp ABI (HDAL 2.0): item 3 = *_ITEM_RLD_DTSI on all three.
 */
typedef struct {
    uint32_t id;
    char node_path[256];
    char file_path[256];
    uint8_t *buf_addr;
} nvt_dtsi_info_t;

extern HD_RESULT vendor_isp_init(void);
extern HD_RESULT vendor_isp_uninit(void);
extern HD_RESULT vendor_isp_set_ae(uint32_t item, void *p);
extern HD_RESULT vendor_isp_set_awb(uint32_t item, void *p);
extern HD_RESULT vendor_isp_set_iq(uint32_t item, void *p);
extern HD_RESULT vendor_isp_set_common(uint32_t item, void *p);
extern HD_RESULT vendor_isp_get_common(uint32_t item, void *p);

#define NVT_ITEM_RLD_DTSI 3
#define NVT_ISPT_ITEM_SENSOR_DIRECTION 44

static uint32_t be32(const uint8_t *p)
{
    return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3];
}

/* Structure-block offset of the node at path, or -1. */
static long fdt_node(const uint8_t *fdt, size_t len, const char *path)
{
    uint32_t off_st = be32(fdt + 8), off_str = be32(fdt + 12), sz = be32(fdt + 36);
    const uint8_t *p = fdt + off_st, *end = p + sz;
    char cur[256] = "";
    size_t depth_len[32];
    int depth = 0;

    (void)off_str;
    if (len < 40 || be32(fdt) != 0xd00dfeed || off_st + sz > len)
        return -1;
    while (p + 4 <= end) {
        uint32_t tag = be32(p);
        const uint8_t *tp = p;
        p += 4;
        if (tag == 1) { /* BEGIN_NODE */
            const char *name = (const char *)p;
            size_t n = strlen(name);
            if (depth >= 32)
                return -1;
            depth_len[depth++] = strlen(cur);
            if (depth > 1 && strlen(cur) + n + 2 < sizeof(cur)) {
                strcat(cur, "/");
                strcat(cur, name);
            }
            if (!strcmp(cur[0] ? cur : "/", path))
                return tp - fdt;
            p += (n + 4) & ~3u;
        } else if (tag == 2) { /* END_NODE */
            if (depth > 0)
                cur[depth_len[--depth]] = '\0';
        } else if (tag == 3) { /* PROP */
            p += 8 + ((be32(p) + 3) & ~3u);
        } else if (tag != 4) { /* not NOP */
            break;
        }
    }
    return -1;
}

/* Property of the node at node_off (no descent into children). */
static const uint8_t *fdt_prop(const uint8_t *fdt, long node_off, const char *name, uint32_t *plen)
{
    const char *strs = (const char *)fdt + be32(fdt + 12);
    const uint8_t *p = fdt + node_off + 4;

    p += (strlen((const char *)p) + 4) & ~3u;
    while (be32(p) == 3 || be32(p) == 4) {
        if (be32(p) == 4) {
            p += 4;
            continue;
        }
        uint32_t l = be32(p + 4), nameoff = be32(p + 8);
        if (!strcmp(strs + nameoff, name)) {
            *plen = l;
            return p + 12;
        }
        p += 12 + ((l + 3) & ~3u);
    }
    return NULL;
}

static void load_tuning(const char *sensor)
{
    static const struct {
        const char *node;
        uint32_t item; /* AWBT_ITEM_* */
    } awb[] = {
        {"awb_th", 31},       {"awb_lv", 32},           {"awb_ct_weight", 33},
        {"awb_target", 34},   {"awb_ct_info", 35},      {"awb_mwb", 36},
        {"awb_converge", 37}, {"awb_expand_block", 38}, {"awb_luma_weight", 43},
    };
    char file[96], ae[128] = "", awbp[128] = "", iq[128] = "";
    uint8_t *fdt = NULL;
    long n, s0;
    FILE *f;
    int awb_ok = 0;

    snprintf(file, sizeof(file), "/etc/sensor/%s_isp.dtb", sensor);
    f = fopen(file, "rb");
    if (!f) {
        LOG_INFO("no %s: ISP runs on its built-in defaults", file);
        return;
    }
    fseek(f, 0, SEEK_END);
    n = ftell(f);
    rewind(f);
    fdt = n > 0 ? malloc(n) : NULL;
    if (!fdt || fread(fdt, 1, n, f) != (size_t)n) {
        fclose(f);
        free(fdt);
        return;
    }
    fclose(f);

    s0 = fdt_node(fdt, n, "/isp/sensor@0");
    if (s0 >= 0) {
        uint32_t l;
        const uint8_t *v;
        if ((v = fdt_prop(fdt, s0, "ae_path", &l)) && l < sizeof(ae))
            memcpy(ae, v, l);
        if ((v = fdt_prop(fdt, s0, "awb_path", &l)) && l < sizeof(awbp))
            memcpy(awbp, v, l);
        if ((v = fdt_prop(fdt, s0, "iq_path", &l)) && l < sizeof(iq))
            memcpy(iq, v, l);
    }
    if (vendor_isp_init() != HD_OK) {
        LOG_WARN("vendor_isp_init failed: tuning not loaded");
        free(fdt);
        return;
    }
    if (ae[0]) {
        nvt_dtsi_info_t d = {0};
        snprintf(d.node_path, sizeof(d.node_path), "%s", ae);
        snprintf(d.file_path, sizeof(d.file_path), "%s", file);
        if (vendor_isp_set_ae(NVT_ITEM_RLD_DTSI, &d) != HD_OK)
            LOG_WARN("AE tuning %s rejected", ae);
    }
    if (iq[0]) {
        nvt_dtsi_info_t d = {0};
        snprintf(d.node_path, sizeof(d.node_path), "%s", iq);
        snprintf(d.file_path, sizeof(d.file_path), "%s", file);
        if (vendor_isp_set_iq(NVT_ITEM_RLD_DTSI, &d) != HD_OK)
            LOG_WARN("IQ tuning %s rejected", iq);
    }
    for (size_t i = 0; awbp[0] && i < sizeof(awb) / sizeof(awb[0]); i++) {
        char path[192];
        uint8_t buf[4 + 512] = {0};
        uint32_t l;
        const uint8_t *v;
        long off;

        snprintf(path, sizeof(path), "%s/%s", awbp, awb[i].node);
        off = fdt_node(fdt, n, path);
        if (off < 0 || !(v = fdt_prop(fdt, off, "data", &l)) || l > sizeof(buf) - 4)
            continue;
        memcpy(buf + 4, v, l); /* {AWB_ID id = 0, table} */
        if (vendor_isp_set_awb(awb[i].item, buf) == HD_OK)
            awb_ok++;
    }
    vendor_isp_uninit();
    free(fdt);
    LOG_INFO("tuning %s: AE %s, IQ %s, AWB %d tables", file, ae[0] ? ae : "-", iq[0] ? iq : "-",
             awb_ok);
}

/* ── capture ── */

static int cap_configure(rss_hal_ctx_t *ctx, const rss_video_config_t *cfg)
{
    HD_VIDEOCAP_IN in = {0};
    HD_VIDEOCAP_CROP crop = {0};
    HD_VIDEOCAP_OUT out = {0};

    if (ctx->cap_configured)
        return RSS_OK;
    /* AUTO picks the smallest sensor mode covering this size and rate;
     * channel 0 is the main stream, so it decides. */
    in.sen_mode = HD_VIDEOCAP_SEN_MODE_AUTO;
    in.frc = HD_VIDEO_FRC_RATIO(cfg->fps_num, cfg->fps_den ? cfg->fps_den : 1);
    in.dim.w = cfg->width;
    in.dim.h = cfg->height;
    in.pxlfmt = NVT_RAW_FMT;
    in.out_frame_num = HD_VIDEOCAP_SEN_FRAME_NUM_1;
    if (hd_videocap_set(ctx->cap_path, HD_VIDEOCAP_PARAM_IN, &in) != HD_OK) {
        LOG_ERR("no sensor mode for %ux%u@%u/%u", cfg->width, cfg->height, cfg->fps_num,
                cfg->fps_den);
        return RSS_ERR_INVAL;
    }
    crop.mode = HD_CROP_OFF;
    hd_videocap_set(ctx->cap_path, HD_VIDEOCAP_PARAM_IN_CROP, &crop);
    out.pxlfmt = NVT_RAW_FMT;
    out.dir = HD_VIDEO_DIR_NONE;
    if (hd_videocap_set(ctx->cap_path, HD_VIDEOCAP_PARAM_OUT, &out) != HD_OK)
        return RSS_ERR_INVAL;
    ctx->cap_configured = true;
    ctx->cap_fps = cfg->fps_den ? cfg->fps_num / cfg->fps_den : cfg->fps_num;
    return RSS_OK;
}

/* Mirror/flip of a MIPI sensor is its readout direction, set through the
 * ISP (capture direction is CCIR-only, the ISP outputs refuse mirror in
 * RAWALL mode). The board's upright direction comes from the sensor's DT
 * node at stream start; user flips are relative to it. */
typedef struct {
    uint32_t id;
    uint32_t mirror, flip;
} nvt_sen_dir_t;

static void img_apply_all(rss_hal_ctx_t *ctx);

static void cap_apply_dir(rss_hal_ctx_t *ctx, bool first)
{
    nvt_sen_dir_t dir = {0};

    if (!ctx->running || vendor_isp_init() != HD_OK)
        return;
    if (first) {
        if (vendor_isp_get_common(NVT_ISPT_ITEM_SENSOR_DIRECTION, &dir) == HD_OK) {
            ctx->base_mirror = dir.mirror;
            ctx->base_flip = dir.flip;
        }
        if (!ctx->hflip && !ctx->vflip)
            goto out;
    }
    dir.id = 0;
    dir.mirror = !ctx->base_mirror != !ctx->hflip;
    dir.flip = !ctx->base_flip != !ctx->vflip;
    if (vendor_isp_set_common(NVT_ISPT_ITEM_SENSOR_DIRECTION, &dir) != HD_OK)
        LOG_WARN("sensor mirror %u flip %u not applied", dir.mirror, dir.flip);
out:
    vendor_isp_uninit();
}

/* ── encoder ── */

static nvt_chn_t *get_chn(rss_hal_ctx_t *ctx, int chn)
{
    if (chn < 0 || chn >= NVT_MAX_CHN)
        return NULL;
    return &ctx->chn[chn];
}

static int set_rc(nvt_chn_t *c, rss_rc_mode_t mode, uint32_t bitrate)
{
    HD_H26XENC_RATE_CONTROL rc = {0};
    uint32_t fb = c->cfg.fps_num ? c->cfg.fps_num : 30;
    uint32_t fi = c->cfg.fps_den ? c->cfg.fps_den : 1;
    uint8_t qmin = c->cfg.min_qp > 0 ? (uint8_t)c->cfg.min_qp : 10;
    uint8_t qmax = c->cfg.max_qp > 0 ? (uint8_t)c->cfg.max_qp : 45;
    uint8_t qinit = c->cfg.init_qp > 0 ? (uint8_t)c->cfg.init_qp : 26;

    switch (mode) {
    case RSS_RC_FIXQP:
        rc.rc_mode = HD_RC_MODE_FIX_QP;
        rc.fixqp.frame_rate_base = fb;
        rc.fixqp.frame_rate_incr = fi;
        rc.fixqp.fix_i_qp = qinit;
        rc.fixqp.fix_p_qp = qinit;
        break;
    case RSS_RC_CBR:
        rc.rc_mode = HD_RC_MODE_CBR;
        rc.cbr.bitrate = bitrate;
        rc.cbr.frame_rate_base = fb;
        rc.cbr.frame_rate_incr = fi;
        rc.cbr.init_i_qp = rc.cbr.init_p_qp = qinit;
        rc.cbr.min_i_qp = rc.cbr.min_p_qp = qmin;
        rc.cbr.max_i_qp = rc.cbr.max_p_qp = qmax;
        rc.cbr.static_time = 4;
        break;
    default:
        rc.rc_mode = HD_RC_MODE_VBR;
        rc.vbr.bitrate = bitrate;
        rc.vbr.frame_rate_base = fb;
        rc.vbr.frame_rate_incr = fi;
        rc.vbr.init_i_qp = rc.vbr.init_p_qp = qinit;
        rc.vbr.min_i_qp = rc.vbr.min_p_qp = qmin;
        rc.vbr.max_i_qp = rc.vbr.max_p_qp = qmax;
        rc.vbr.static_time = 4;
        break;
    }
    if (hd_videoenc_set(c->enc, HD_VIDEOENC_PARAM_OUT_RATE_CONTROL, &rc) != HD_OK)
        return RSS_ERR_INVAL;
    c->cfg.rc_mode = mode;
    c->cfg.bitrate = bitrate;
    return RSS_OK;
}

static int set_enc_out(nvt_chn_t *c)
{
    HD_VIDEOENC_OUT out = {0};

    if (c->cfg.codec == RSS_CODEC_JPEG) {
        out.codec_type = HD_CODEC_TYPE_JPEG;
        out.jpeg.image_quality = c->cfg.init_qp > 0 ? (UINT32)c->cfg.init_qp : 75;
        return hd_videoenc_set(c->enc, HD_VIDEOENC_PARAM_OUT_ENC_PARAM, &out) == HD_OK
                   ? RSS_OK
                   : RSS_ERR_INVAL;
    }
    if (c->cfg.codec == RSS_CODEC_H265) {
        out.codec_type = HD_CODEC_TYPE_H265;
        out.h26x.profile = HD_H265E_MAIN_PROFILE;
        out.h26x.level_idc = HD_H265E_LEVEL_5;
        out.h26x.entropy_mode = HD_H265E_CABAC_CODING;
    } else {
        out.codec_type = HD_CODEC_TYPE_H264;
        out.h26x.profile = c->cfg.profile == 0   ? HD_H264E_BASELINE_PROFILE
                           : c->cfg.profile == 1 ? HD_H264E_MAIN_PROFILE
                                                 : HD_H264E_HIGH_PROFILE;
        out.h26x.level_idc = HD_H264E_LEVEL_5_1;
        out.h26x.entropy_mode = c->cfg.profile == 0 ? HD_H264E_CAVLC_CODING : HD_H264E_CABAC_CODING;
    }
    out.h26x.gop_num = c->cfg.gop_length ? c->cfg.gop_length : 2 * c->cfg.fps_num;
    out.h26x.svc_layer = HD_SVC_DISABLE;
    return hd_videoenc_set(c->enc, HD_VIDEOENC_PARAM_OUT_ENC_PARAM, &out) == HD_OK ? RSS_OK
                                                                                   : RSS_ERR_INVAL;
}

static int nvt_enc_create_channel(void *vctx, int chn, const rss_video_config_t *cfg)
{
    rss_hal_ctx_t *ctx = vctx;
    nvt_chn_t *c = get_chn(ctx, chn);
    HD_VIDEOPROC_OUT pout = {0};
    HD_VIDEOENC_PATH_CONFIG pcfg = {0};
    HD_VIDEOENC_IN ein = {0};
    int ret;

    if (!c || !cfg)
        return RSS_ERR_INVAL;
    if (!ctx->inited)
        return RSS_ERR_BUSY;
    if (c->open)
        return RSS_ERR_BUSY;
    if (cfg->codec != RSS_CODEC_H264 && cfg->codec != RSS_CODEC_H265 &&
        cfg->codec != RSS_CODEC_JPEG)
        return RSS_ERR_NOTSUP;

    pthread_mutex_lock(&ctx->lock);
    memset(c, 0, sizeof(*c));
    c->cfg = *cfg;
    if (!c->cfg.fps_num) {
        c->cfg.fps_num = 30;
        c->cfg.fps_den = 1;
    }

    ret = cap_configure(ctx, &c->cfg);
    if (ret != RSS_OK)
        goto out;

    ret = RSS_ERR_IO;
    if (hd_videoproc_open(HD_VIDEOPROC_0_IN_0, HD_VIDEOPROC_OUT(0, chn), &c->prc) != HD_OK)
        goto out;
    pout.dim.w = cfg->width;
    pout.dim.h = cfg->height;
    pout.pxlfmt = HD_VIDEO_PXLFMT_YUV420;
    pout.dir = HD_VIDEO_DIR_NONE;
    pout.frc = HD_VIDEO_FRC_RATIO(1, 1);
    /* A snapshot channel takes only every n-th frame from the ISP, so the
     * JPEG engine never sees the full sensor rate. */
    if (cfg->codec == RSS_CODEC_JPEG && ctx->cap_fps > c->cfg.fps_num)
        pout.frc = HD_VIDEO_FRC_RATIO(c->cfg.fps_num, ctx->cap_fps);
    if (hd_videoproc_set(c->prc, HD_VIDEOPROC_PARAM_OUT, &pout) != HD_OK) {
        LOG_ERR("chn%d: videoproc output %ux%u rejected", chn, cfg->width, cfg->height);
        goto close_prc;
    }

    if (hd_videoenc_open(HD_VIDEOENC_IN(0, chn), HD_VIDEOENC_OUT(0, chn), &c->enc) != HD_OK)
        goto close_prc;
    pcfg.max_mem.codec_type = cfg->codec == RSS_CODEC_JPEG   ? HD_CODEC_TYPE_JPEG
                              : cfg->codec == RSS_CODEC_H265 ? HD_CODEC_TYPE_H265
                                                             : HD_CODEC_TYPE_H264;
    pcfg.max_mem.max_dim.w = cfg->width;
    pcfg.max_mem.max_dim.h = cfg->height;
    pcfg.max_mem.bitrate = cfg->max_bitrate > cfg->bitrate ? cfg->max_bitrate : cfg->bitrate;
    if (!pcfg.max_mem.bitrate) /* JPEG: size the buffer as for a busy stream */
        pcfg.max_mem.bitrate = 8000000;
    pcfg.max_mem.enc_buf_ms = 3000;
    pcfg.max_mem.svc_layer = HD_SVC_DISABLE;
    pcfg.max_mem.ltr = FALSE;
    pcfg.isp_id = 0;
    if (hd_videoenc_set(c->enc, HD_VIDEOENC_PARAM_PATH_CONFIG, &pcfg) != HD_OK) {
        LOG_ERR("chn%d: encoder memory config rejected", chn);
        goto close_enc;
    }
    ein.dir = HD_VIDEO_DIR_NONE;
    ein.pxl_fmt = HD_VIDEO_PXLFMT_YUV420;
    ein.dim.w = cfg->width;
    ein.dim.h = cfg->height;
    ein.frc = HD_VIDEO_FRC_RATIO(1, 1);
    if (hd_videoenc_set(c->enc, HD_VIDEOENC_PARAM_IN, &ein) != HD_OK || set_enc_out(c) != RSS_OK ||
        (cfg->codec != RSS_CODEC_JPEG &&
         set_rc(c, cfg->rc_mode, cfg->bitrate ? cfg->bitrate : 4000000) != RSS_OK)) {
        LOG_ERR("chn%d: encoder parameters rejected", chn);
        goto close_enc;
    }
    if (hd_videoproc_bind(HD_VIDEOPROC_OUT(0, chn), HD_VIDEOENC_IN(0, chn)) != HD_OK)
        goto close_enc;

    c->open = true;
    ret = RSS_OK;
    LOG_INFO("chn%d: %s %ux%u %u/%u fps %u bps", chn,
             cfg->codec == RSS_CODEC_JPEG   ? "JPEG"
             : cfg->codec == RSS_CODEC_H265 ? "H.265"
                                            : "H.264",
             cfg->width, cfg->height, c->cfg.fps_num, c->cfg.fps_den, c->cfg.bitrate);
    goto out;

close_enc:
    hd_videoenc_close(c->enc);
close_prc:
    hd_videoproc_close(c->prc);
out:
    pthread_mutex_unlock(&ctx->lock);
    return ret;
}

static int nvt_enc_stop(void *vctx, int chn);

static int nvt_enc_destroy_channel(void *vctx, int chn)
{
    rss_hal_ctx_t *ctx = vctx;
    nvt_chn_t *c = get_chn(ctx, chn);

    if (!c || !c->open)
        return RSS_ERR_INVAL;
    if (c->started)
        nvt_enc_stop(vctx, chn);
    pthread_mutex_lock(&ctx->lock);
    hd_videoproc_unbind(HD_VIDEOPROC_OUT(0, chn));
    hd_videoenc_close(c->enc);
    hd_videoproc_close(c->prc);
    c->open = false;
    pthread_mutex_unlock(&ctx->lock);
    return RSS_OK;
}

static int nvt_enc_start(void *vctx, int chn)
{
    rss_hal_ctx_t *ctx = vctx;
    nvt_chn_t *c = get_chn(ctx, chn);
    HD_VIDEOENC_BUFINFO bi = {0};
    int ret = RSS_ERR_IO;

    if (!c || !c->open)
        return RSS_ERR_INVAL;
    if (c->started)
        return RSS_OK;

    pthread_mutex_lock(&ctx->lock);
    if (!ctx->running && hd_videocap_start(ctx->cap_path) != HD_OK) {
        LOG_ERR("capture start failed");
        goto out;
    }
    ctx->running++;
    if (ctx->running == 1)
        cap_apply_dir(ctx, true);
    if (!ctx->tuned) {
        load_tuning(ctx->sensor);
        ctx->tuned = true;
        img_apply_all(ctx);
    }
    if (hd_videoproc_start(c->prc) != HD_OK || hd_videoenc_start(c->enc) != HD_OK) {
        LOG_ERR("chn%d: start failed", chn);
        goto undo;
    }
    /* The bitstream buffer only exists once the path is started. */
    if (hd_videoenc_get(c->enc, HD_VIDEOENC_PARAM_BUFINFO, &bi) != HD_OK)
        goto undo;
    c->bs_pa = bi.buf_info.phy_addr;
    c->bs_size = bi.buf_info.buf_size;
    c->bs_va = (uintptr_t)hd_common_mem_mmap(HD_COMMON_MEM_MEM_TYPE_CACHE, c->bs_pa, c->bs_size);
    if (!c->bs_va)
        goto undo;
    c->started = true;
    ret = RSS_OK;
    goto out;

undo:
    hd_videoenc_stop(c->enc);
    hd_videoproc_stop(c->prc);
    if (--ctx->running == 0)
        hd_videocap_stop(ctx->cap_path);
out:
    pthread_mutex_unlock(&ctx->lock);
    return ret;
}

static int nvt_enc_stop(void *vctx, int chn)
{
    rss_hal_ctx_t *ctx = vctx;
    nvt_chn_t *c = get_chn(ctx, chn);

    if (!c || !c->started)
        return RSS_OK;
    pthread_mutex_lock(&ctx->lock);
    if (c->held) {
        hd_videoenc_release_out_buf(c->enc, &c->bs);
        c->held = false;
    }
    hd_videoenc_stop(c->enc);
    hd_videoproc_stop(c->prc);
    hd_common_mem_munmap((void *)c->bs_va, c->bs_size);
    c->bs_va = 0;
    c->started = false;
    if (--ctx->running == 0)
        hd_videocap_stop(ctx->cap_path);
    pthread_mutex_unlock(&ctx->lock);
    return RSS_OK;
}

static int nvt_enc_poll(void *vctx, int chn, uint32_t timeout_ms)
{
    nvt_chn_t *c = get_chn(vctx, chn);
    HD_VIDEOENC_POLL_LIST pl = {0};

    if (!c || !c->started)
        return RSS_ERR_INVAL;
    pl.path_id = c->enc;
    if (hd_videoenc_poll_list(&pl, 1, (INT32)timeout_ms) != HD_OK || !pl.revent.event)
        return RSS_ERR_TIMEOUT;
    return RSS_OK;
}

static rss_nal_type_t nal_type(rss_codec_t codec, HD_NALU_TYPE t)
{
    if (codec == RSS_CODEC_JPEG)
        return RSS_NAL_JPEG_FRAME;
    if (codec == RSS_CODEC_H265) {
        switch (t.h265_type) {
        case H265_NALU_TYPE_VPS:
            return RSS_NAL_H265_VPS;
        case H265_NALU_TYPE_SPS:
            return RSS_NAL_H265_SPS;
        case H265_NALU_TYPE_PPS:
            return RSS_NAL_H265_PPS;
        case H265_NALU_TYPE_SEI_PREFIX:
        case H265_NALU_TYPE_SEI_SUFFIX:
            return RSS_NAL_H265_SEI;
        case H265_NALU_TYPE_IDR:
            return RSS_NAL_H265_IDR;
        case H265_NALU_TYPE_SLICE:
            return RSS_NAL_H265_SLICE;
        default:
            return RSS_NAL_UNKNOWN;
        }
    }
    switch (t.h264_type) {
    case H264_NALU_TYPE_SPS:
        return RSS_NAL_H264_SPS;
    case H264_NALU_TYPE_PPS:
        return RSS_NAL_H264_PPS;
    case H264_NALU_TYPE_SEI:
        return RSS_NAL_H264_SEI;
    case H264_NALU_TYPE_IDR:
        return RSS_NAL_H264_IDR;
    case H264_NALU_TYPE_SLICE:
        return RSS_NAL_H264_SLICE;
    default:
        return RSS_NAL_UNKNOWN;
    }
}

static int nvt_enc_get_frame(void *vctx, int chn, rss_frame_t *frame)
{
    nvt_chn_t *c = get_chn(vctx, chn);
    HD_RESULT r;

    if (!c || !c->started || !frame)
        return RSS_ERR_INVAL;
    if (c->held)
        return RSS_ERR_BUSY;
    r = hd_videoenc_pull_out_buf(c->enc, &c->bs, 0);
    if (r != HD_OK)
        return r == HD_ERR_TIMEDOUT ? RSS_ERR_TIMEOUT : RSS_ERR_IO;
    c->held = true;

    for (uint32_t i = 0; i < c->bs.pack_num && i < VENC_BS_PACK_MAX; i++) {
        const HD_VIDEO_BS_DATA *p = &c->bs.video_pack[i];
        c->nals[i].data = (const uint8_t *)(c->bs_va + (p->phy_addr - c->bs_pa));
        c->nals[i].length = p->size;
        c->nals[i].type = nal_type(c->cfg.codec, p->pack_type);
        c->nals[i].frame_end = (i + 1 == c->bs.pack_num);
    }
    /* The buffer is mapped cached and the encoder wrote it by DMA. */
    for (uint32_t i = 0; i < c->bs.pack_num && i < VENC_BS_PACK_MAX; i++)
        hd_common_mem_flush_cache((void *)c->nals[i].data, c->nals[i].length);

    frame->nals = c->nals;
    frame->nal_count = c->bs.pack_num < VENC_BS_PACK_MAX ? c->bs.pack_num : VENC_BS_PACK_MAX;
    frame->codec = c->cfg.codec;
    frame->timestamp = (int64_t)c->bs.timestamp;
    frame->seq = c->seq++;
    frame->is_key = c->cfg.codec == RSS_CODEC_JPEG || c->bs.frame_type == HD_FRAME_TYPE_IDR ||
                    c->bs.frame_type == HD_FRAME_TYPE_I;
    frame->_priv = c;
    return RSS_OK;
}

static int nvt_enc_release_frame(void *vctx, int chn, rss_frame_t *frame)
{
    nvt_chn_t *c = get_chn(vctx, chn);

    (void)frame;
    if (!c || !c->held)
        return RSS_ERR_INVAL;
    hd_videoenc_release_out_buf(c->enc, &c->bs);
    c->held = false;
    return RSS_OK;
}

static int nvt_enc_request_idr(void *vctx, int chn)
{
    nvt_chn_t *c = get_chn(vctx, chn);
    HD_H26XENC_REQUEST_IFRAME req = {.enable = TRUE};

    if (!c || !c->open)
        return RSS_ERR_INVAL;
    return hd_videoenc_set(c->enc, HD_VIDEOENC_PARAM_OUT_REQUEST_IFRAME, &req) == HD_OK
               ? RSS_OK
               : RSS_ERR_IO;
}

/* HDAL applies changed encoder parameters on the next start call. */
static int apply_live(nvt_chn_t *c)
{
    if (c->started && hd_videoenc_start(c->enc) != HD_OK)
        return RSS_ERR_IO;
    return RSS_OK;
}

static int nvt_enc_set_rc_mode(void *vctx, int chn, rss_rc_mode_t mode, uint32_t bitrate)
{
    nvt_chn_t *c = get_chn(vctx, chn);
    int ret;

    if (!c || !c->open)
        return RSS_ERR_INVAL;
    ret = set_rc(c, mode, bitrate);
    return ret == RSS_OK ? apply_live(c) : ret;
}

static int nvt_enc_set_bitrate(void *vctx, int chn, uint32_t bitrate)
{
    nvt_chn_t *c = get_chn(vctx, chn);

    if (!c || !c->open)
        return RSS_ERR_INVAL;
    return nvt_enc_set_rc_mode(vctx, chn, c->cfg.rc_mode, bitrate);
}

static int nvt_enc_set_gop(void *vctx, int chn, uint32_t gop)
{
    nvt_chn_t *c = get_chn(vctx, chn);

    if (!c || !c->open || !gop)
        return RSS_ERR_INVAL;
    c->cfg.gop_length = gop;
    if (set_enc_out(c) != RSS_OK)
        return RSS_ERR_INVAL;
    return apply_live(c);
}

static int nvt_enc_get_gop(void *vctx, int chn, uint32_t *gop)
{
    nvt_chn_t *c = get_chn(vctx, chn);

    if (!c || !c->open || !gop)
        return RSS_ERR_INVAL;
    *gop = c->cfg.gop_length;
    return RSS_OK;
}

static int nvt_enc_get_channel_attr(void *vctx, int chn, rss_video_config_t *cfg)
{
    nvt_chn_t *c = get_chn(vctx, chn);

    if (!c || !c->open || !cfg)
        return RSS_ERR_INVAL;
    *cfg = c->cfg;
    return RSS_OK;
}

static int nvt_enc_get_fps(void *vctx, int chn, uint32_t *num, uint32_t *den)
{
    nvt_chn_t *c = get_chn(vctx, chn);

    if (!c || !c->open || !num || !den)
        return RSS_ERR_INVAL;
    *num = c->cfg.fps_num;
    *den = c->cfg.fps_den;
    return RSS_OK;
}

/* ── system / ISP ── */

static int nvt_sys_get_timestamp(void *vctx, int64_t *ts)
{
    (void)vctx;
    if (!ts)
        return RSS_ERR_INVAL;
    *ts = (int64_t)hd_gettime_us();
    return RSS_OK;
}

/* Mirror and flip are capture-path direction flags; they take effect on
 * the next capture configuration. */
/* ── image controls ── */

/* vendor_isp items (HDAL 2.0): IQT_ITEM_* and AET_ITEM_*. All take
 * {UINT32 id; UINT32 value}. IQ levels run 0..200 with 100 = default. */
#define NVT_IQT_NR_LV 10
#define NVT_IQT_SHARPNESS_LV 11
#define NVT_IQT_SATURATION_LV 12
#define NVT_IQT_CONTRAST_LV 13
#define NVT_IQT_BRIGHTNESS_LV 14
#define NVT_IQT_NIGHT_MODE 15
#define NVT_IQT_HUE_SHIFT 20
#define NVT_IQT_3DNR_LV 22
#define NVT_AET_FREQUENCY 12
#define NVT_AET_EV 14
#define NVT_AE_EV_0 16 /* AE_EV_00 of 33 steps, AE_EV_N4..AE_EV_P4 */

static uint32_t to_lv(int v)
{
    v = v < 0 ? 0 : v > 255 ? 255 : v;
    return (uint32_t)(v * 200 + 127) / 255;
}

static void img_apply(rss_hal_ctx_t *ctx, int which)
{
    struct {
        uint32_t id, val;
    } p = {0, 0};
    int v = ctx->img[which];
    HD_RESULT r = HD_OK;

    if (v < 0 || !ctx->running || vendor_isp_init() != HD_OK)
        return;
    switch (which) {
    case NVT_IMG_BRIGHTNESS:
        p.val = to_lv(v);
        r = vendor_isp_set_iq(NVT_IQT_BRIGHTNESS_LV, &p);
        break;
    case NVT_IMG_CONTRAST:
        p.val = to_lv(v);
        r = vendor_isp_set_iq(NVT_IQT_CONTRAST_LV, &p);
        break;
    case NVT_IMG_SATURATION:
        p.val = to_lv(v);
        r = vendor_isp_set_iq(NVT_IQT_SATURATION_LV, &p);
        break;
    case NVT_IMG_SHARPNESS:
        p.val = to_lv(v);
        r = vendor_isp_set_iq(NVT_IQT_SHARPNESS_LV, &p);
        break;
    case NVT_IMG_NR:
        p.val = to_lv(v);
        r = vendor_isp_set_iq(NVT_IQT_NR_LV, &p);
        break;
    case NVT_IMG_3DNR:
        p.val = to_lv(v);
        r = vendor_isp_set_iq(NVT_IQT_3DNR_LV, &p);
        break;
    case NVT_IMG_HUE:
        /* 128 = no shift; the 0..255 range spans -180..+180 degrees */
        p.val = (uint32_t)((((v - 128) * 180 / 128) + 360) % 360);
        r = vendor_isp_set_iq(NVT_IQT_HUE_SHIFT, &p);
        break;
    case NVT_IMG_NIGHT:
        p.val = v ? 1 : 0;
        r = vendor_isp_set_iq(NVT_IQT_NIGHT_MODE, &p);
        break;
    case NVT_IMG_AE_COMP: {
        /* IMP's ae_comp is centred on 128 (90..250); map onto EV steps */
        int step = NVT_AE_EV_0 + (v - 128) * NVT_AE_EV_0 / 122;
        p.val = (uint32_t)(step < 0 ? 0 : step > 2 * NVT_AE_EV_0 ? 2 * NVT_AE_EV_0 : step);
        r = vendor_isp_set_ae(NVT_AET_EV, &p);
        break;
    }
    case NVT_IMG_ANTIFLICKER:
        if (v == RSS_ANTIFLICKER_OFF)
            break; /* the AE has no "off": keep the current band */
        p.val = v == RSS_ANTIFLICKER_50HZ ? 0 : 1;
        r = vendor_isp_set_ae(NVT_AET_FREQUENCY, &p);
        break;
    }
    if (r != HD_OK)
        LOG_WARN("image control %d = %d rejected (%d)", which, v, r);
    vendor_isp_uninit();
}

static void img_apply_all(rss_hal_ctx_t *ctx)
{
    for (int i = 0; i < NVT_IMG_MAX; i++)
        img_apply(ctx, i);
}

static int img_set(void *vctx, int which, int val)
{
    rss_hal_ctx_t *ctx = vctx;

    pthread_mutex_lock(&ctx->lock);
    ctx->img[which] = val;
    if (ctx->tuned)
        img_apply(ctx, which);
    pthread_mutex_unlock(&ctx->lock);
    return RSS_OK;
}

static int nvt_isp_set_brightness(void *ctx, int v)
{
    return img_set(ctx, NVT_IMG_BRIGHTNESS, v);
}

static int nvt_isp_set_contrast(void *ctx, int v)
{
    return img_set(ctx, NVT_IMG_CONTRAST, v);
}

static int nvt_isp_set_saturation(void *ctx, int v)
{
    return img_set(ctx, NVT_IMG_SATURATION, v);
}

static int nvt_isp_set_sharpness(void *ctx, int v)
{
    return img_set(ctx, NVT_IMG_SHARPNESS, v);
}

static int nvt_isp_set_hue(void *ctx, int v)
{
    return img_set(ctx, NVT_IMG_HUE, v);
}

static int nvt_isp_set_sinter(void *ctx, int v)
{
    return img_set(ctx, NVT_IMG_NR, v);
}

static int nvt_isp_set_temper(void *ctx, int v)
{
    return img_set(ctx, NVT_IMG_3DNR, v);
}

static int nvt_isp_set_ae_comp(void *ctx, int v)
{
    return img_set(ctx, NVT_IMG_AE_COMP, v);
}

static int nvt_isp_set_antiflicker(void *ctx, rss_antiflicker_t mode)
{
    return img_set(ctx, NVT_IMG_ANTIFLICKER, (int)mode);
}

static int nvt_isp_set_running_mode(void *ctx, rss_isp_mode_t mode)
{
    return img_set(ctx, NVT_IMG_NIGHT, mode == RSS_ISP_NIGHT);
}

/* Getters answer from the cache; unset controls read as neutral. */
static int img_get(void *vctx, int which)
{
    rss_hal_ctx_t *ctx = vctx;
    int v;

    pthread_mutex_lock(&ctx->lock);
    v = ctx->img[which];
    pthread_mutex_unlock(&ctx->lock);
    return v;
}

#define NVT_IMG_GETTER_U8(fn, which)                                                               \
    static int fn(void *ctx, uint8_t *val)                                                         \
    {                                                                                              \
        int v = img_get(ctx, which);                                                               \
        if (!val)                                                                                  \
            return RSS_ERR_INVAL;                                                                  \
        *val = (uint8_t)(v < 0 ? 128 : v > 255 ? 255 : v);                                         \
        return RSS_OK;                                                                             \
    }

NVT_IMG_GETTER_U8(nvt_isp_get_brightness, NVT_IMG_BRIGHTNESS)
NVT_IMG_GETTER_U8(nvt_isp_get_contrast, NVT_IMG_CONTRAST)
NVT_IMG_GETTER_U8(nvt_isp_get_saturation, NVT_IMG_SATURATION)
NVT_IMG_GETTER_U8(nvt_isp_get_sharpness, NVT_IMG_SHARPNESS)
NVT_IMG_GETTER_U8(nvt_isp_get_hue, NVT_IMG_HUE)
NVT_IMG_GETTER_U8(nvt_isp_get_sinter, NVT_IMG_NR)
NVT_IMG_GETTER_U8(nvt_isp_get_temper, NVT_IMG_3DNR)

static int nvt_isp_get_ae_comp(void *ctx, int *val)
{
    int v = img_get(ctx, NVT_IMG_AE_COMP);
    if (!val)
        return RSS_ERR_INVAL;
    *val = v < 0 ? 128 : v;
    return RSS_OK;
}

static int nvt_isp_get_running_mode(void *ctx, rss_isp_mode_t *mode)
{
    if (!mode)
        return RSS_ERR_INVAL;
    *mode = img_get(ctx, NVT_IMG_NIGHT) > 0 ? RSS_ISP_NIGHT : RSS_ISP_DAY;
    return RSS_OK;
}

static int nvt_isp_get_antiflicker(void *ctx, rss_antiflicker_t *mode)
{
    int v = img_get(ctx, NVT_IMG_ANTIFLICKER);
    if (!mode)
        return RSS_ERR_INVAL;
    *mode = v < 0 ? RSS_ANTIFLICKER_OFF : (rss_antiflicker_t)v;
    return RSS_OK;
}

static int nvt_isp_set_hflip(void *vctx, int enable)
{
    rss_hal_ctx_t *ctx = vctx;

    pthread_mutex_lock(&ctx->lock);
    ctx->hflip = enable;
    cap_apply_dir(ctx, false);
    pthread_mutex_unlock(&ctx->lock);
    return RSS_OK;
}

static int nvt_isp_set_vflip(void *vctx, int enable)
{
    rss_hal_ctx_t *ctx = vctx;

    pthread_mutex_lock(&ctx->lock);
    ctx->vflip = enable;
    cap_apply_dir(ctx, false);
    pthread_mutex_unlock(&ctx->lock);
    return RSS_OK;
}

static const rss_hal_ops_t nvt_ops = {
    .init = nvt_init,
    .deinit = nvt_deinit,
    .get_caps = nvt_get_caps,
    .sys_get_timestamp = nvt_sys_get_timestamp,
    .enc_create_channel = nvt_enc_create_channel,
    .enc_destroy_channel = nvt_enc_destroy_channel,
    .enc_start = nvt_enc_start,
    .enc_stop = nvt_enc_stop,
    .enc_poll = nvt_enc_poll,
    .enc_get_frame = nvt_enc_get_frame,
    .enc_release_frame = nvt_enc_release_frame,
    .enc_request_idr = nvt_enc_request_idr,
    .enc_set_rc_mode = nvt_enc_set_rc_mode,
    .enc_set_bitrate = nvt_enc_set_bitrate,
    .enc_set_gop = nvt_enc_set_gop,
    .enc_get_gop_attr = nvt_enc_get_gop,
    .enc_get_channel_attr = nvt_enc_get_channel_attr,
    .enc_get_fps = nvt_enc_get_fps,
    .isp_set_hflip = nvt_isp_set_hflip,
    .isp_set_vflip = nvt_isp_set_vflip,
    .isp_set_brightness = nvt_isp_set_brightness,
    .isp_set_contrast = nvt_isp_set_contrast,
    .isp_set_saturation = nvt_isp_set_saturation,
    .isp_set_sharpness = nvt_isp_set_sharpness,
    .isp_set_hue = nvt_isp_set_hue,
    .isp_set_sinter_strength = nvt_isp_set_sinter,
    .isp_set_temper_strength = nvt_isp_set_temper,
    .isp_set_ae_comp = nvt_isp_set_ae_comp,
    .isp_set_antiflicker = nvt_isp_set_antiflicker,
    .isp_set_running_mode = nvt_isp_set_running_mode,
    .isp_get_brightness = nvt_isp_get_brightness,
    .isp_get_contrast = nvt_isp_get_contrast,
    .isp_get_saturation = nvt_isp_get_saturation,
    .isp_get_sharpness = nvt_isp_get_sharpness,
    .isp_get_hue = nvt_isp_get_hue,
    .isp_get_sinter_strength = nvt_isp_get_sinter,
    .isp_get_temper_strength = nvt_isp_get_temper,
    .isp_get_ae_comp = nvt_isp_get_ae_comp,
    .isp_get_running_mode = nvt_isp_get_running_mode,
    .isp_get_antiflicker = nvt_isp_get_antiflicker,
};

/* ── public API ── */

rss_hal_ctx_t *rss_hal_create_backend(const char *backend)
{
    rss_hal_ctx_t *ctx;

    if (backend && strcmp(backend, "novatek") != 0)
        return NULL;
    ctx = calloc(1, sizeof(*ctx));
    if (!ctx)
        return NULL;
    pthread_mutex_init(&ctx->lock, NULL);
    for (int i = 0; i < NVT_IMG_MAX; i++)
        ctx->img[i] = -1;
    ctx->ops = &nvt_ops;
    ctx->caps.soc_name = NVT_PLATFORM_NAME;
    ctx->caps.sdk_version = "HDAL";
    ctx->caps.max_fs_channels = 0;
    ctx->caps.max_enc_channels = NVT_MAX_CHN;
    ctx->caps.has_h265 = true;
    ctx->caps.has_set_bitrate = true;
    ctx->caps.has_gop_attr = true;
    ctx->caps.has_jpeg = true;
    ctx->caps.max_sensors = 1;
    return ctx;
}

rss_hal_ctx_t *rss_hal_create(void)
{
    return rss_hal_create_backend("novatek");
}

void rss_hal_destroy(rss_hal_ctx_t *ctx)
{
    if (!ctx)
        return;
    pthread_mutex_destroy(&ctx->lock);
    free(ctx);
}

const rss_hal_ops_t *rss_hal_get_ops(rss_hal_ctx_t *ctx)
{
    return ctx ? ctx->ops : NULL;
}

int rss_hal_get_imp_version(char *buf, int size)
{
    if (!buf || size <= 0)
        return -EINVAL;
    snprintf(buf, (size_t)size, "HDAL %06x", HDAL_VERSION);
    return 0;
}

int rss_hal_get_sysutils_version(char *buf, int size)
{
    (void)buf;
    (void)size;
    return -1;
}

const char *rss_hal_get_cpu_info(void)
{
    return NVT_PLATFORM_NAME;
}

const char *rss_hal_get_platform_name(void)
{
    return NVT_PLATFORM_NAME;
}

void rss_hal_check_platform(const char *name)
{
    (void)name;
}
