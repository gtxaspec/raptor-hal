/*
 * hal_ivs_uranus.cpp -- YOLOv5 detection on the NNA (T41) through
 * Ingenic's Uranus runtime (Magik 3.x, InferenceKit/uranus/nna2)
 *
 * The same C API as hal_ivs_jzdl.cpp (hal_jzdl_create/detect/destroy),
 * so RVD's standalone inference thread drives either; a build has one.
 *
 * Per frame: the NV12 frame is copied into accelerator memory (nmem), the
 * AIP resizes and converts it into the model's input with a letterbox
 * (YOLOv5's own preprocessing: aspect kept, padded with 114), the network
 * runs on the NNA, and the head tensors are decoded on the CPU by the
 * shared YOLO decode (hal_ivs_yolo.c), one head at a time; the boxes come
 * back through the letterbox into frame pixels.
 *
 * The model is a .mgk compiled by the Magik converter for the T41 with its
 * head convolutions as outputs (one to three, at strides 8, 16 and 32, each
 * anchors x (5 + classes) channels) and a 4-channel input (COLOR RGB).
 * Thresholds per class and anchors per stride come from the caller
 * (param->class_threshold, param->anchors), as for the other backends; a
 * stride the caller leaves at 0 gets YOLOv5's stock anchors.
 * The runtime needs nmem= on the kernel command line, and on a T41 the
 * L2 cache's share of the on-chip SRAM at 128 KiB (CCU MSCR) for soc-nna
 * to map the ORAM where it expects it.
 *
 * Every runtime call runs on core 0: a T41's core 1 takes Reserved
 * Instruction on the NN instructions (the vendor samples pin the process
 * to core 0 for the same reason). create and destroy move the calling
 * thread there for their span; detect pins its thread, rvd's inference
 * thread, to core 0 for good.
 *
 * The first create also pins the ISP firmware thread (isp_fw_process) to
 * core 0: NN work on core 0 beside the ISP firmware's MXU3 code on core 1
 * can deadlock both cores until the watchdog resets the SoC (see
 * hal_ivs_nna_pin.h). The pin stays after destroy.
 *
 * Requires: liburanus_core, liburanus_tensor, liburanus_extend, libaip,
 * libdrivers (uclibc build) and libstdc++.
 */

extern "C" {
#include "hal_internal.h"
}

#include "hal_ivs_nna_pin.h"
#include "hal_ivs_yolo.h"

#include "ingenic_aip.h"
#include "ingenic_mnni.h"
#include "utils/uranus_imgproc.h"

#include <sched.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <new>
#include <vector>

namespace mu = magik::uranus;

/* Report the time per stage every this many inferences. */
#define URANUS_STATS_EVERY 100

struct uranus_ctx {
    mu::Network *net;
    mu::Tensor *frame; /* NV12 frame in nmem, the AIP's source */
    int input_w, input_h, input_c;
    int frame_w, frame_h;
    /* the AIP's SYMMETRY pad is YOLOv5's letterbox: model = frame * scale + pad */
    float lb_scale, lb_pad_x, lb_pad_y;
    int n_out; /* head outputs, 1 to 3 */
    hal_yolo_t yolo;
    /* stage times since the last report, microseconds */
    long runs;
    int64_t t_copy, t_resize, t_net, t_post;
    long pinned_tid; /* the thread detect last pinned to core 0 */
};

static int pin_core0(void)
{
    cpu_set_t m;
    CPU_ZERO(&m);
    CPU_SET(0, &m);
    if (sched_setaffinity(0, sizeof(m), &m) != 0) {
        HAL_LOG_ERR("NNA: cannot move the thread to core 0");
        return -1;
    }
    return 0;
}

/* Core 0 for the lifetime of the object, then the thread's own set back. */
struct core0_scope {
    cpu_set_t old;
    bool saved;
    int ok;
    core0_scope()
    {
        saved = sched_getaffinity(0, sizeof(old), &old) == 0;
        ok = pin_core0();
    }
    ~core0_scope()
    {
        if (saved)
            sched_setaffinity(0, sizeof(old), &old);
    }
};

static int64_t now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

/* Contexts using the runtime: it is brought up by the first and down by the last. */
static int runtime_users;

static int runtime_init(void)
{
    if (runtime_users > 0) {
        runtime_users++;
        return 0;
    }
    pid_t isp = nna_pin_isp_core0();
    if (isp > 0)
        HAL_LOG_INFO("NNA: isp_fw_process (%d) pinned to core 0", (int)isp);
    else if (isp == 0)
        HAL_LOG_WARN("NNA: no isp_fw_process thread to pin to core 0");
    else
        HAL_LOG_WARN("NNA: cannot pin isp_fw_process to core 0");
    if (ingenic_aip_init() != 0) {
        HAL_LOG_ERR("NNA: AIP init failed");
        return -1;
    }
    if (mu::uranus_init() != 0) {
        HAL_LOG_ERR("NNA: uranus_init failed (is nmem= on the kernel command line?)");
        ingenic_aip_deinit();
        return -1;
    }
    runtime_users = 1;
    return 0;
}

static void runtime_deinit(void)
{
    if (runtime_users <= 0 || --runtime_users > 0)
        return;
    mu::uranus_deinit();
    ingenic_aip_deinit();
}

static void ctx_free(uranus_ctx *ctx)
{
    if (ctx->frame) {
        ctx->frame->free_data();
        delete ctx->frame;
    }
    if (ctx->net)
        mu::magik_destroy_model(ctx->net);
    delete ctx;
}

extern "C" void *hal_jzdl_create(const rss_ivs_jzdl_param_t *param)
{
    if (!param)
        return NULL;

    if (strnlen(param->model_path, sizeof(param->model_path)) >= sizeof(param->model_path)) {
        HAL_LOG_ERR("NNA: model_path not null-terminated");
        return NULL;
    }
    if (param->width <= 0 || param->height <= 0 || param->width > 8192 || param->height > 8192 ||
        (param->width & 1) || (param->height & 1)) {
        HAL_LOG_ERR("NNA: invalid frame dimensions %dx%d", param->width, param->height);
        return NULL;
    }
    if (param->num_classes <= 0) {
        HAL_LOG_ERR("NNA: invalid class count %d", param->num_classes);
        return NULL;
    }

    core0_scope core0;
    if (core0.ok != 0 || runtime_init() != 0)
        return NULL;

    uranus_ctx *ctx = new (std::nothrow) uranus_ctx();
    if (!ctx) {
        runtime_deinit();
        return NULL;
    }

    if (mu::magik_load_model(ctx->net, param->model_path) != 0 || !ctx->net) {
        HAL_LOG_ERR("NNA: failed to load model: %s", param->model_path);
        ctx->net = NULL;
        ctx_free(ctx);
        runtime_deinit();
        return NULL;
    }

    mu::shape_t s = ctx->net->get_input(0).shape();
    if (s.size() != 4 || s[3] != 4) {
        HAL_LOG_ERR("NNA: model input is not 1xHxWx4 (%zu dims, last %d)", s.size(),
                    s.empty() ? -1 : s.back());
        ctx_free(ctx);
        runtime_deinit();
        return NULL;
    }
    ctx->input_h = s[1];
    ctx->input_w = s[2];
    ctx->input_c = s[3];

    ctx->frame_w = param->width;
    ctx->frame_h = param->height;
    mu::shape_t fs = {1, param->height, param->width, 1};
    ctx->frame = new (std::nothrow) mu::Tensor(fs, mu::DataType::UINT8, mu::DataFormat::NV12);
    if (!ctx->frame || !ctx->frame->mudata<uint8_t>()) {
        HAL_LOG_ERR("NNA: no nmem for a %dx%d frame", param->width, param->height);
        ctx_free(ctx);
        runtime_deinit();
        return NULL;
    }

    ctx->lb_scale =
        std::min((float)ctx->input_w / ctx->frame_w, (float)ctx->input_h / ctx->frame_h);
    ctx->lb_pad_x = (ctx->input_w - ctx->frame_w * ctx->lb_scale) / 2;
    ctx->lb_pad_y = (ctx->input_h - ctx->frame_h * ctx->lb_scale) / 2;

    ctx->n_out = (int)ctx->net->get_output_names().size();
    if (ctx->n_out < 1 || ctx->n_out > HAL_YOLO_MAX_HEADS) {
        HAL_LOG_ERR("NNA: %d outputs (1 to %d YOLO heads supported)", ctx->n_out,
                    HAL_YOLO_MAX_HEADS);
        ctx_free(ctx);
        runtime_deinit();
        return NULL;
    }

    hal_yolo_init(&ctx->yolo, param, ctx->input_w, ctx->input_h);

    HAL_LOG_INFO("NNA: model loaded: %s (input %dx%dx%d, %d outputs, forward memory %lld)",
                 param->model_path, ctx->input_w, ctx->input_h, ctx->input_c, ctx->n_out,
                 (long long)ctx->net->get_forward_memory_size());
    HAL_LOG_INFO("NNA: ready (%dx%d -> %dx%d, %d classes, conf=%.2f, nms=%.2f)", ctx->frame_w,
                 ctx->frame_h, ctx->input_w, ctx->input_h, param->num_classes,
                 (double)param->conf_threshold, (double)param->nms_threshold);
    return ctx;
}

extern "C" int hal_jzdl_detect(void *handle, const uint8_t *nv12_data,
                               rss_ivs_detect_result_t *result)
{
    if (!handle || !nv12_data || !result)
        return RSS_ERR_INVAL;

    uranus_ctx *ctx = (uranus_ctx *)handle;
    long tid = syscall(SYS_gettid);
    if (ctx->pinned_tid != tid) {
        if (pin_core0() != 0)
            return RSS_ERR;
        ctx->pinned_tid = tid;
    }
    result->count = 0;
    result->timestamp = 0; /* caller sets this */
    int64_t t0 = now_us();

    /* Into nmem: the AIP reads accelerator memory by its virtual address. */
    uint8_t *fdata = ctx->frame->mudata<uint8_t>();
    memcpy(fdata, nv12_data, (size_t)ctx->frame_w * ctx->frame_h * 3 / 2);
    int64_t t1 = now_us();

    mu::BsCommonParam bp;
    bp.pad_val = 114;
    bp.pad_type = mu::BsPadType::SYMMETRY;
    bp.in_layout = mu::ChannelLayout::NV12;
    bp.out_layout = mu::ChannelLayout::RGBA;
    bp.input_height = ctx->frame_h;
    bp.input_width = ctx->frame_w;
    bp.input_line_stride = ctx->frame_w;
    bp.addr_attr.vir_addr = fdata;
    std::vector<mu::Box_t> roi(1, mu::Box_t{0, 0, (float)ctx->frame_w, (float)ctx->frame_h});
    /* Straight into the network's input tensor: no copy after the resize. */
    std::vector<mu::Tensor> dst(1, ctx->net->get_input(0));
    if (mu::crop_common_resize(dst, roi, mu::AddressLocate::NMEM_VIRTUAL, &bp) != 0) {
        HAL_LOG_ERR("NNA: AIP resize failed");
        return RSS_ERR;
    }
    int64_t t2 = now_us();

    if (ctx->net->run() != 0) {
        HAL_LOG_ERR("NNA: network run failed");
        return RSS_ERR;
    }
    int64_t t3 = now_us();

    hal_yolo_begin(&ctx->yolo);
    for (int i = 0; i < ctx->n_out; i++) {
        const mu::Tensor out = ctx->net->get_output(i);
        mu::shape_t s = out.shape();
        if (s.size() != 4 || out.data_type() != mu::DataType::FP32 ||
            hal_yolo_decode_head(&ctx->yolo, out.vdata<float>(), s[1], s[2], s[3]) != RSS_OK) {
            HAL_LOG_ERR("NNA: output %d is not a YOLO head for %d classes", i,
                        ctx->yolo.num_classes);
            return RSS_ERR;
        }
    }
    /* model = frame * scale + pad, so frame = (model - pad) / scale */
    hal_yolo_emit(&ctx->yolo, hal_yolo_finish(&ctx->yolo), 1.0f / ctx->lb_scale,
                  1.0f / ctx->lb_scale, ctx->lb_pad_x, ctx->lb_pad_y, ctx->frame_w, ctx->frame_h,
                  result);
    int64_t t4 = now_us();

    ctx->t_copy += t1 - t0;
    ctx->t_resize += t2 - t1;
    ctx->t_net += t3 - t2;
    ctx->t_post += t4 - t3;
    if (++ctx->runs == URANUS_STATS_EVERY) {
        HAL_LOG_INFO("NNA: %d runs, mean ms: copy %.2f, resize %.2f, net %.2f, decode %.2f",
                     URANUS_STATS_EVERY, ctx->t_copy / 1000.0 / URANUS_STATS_EVERY,
                     ctx->t_resize / 1000.0 / URANUS_STATS_EVERY,
                     ctx->t_net / 1000.0 / URANUS_STATS_EVERY,
                     ctx->t_post / 1000.0 / URANUS_STATS_EVERY);
        ctx->runs = 0;
        ctx->t_copy = ctx->t_resize = ctx->t_net = ctx->t_post = 0;
    }

    return RSS_OK;
}

extern "C" void hal_jzdl_destroy(void *handle)
{
    if (!handle)
        return;

    core0_scope core0;
    ctx_free((uranus_ctx *)handle);
    runtime_deinit();

    HAL_LOG_INFO("NNA: destroyed");
}
