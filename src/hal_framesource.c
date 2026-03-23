/*
 * hal_framesource.c -- Raptor HAL framesource implementation
 *
 * Translates rss_fs_config_t to vendor IMPFSChnAttr and dispatches
 * IMP_FrameSource_* calls. Handles struct layout differences across
 * T20/T21/T30 (Layout A), T23 (Layout B), T31 (Layout C), and
 * T32/T40/T41 (Layout D).
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: MIT
 */

#include "hal_internal.h"
#include <errno.h>

/* ================================================================
 * Pixel format translation
 * ================================================================ */

static IMPPixelFormat hal_translate_pixfmt(rss_pixfmt_t fmt)
{
    switch (fmt) {
    case RSS_PIXFMT_NV12:
        return PIX_FMT_NV12;
    case RSS_PIXFMT_YUYV422:
        return PIX_FMT_YUYV422;
    default:
        return PIX_FMT_NV12;
    }
}

/* ================================================================
 * hal_fs_create_channel
 *
 * Build an IMPFSChnAttr from rss_fs_config_t and create+set the
 * channel. The struct layout varies per SoC generation.
 * ================================================================ */

int hal_fs_create_channel(void *ctx, int chn, const rss_fs_config_t *cfg)
{
    (void)ctx;
    IMPFSChnAttr attr;
    int ret;

    if (!cfg)
        return -EINVAL;

    memset(&attr, 0, sizeof(attr));

    /* ── I2D attributes (T32/T40/T41: placed at start of struct) ── */
#if defined(PLATFORM_T32) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
    attr.i2dattr.i2d_enable = 0;
    attr.i2dattr.flip_enable = 0;
    attr.i2dattr.mirr_enable = 0;
    attr.i2dattr.rotate_enable = 0;
    attr.i2dattr.rotate_angle = 0;
#endif

    /* ── Core fields (all SoCs) ── */
    attr.picWidth = cfg->width;
    attr.picHeight = cfg->height;
    attr.pixFmt = hal_translate_pixfmt(cfg->pixfmt);
    attr.outFrmRateNum = cfg->fps_num;
    attr.outFrmRateDen = cfg->fps_den;
    attr.nrVBs = (cfg->nr_vbs > 0) ? cfg->nr_vbs : 3;
    attr.type = (IMPFSChnType)cfg->chn_type;

    /* ── ISP crop (all SoCs) ── */
    if (cfg->crop.enable) {
        attr.crop.enable = 1;
        attr.crop.left = cfg->crop.x;
        attr.crop.top = cfg->crop.y;
        attr.crop.width = cfg->crop.w;
        attr.crop.height = cfg->crop.h;
    }

    /* ── Frame crop (T23/T31/T32/T40/T41) ── */
#if defined(HAL_NEW_SDK) || defined(PLATFORM_T23)
    if (cfg->fcrop.enable) {
        attr.fcrop.enable = 1;
        attr.fcrop.left = cfg->fcrop.x;
        attr.fcrop.top = cfg->fcrop.y;
        attr.fcrop.width = cfg->fcrop.w;
        attr.fcrop.height = cfg->fcrop.h;
    }
#endif

    /* ── Mirror enable (T23/T32 have mirr_enable in struct; T40 does not) ── */
#if defined(PLATFORM_T23) || defined(PLATFORM_T32)
    attr.mirr_enable = 0;
#endif

    /* ── Scaler (required when output differs from sensor resolution) ── */
    if (cfg->scaler.enable) {
        attr.scaler.enable = 1;
        attr.scaler.outwidth = cfg->scaler.out_width;
        attr.scaler.outheight = cfg->scaler.out_height;
    } else {
        attr.scaler.enable = 0;
    }

    /* ── Create and set channel ── */
    ret = IMP_FrameSource_CreateChn(chn, &attr);
    if (ret != 0) {
        HAL_LOG_ERR("IMP_FrameSource_CreateChn(%d) failed: %d", chn, ret);
        return ret;
    }

    ret = IMP_FrameSource_SetChnAttr(chn, &attr);
    if (ret != 0) {
        HAL_LOG_ERR("IMP_FrameSource_SetChnAttr(%d) failed: %d", chn, ret);
        IMP_FrameSource_DestroyChn(chn);
        return ret;
    }

    return 0;
}

/* ================================================================
 * hal_fs_destroy_channel
 * ================================================================ */

int hal_fs_destroy_channel(void *ctx, int chn)
{
    (void)ctx;
    int ret;

    ret = IMP_FrameSource_DestroyChn(chn);
    if (ret != 0)
        HAL_LOG_ERR("IMP_FrameSource_DestroyChn(%d) failed: %d", chn, ret);

    return ret;
}

/* ================================================================
 * hal_fs_enable_channel
 * ================================================================ */

int hal_fs_enable_channel(void *ctx, int chn)
{
    (void)ctx;
    int ret;

    ret = IMP_FrameSource_EnableChn(chn);
    if (ret != 0)
        HAL_LOG_ERR("IMP_FrameSource_EnableChn(%d) failed: %d", chn, ret);

    return ret;
}

/* ================================================================
 * hal_fs_disable_channel
 * ================================================================ */

int hal_fs_disable_channel(void *ctx, int chn)
{
    (void)ctx;
    int ret;

    ret = IMP_FrameSource_DisableChn(chn);
    if (ret != 0)
        HAL_LOG_ERR("IMP_FrameSource_DisableChn(%d) failed: %d", chn, ret);

    return ret;
}

/* ================================================================
 * hal_fs_set_rotation
 *
 * T31: software rotation via IMP_FrameSource_SetChnRotate
 * T32/T40/T41: hardware I2D rotation via SetI2dAttr
 * Others: not supported (return 0 if degrees==0, -ENOTSUP otherwise)
 * ================================================================ */

int hal_fs_set_rotation(void *ctx, int chn, int degrees)
{
    (void)ctx;

    if (degrees == 0)
        return 0;

#if defined(PLATFORM_T31)
    {
        uint8_t rot;
        IMPFSChnAttr attr;
        int ret;

        /* Get current channel dimensions for the rotation call */
        ret = IMP_FrameSource_GetChnAttr(chn, &attr);
        if (ret != 0) {
            HAL_LOG_ERR("GetChnAttr(%d) failed for rotation: %d", chn, ret);
            return ret;
        }

        switch (degrees) {
        case 90:
            rot = 1; /* 90 degrees counterclockwise */
            break;
        case 270:
            rot = 2; /* 90 degrees clockwise */
            break;
        default:
            HAL_LOG_ERR("T31 rotation only supports 90/270, got %d", degrees);
            return -EINVAL;
        }

        ret = IMP_FrameSource_SetChnRotate(chn, rot, attr.picWidth, attr.picHeight);
        if (ret != 0) {
            HAL_LOG_ERR("SetChnRotate(%d, %d) failed: %d", chn, rot, ret);
            return ret;
        }

        return 0;
    }

#elif defined(PLATFORM_T32) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
    {
        IMPFSI2DAttr i2d;
        int ret;

        ret = IMP_FrameSource_GetI2dAttr(chn, &i2d);
        if (ret != 0) {
            HAL_LOG_ERR("GetI2dAttr(%d) failed: %d", chn, ret);
            return ret;
        }

        switch (degrees) {
        case 90:
            i2d.i2d_enable = 1;
            i2d.rotate_enable = 1;
            i2d.rotate_angle = 90;
            break;
        case 270:
            i2d.i2d_enable = 1;
            i2d.rotate_enable = 1;
            i2d.rotate_angle = 270;
            break;
        default:
            HAL_LOG_ERR("I2D rotation only supports 90/270, got %d", degrees);
            return -EINVAL;
        }

        ret = IMP_FrameSource_SetI2dAttr(chn, &i2d);
        if (ret != 0) {
            HAL_LOG_ERR("SetI2dAttr(%d) failed: %d", chn, ret);
            return ret;
        }

        return 0;
    }

#else
    /* T20/T21/T23/T30: rotation not supported */
    (void)chn;
    HAL_LOG_WARN("rotation not supported on this SoC (requested %d deg)", degrees);
    return -ENOTSUP;
#endif
}

/* ================================================================
 * hal_fs_set_fifo
 *
 * Configure channel FIFO depth. Same API on all SoCs.
 * ================================================================ */

int hal_fs_set_fifo(void *ctx, int chn, int depth)
{
    (void)ctx;
    IMPFSChnFifoAttr fifo;
    int ret;

    memset(&fifo, 0, sizeof(fifo));
    fifo.maxdepth = depth;
    fifo.type = FIFO_CACHE_PRIORITY;

    ret = IMP_FrameSource_SetChnFifoAttr(chn, &fifo);
    if (ret != 0)
        HAL_LOG_ERR("SetChnFifoAttr(%d) failed: %d", chn, ret);

    return ret;
}

/* ================================================================
 * Reverse pixel format translation
 * ================================================================ */

static rss_pixfmt_t hal_reverse_pixfmt(IMPPixelFormat fmt)
{
    switch (fmt) {
    case PIX_FMT_NV12:
        return RSS_PIXFMT_NV12;
    case PIX_FMT_YUYV422:
        return RSS_PIXFMT_YUYV422;
    default:
        return RSS_PIXFMT_NV12;
    }
}

/* ================================================================
 * hal_fs_get_frame
 *
 * Get a raw frame from the framesource channel. Uses
 * IMP_FrameSource_GetFrame on all SoCs.
 * ================================================================ */

int hal_fs_get_frame(void *ctx, int chn, void **frame_data, rss_frame_info_t *info)
{
    (void)ctx;
    int ret;

    if (!frame_data || !info)
        return -EINVAL;

    IMPFrameInfo *imp_frame = NULL;
    ret = IMP_FrameSource_GetFrame(chn, &imp_frame);
    if (ret != 0 || !imp_frame) {
        HAL_LOG_ERR("IMP_FrameSource_GetFrame(%d) failed: %d", chn, ret);
        return ret;
    }

    info->width = imp_frame->width;
    info->height = imp_frame->height;
    info->pixfmt = hal_reverse_pixfmt(imp_frame->pixfmt);
    info->timestamp = imp_frame->timeStamp;
    info->phys_addr = imp_frame->phyAddr;
    info->virt_addr = (void *)(uintptr_t)imp_frame->virAddr;
    info->size = imp_frame->size;

    *frame_data = imp_frame;
    return 0;
}

/* ================================================================
 * hal_fs_release_frame
 * ================================================================ */

int hal_fs_release_frame(void *ctx, int chn, void *frame_data)
{
    (void)ctx;
    int ret;

    if (!frame_data)
        return -EINVAL;

    IMPFrameInfo *imp_frame = (IMPFrameInfo *)frame_data;
    ret = IMP_FrameSource_ReleaseFrame(chn, imp_frame);
    if (ret != 0)
        HAL_LOG_ERR("IMP_FrameSource_ReleaseFrame(%d) failed: %d", chn, ret);
    /* Don't free imp_frame — it's owned by the SDK */
    return ret;
}

/* ================================================================
 * hal_fs_snap_frame
 *
 * Snap a single frame (one-shot capture). Uses
 * IMP_FrameSource_SnapFrame on all SoCs.
 * ================================================================ */

int hal_fs_snap_frame(void *ctx, int chn, void **frame_data, rss_frame_info_t *info)
{
    (void)ctx;
    int ret;

    if (!frame_data || !info)
        return -EINVAL;

    /* SnapFrame needs format, width, height, and a pre-allocated buffer.
     * We allocate the frame data buffer and let the SDK fill it. */
    uint32_t alloc_size = info->width * info->height * 3 / 2; /* NV12 */
    void *framedata = malloc(alloc_size);
    if (!framedata)
        return -ENOMEM;

    IMPFrameInfo imp_frame;
    memset(&imp_frame, 0, sizeof(imp_frame));
    ret = IMP_FrameSource_SnapFrame(chn, PIX_FMT_NV12, info->width, info->height, framedata,
                                    &imp_frame);
    if (ret != 0) {
        free(framedata);
        HAL_LOG_ERR("IMP_FrameSource_SnapFrame(%d) failed: %d", chn, ret);
        return ret;
    }

    info->pixfmt = RSS_PIXFMT_NV12;
    info->timestamp = imp_frame.timeStamp;
    info->phys_addr = imp_frame.phyAddr;
    info->virt_addr = framedata;
    info->size = alloc_size;

    *frame_data = framedata;
    return 0;
}

/* ================================================================
 * hal_fs_set_frame_depth / hal_fs_get_frame_depth
 *
 * Control the frame buffer depth for a channel.
 * Uses SetFrameDepth/GetFrameDepth on all SoCs.
 * ================================================================ */

int hal_fs_set_frame_depth(void *ctx, int chn, int depth)
{
    (void)ctx;
    int ret = IMP_FrameSource_SetFrameDepth(chn, depth);
    if (ret != 0)
        HAL_LOG_ERR("SetFrameDepth(%d, %d) failed: %d", chn, depth, ret);
    return ret;
}

int hal_fs_get_frame_depth(void *ctx, int chn, int *depth)
{
    (void)ctx;
    if (!depth)
        return -EINVAL;

    int ret = IMP_FrameSource_GetFrameDepth(chn, depth);
    if (ret != 0)
        HAL_LOG_ERR("GetFrameDepth(%d) failed: %d", chn, ret);
    return ret;
}

/* ================================================================
 * hal_fs_get_fifo
 *
 * Retrieve current FIFO depth for a channel.
 * ================================================================ */

int hal_fs_get_fifo(void *ctx, int chn, int *depth)
{
    (void)ctx;
    IMPFSChnFifoAttr fifo;
    int ret;

    if (!depth)
        return -EINVAL;

    memset(&fifo, 0, sizeof(fifo));
    ret = IMP_FrameSource_GetChnFifoAttr(chn, &fifo);
    if (ret != 0) {
        HAL_LOG_ERR("GetChnFifoAttr(%d) failed: %d", chn, ret);
        return ret;
    }

    *depth = fifo.maxdepth;
    return 0;
}

/* ================================================================
 * hal_fs_set_delay / hal_fs_get_delay
 *
 * Set/get channel delay in milliseconds.
 * Not all SoCs support SetDelay; return -ENOTSUP where missing.
 * ================================================================ */

int hal_fs_set_delay(void *ctx, int chn, int delay_ms)
{
    (void)ctx;
#if defined(HAL_NEW_SDK)
    int ret = IMP_FrameSource_SetDelay(chn, delay_ms);
    if (ret != 0)
        HAL_LOG_ERR("SetDelay(%d, %d) failed: %d", chn, delay_ms, ret);
    return ret;
#else
    (void)chn;
    (void)delay_ms;
    return -ENOTSUP;
#endif
}

int hal_fs_get_delay(void *ctx, int chn, int *delay_ms)
{
    (void)ctx;
    if (!delay_ms)
        return -EINVAL;
#if defined(HAL_NEW_SDK)
    int ret = IMP_FrameSource_GetDelay(chn, delay_ms);
    if (ret != 0)
        HAL_LOG_ERR("GetDelay(%d) failed: %d", chn, ret);
    return ret;
#else
    (void)chn;
    return -ENOTSUP;
#endif
}

/* ================================================================
 * hal_fs_set_max_delay / hal_fs_get_max_delay
 * ================================================================ */

int hal_fs_set_max_delay(void *ctx, int chn, int max_delay_ms)
{
    (void)ctx;
#if defined(HAL_NEW_SDK)
    int ret = IMP_FrameSource_SetMaxDelay(chn, max_delay_ms);
    if (ret != 0)
        HAL_LOG_ERR("SetMaxDelay(%d, %d) failed: %d", chn, max_delay_ms, ret);
    return ret;
#else
    (void)chn;
    (void)max_delay_ms;
    return -ENOTSUP;
#endif
}

int hal_fs_get_max_delay(void *ctx, int chn, int *max_delay_ms)
{
    (void)ctx;
    if (!max_delay_ms)
        return -EINVAL;
#if defined(HAL_NEW_SDK)
    int ret = IMP_FrameSource_GetMaxDelay(chn, max_delay_ms);
    if (ret != 0)
        HAL_LOG_ERR("GetMaxDelay(%d) failed: %d", chn, ret);
    return ret;
#else
    (void)chn;
    return -ENOTSUP;
#endif
}

/* ================================================================
 * hal_fs_set_pool / hal_fs_get_pool
 *
 * Set/get the memory pool assignment for a channel.
 * Only supported on new SDK (T31+).
 * ================================================================ */

int hal_fs_set_pool(void *ctx, int chn, int pool_id)
{
    (void)ctx;
#if defined(HAL_NEW_SDK) && !defined(PLATFORM_T40)
    int ret = IMP_FrameSource_SetPool(chn, pool_id);
    if (ret != 0)
        HAL_LOG_ERR("SetPool(%d, %d) failed: %d", chn, pool_id, ret);
    return ret;
#else
    (void)chn;
    (void)pool_id;
    return -ENOTSUP;
#endif
}

int hal_fs_get_pool(void *ctx, int chn, int *pool_id)
{
    (void)ctx;
    if (!pool_id)
        return -EINVAL;
#if defined(HAL_NEW_SDK) && !defined(PLATFORM_T40)
    /* GetPool returns the pool ID directly (not via output param) */
    int ret = IMP_FrameSource_GetPool(chn);
    if (ret < 0) {
        HAL_LOG_ERR("GetPool(%d) failed: %d", chn, ret);
        return ret;
    }
    *pool_id = ret;
    return 0;
#else
    (void)chn;
    return -ENOTSUP;
#endif
}

/* ================================================================
 * hal_fs_get_timed_frame
 *
 * Get a frame with specified timestamp.
 * IMP_FrameSource_GetTimedFrame(chn, framets, block, framedata, frame)
 * All SoCs (that have imp_framesource.h with this function).
 * ================================================================ */

int hal_fs_get_timed_frame(void *ctx, int chn, void *framets, int block, void *framedata,
                           void *frame)
{
    (void)ctx;

    if (!framets || !frame)
        return -EINVAL;

    return IMP_FrameSource_GetTimedFrame(chn, (IMPFrameTimestamp *)framets, block, framedata,
                                         (IMPFrameInfo *)frame);
}

/* ================================================================
 * hal_fs_set_frame_offset
 *
 * Set frame offset for a channel. Not in T31 SDK headers but
 * requested as a passthrough. Declare extern and attempt link.
 * If the symbol doesn't exist for a given SoC, return -ENOTSUP.
 * ================================================================ */

int hal_fs_set_frame_offset(void *ctx, int chn, int offset)
{
    (void)ctx;
    (void)chn;
    (void)offset;
    /* Not found in any vendor header -- return not supported */
    return -ENOTSUP;
}

/* ================================================================
 * hal_fs_chn_stat_query
 *
 * Query the channel state (closed / open / running).
 * IMP_FrameSource_ChnStatQuery(chn, IMPFSChannelState *)
 * T31 only.
 * ================================================================ */

int hal_fs_chn_stat_query(void *ctx, int chn, void *stat)
{
    (void)ctx;
    if (!stat)
        return -EINVAL;
#if defined(PLATFORM_T31)
    return IMP_FrameSource_ChnStatQuery(chn, (IMPFSChannelState *)stat);
#else
    (void)chn;
    return -ENOTSUP;
#endif
}

/* ================================================================
 * hal_fs_enable_chn_undistort / hal_fs_disable_chn_undistort
 *
 * Not found in any vendor headers. Provide stubs that return
 * -ENOTSUP until the vendor symbols are confirmed available.
 * ================================================================ */

int hal_fs_enable_chn_undistort(void *ctx, int chn)
{
    (void)ctx;
    (void)chn;
    return -ENOTSUP;
}

int hal_fs_disable_chn_undistort(void *ctx, int chn)
{
    (void)ctx;
    (void)chn;
    return -ENOTSUP;
}
