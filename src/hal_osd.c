/*
 * hal_osd.c -- Raptor HAL OSD implementation
 *
 * Translates rss_osd_region_t to vendor IMPOSDRgnAttr and dispatches
 * IMP_OSD_* calls. Handles the OSD enum value shift between classic
 * (T20/T21/T30/T31) and extended (T23/T32/T40/T41) SDK.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: MIT
 */

#include "hal_internal.h"
#include <errno.h>

/* ================================================================
 * OSD Type Translation
 *
 * The enum symbol OSD_REG_PIC resolves to 5 on classic SDK and 7
 * on extended SDK. We always use the symbol so the compiler picks
 * the correct value. This function maps from the HAL's rss_osd_type_t
 * to the vendor IMPOsdRgnType.
 * ================================================================ */

static IMPOsdRgnType hal_translate_osd_type(rss_osd_type_t type)
{
    switch (type) {
    case RSS_OSD_PIC:
        return OSD_REG_PIC;

    case RSS_OSD_COVER:
        return OSD_REG_COVER;

    case RSS_OSD_PIC_RMEM:
#if defined(PLATFORM_T31) || defined(HAL_EXTENDED_OSD)
        return OSD_REG_PIC_RMEM;
#else
        /* T20/T21/T30: PIC_RMEM not available, fall back to PIC */
        HAL_LOG_WARN("OSD_REG_PIC_RMEM not available, using PIC");
        return OSD_REG_PIC;
#endif

    default:
        HAL_LOG_ERR("unknown OSD type: %d", type);
        return OSD_REG_INV;
    }
}

/* ================================================================
 * Pixel format translation (OSD context)
 * ================================================================ */

static IMPPixelFormat hal_osd_translate_pixfmt(rss_pixfmt_t fmt)
{
    switch (fmt) {
    case RSS_PIXFMT_BGRA:
        return PIX_FMT_BGRA;
    case RSS_PIXFMT_ARGB:
        return PIX_FMT_ARGB;
    default:
        return PIX_FMT_BGRA;
    }
}

/* ================================================================
 * Build IMPOSDRgnAttr from rss_osd_region_t
 *
 * The IMPOSDRgnAttr struct layout differs between classic and
 * extended SDK (extended has additional line, osdispdraw, fontData,
 * mosaicAttr fields). We memset the whole struct to zero so the
 * extra fields are safely initialized.
 * ================================================================ */

static void hal_build_rgn_attr(IMPOSDRgnAttr *attr, const rss_osd_region_t *region)
{
    memset(attr, 0, sizeof(*attr));

    attr->type = hal_translate_osd_type(region->type);
    attr->rect.p0.x = region->x;
    attr->rect.p0.y = region->y;
    attr->rect.p1.x = region->x + region->width - 1;
    attr->rect.p1.y = region->y + region->height - 1;
    attr->fmt = hal_osd_translate_pixfmt(region->bitmap_fmt);

    switch (region->type) {
    case RSS_OSD_PIC:
    case RSS_OSD_PIC_RMEM:
        attr->data.picData.pData = (void *)region->bitmap_data;
        break;

    case RSS_OSD_COVER:
        attr->data.coverData.color = region->cover_color;
        break;

    default:
        break;
    }
}

/* ================================================================
 * hal_osd_set_pool_size
 * ================================================================ */

int hal_osd_set_pool_size(void *ctx, uint32_t bytes)
{
    (void)ctx;
    int ret;

    ret = IMP_OSD_SetPoolSize((int)bytes);
    if (ret != 0)
        HAL_LOG_ERR("IMP_OSD_SetPoolSize(%u) failed: %d", bytes, ret);

    return ret;
}

/* ================================================================
 * hal_osd_create_group
 * ================================================================ */

int hal_osd_create_group(void *ctx, int grp)
{
    (void)ctx;
    int ret;

    ret = IMP_OSD_CreateGroup(grp);
    if (ret != 0)
        HAL_LOG_ERR("IMP_OSD_CreateGroup(%d) failed: %d", grp, ret);

    return ret;
}

/* ================================================================
 * hal_osd_destroy_group
 * ================================================================ */

int hal_osd_destroy_group(void *ctx, int grp)
{
    (void)ctx;
    int ret;

    ret = IMP_OSD_DestroyGroup(grp);
    if (ret != 0)
        HAL_LOG_ERR("IMP_OSD_DestroyGroup(%d) failed: %d", grp, ret);

    return ret;
}

/* ================================================================
 * hal_osd_create_region
 *
 * Build IMPOSDRgnAttr, call CreateRgn to get a handle, then
 * SetRgnAttr to apply the full attributes.
 * ================================================================ */

int hal_osd_create_region(void *ctx, int *handle, const rss_osd_region_t *attr)
{
    (void)ctx;
    IMPRgnHandle h;

    if (!handle || !attr)
        return -EINVAL;

    (void)attr;

    /* SDK requires CreateRgn(NULL). SetRgnAttr must be called AFTER
     * RegisterRgn per vendor sample — caller must call osd_set_region_attr
     * after osd_register_region. */
    h = IMP_OSD_CreateRgn(NULL);
    if (h == INVHANDLE) {
        HAL_LOG_ERR("IMP_OSD_CreateRgn failed");
        return -EIO;
    }

    *handle = (int)h;
    return 0;
}

/* ================================================================
 * hal_osd_destroy_region
 * ================================================================ */

int hal_osd_destroy_region(void *ctx, int handle)
{
    (void)ctx;

    /* IMP_OSD_DestroyRgn returns void on T31 but int on some SDKs.
     * We call it unconditionally and return success. */
    IMP_OSD_DestroyRgn((IMPRgnHandle)handle);

    return 0;
}

/* ================================================================
 * hal_osd_register_region
 * ================================================================ */

int hal_osd_register_region(void *ctx, int handle, int grp)
{
    (void)ctx;
    int ret;

    /* Register with NULL group attributes; caller sets them via
     * osd_show_region or osd_set_region_attr afterward */
    ret = IMP_OSD_RegisterRgn((IMPRgnHandle)handle, grp, NULL);
    if (ret != 0)
        HAL_LOG_ERR("IMP_OSD_RegisterRgn(%d, %d) failed: %d", handle, grp, ret);

    return ret;
}

/* ================================================================
 * hal_osd_unregister_region
 * ================================================================ */

int hal_osd_unregister_region(void *ctx, int handle, int grp)
{
    (void)ctx;
    int ret;

    ret = IMP_OSD_UnRegisterRgn((IMPRgnHandle)handle, grp);
    if (ret != 0)
        HAL_LOG_ERR("IMP_OSD_UnRegisterRgn(%d, %d) failed: %d", handle, grp, ret);

    return ret;
}

/* ================================================================
 * hal_osd_set_region_attr
 *
 * Rebuild IMPOSDRgnAttr from rss_osd_region_t and update the
 * region. Used for changing region type, position, size, or data.
 * ================================================================ */

int hal_osd_set_region_attr(void *ctx, int handle, const rss_osd_region_t *attr)
{
    (void)ctx;
    IMPOSDRgnAttr rgn_attr;
    int ret;

    if (!attr)
        return -EINVAL;

    hal_build_rgn_attr(&rgn_attr, attr);

    ret = IMP_OSD_SetRgnAttr((IMPRgnHandle)handle, &rgn_attr);
    if (ret != 0)
        HAL_LOG_ERR("IMP_OSD_SetRgnAttr(%d) failed: %d", handle, ret);

    return ret;
}

/* ================================================================
 * hal_osd_update_region_data
 *
 * Fast-path update: only changes the bitmap data pointer without
 * rebuilding the full region attributes. Used for timestamp/logo
 * updates where position and size remain constant.
 * ================================================================ */

int hal_osd_update_region_data(void *ctx, int handle, const uint8_t *data)
{
    (void)ctx;
    IMPOSDRgnAttrData attr_data;
    int ret;

    if (!data)
        return -EINVAL;

    /* Do NOT memset — vendor sample leaves struct uninitialized,
     * only sets picData.pData. Zeroing other union members may
     * confuse the SDK's internal state. */
    attr_data.picData.pData = (void *)data;

    ret = IMP_OSD_UpdateRgnAttrData((IMPRgnHandle)handle, &attr_data);
    if (ret != 0)
        HAL_LOG_ERR("IMP_OSD_UpdateRgnAttrData(%d) failed: %d", handle, ret);

    return ret;
}

/* ================================================================
 * hal_osd_show_region
 *
 * Set group region attributes (show/hide flag, alpha, layer) and
 * start the OSD group on first show.
 * ================================================================ */

int hal_osd_show_region(void *ctx, int handle, int grp, int show, int layer)
{
    (void)ctx;
    IMPOSDGrpRgnAttr gattr;
    int ret;

    memset(&gattr, 0, sizeof(gattr));

    gattr.show = show ? 1 : 0;
    gattr.offPos.x = 0;
    gattr.offPos.y = 0;
    gattr.scalex = 1.0f;
    gattr.scaley = 1.0f;
    gattr.gAlphaEn = 1;
    gattr.fgAlhpa = 0xff;
    gattr.bgAlhpa = 0;
    gattr.layer = layer;

    ret = IMP_OSD_SetGrpRgnAttr((IMPRgnHandle)handle, grp, &gattr);
    if (ret != 0) {
        HAL_LOG_ERR("IMP_OSD_SetGrpRgnAttr(%d, %d) failed: %d", handle, grp, ret);
        return ret;
    }

    /* IMP_OSD_Start is called separately by the pipeline init (after
     * System_Bind per SDK spec). Do NOT call it here — starting OSD
     * from show_region can crash the encoder if called at wrong time. */

    return 0;
}

/* ================================================================
 * hal_osd_get_region_attr
 *
 * Retrieve region attributes and reverse-translate to rss_osd_region_t.
 * ================================================================ */

static rss_osd_type_t hal_reverse_osd_type(IMPOsdRgnType type)
{
    if (type == OSD_REG_PIC)
        return RSS_OSD_PIC;
    if (type == OSD_REG_COVER)
        return RSS_OSD_COVER;
#if defined(PLATFORM_T31) || defined(HAL_EXTENDED_OSD)
    if (type == OSD_REG_PIC_RMEM)
        return RSS_OSD_PIC_RMEM;
#endif
    return RSS_OSD_PIC;
}

int hal_osd_get_region_attr(void *ctx, int handle, rss_osd_region_t *attr)
{
    (void)ctx;
    IMPOSDRgnAttr rgn_attr;
    int ret;

    if (!attr)
        return -EINVAL;

    ret = IMP_OSD_GetRgnAttr((IMPRgnHandle)handle, &rgn_attr);
    if (ret != 0) {
        HAL_LOG_ERR("IMP_OSD_GetRgnAttr(%d) failed: %d", handle, ret);
        return ret;
    }

    memset(attr, 0, sizeof(*attr));
    attr->type = hal_reverse_osd_type(rgn_attr.type);
    attr->x = rgn_attr.rect.p0.x;
    attr->y = rgn_attr.rect.p0.y;
    attr->width = rgn_attr.rect.p1.x - rgn_attr.rect.p0.x + 1;
    attr->height = rgn_attr.rect.p1.y - rgn_attr.rect.p0.y + 1;

    if (attr->type == RSS_OSD_PIC || attr->type == RSS_OSD_PIC_RMEM) {
        attr->bitmap_data = (const uint8_t *)rgn_attr.data.picData.pData;
        attr->bitmap_fmt = RSS_PIXFMT_BGRA;
    } else if (attr->type == RSS_OSD_COVER) {
        attr->cover_color = rgn_attr.data.coverData.color;
    }

    return 0;
}

/* ================================================================
 * hal_osd_get_group_region_attr
 * ================================================================ */

int hal_osd_get_group_region_attr(void *ctx, int handle, int grp, rss_osd_region_t *attr)
{
    (void)ctx;
    IMPOSDGrpRgnAttr gattr;
    int ret;

    if (!attr)
        return -EINVAL;

    ret = IMP_OSD_GetGrpRgnAttr((IMPRgnHandle)handle, grp, &gattr);
    if (ret != 0) {
        HAL_LOG_ERR("IMP_OSD_GetGrpRgnAttr(%d, %d) failed: %d", handle, grp, ret);
        return ret;
    }

    /* Fill alpha and layer info into the attr struct */
    attr->global_alpha_en = (gattr.gAlphaEn != 0);
    attr->fg_alpha = gattr.fgAlhpa;
    attr->bg_alpha = gattr.bgAlhpa;
    attr->layer = gattr.layer;

    return 0;
}

/* ================================================================
 * hal_osd_show -- show/hide a region in a group (bool interface)
 * ================================================================ */

int hal_osd_show(void *ctx, int handle, int grp, bool show)
{
    (void)ctx;
    return IMP_OSD_ShowRgn((IMPRgnHandle)handle, grp, show ? 1 : 0);
}

/* ================================================================
 * hal_osd_start / hal_osd_stop
 * ================================================================ */

int hal_osd_start(void *ctx, int grp)
{
    (void)ctx;
    int ret = IMP_OSD_Start(grp);
    if (ret != 0)
        HAL_LOG_ERR("IMP_OSD_Start(%d) failed: %d", grp, ret);
    return ret;
}

int hal_osd_stop(void *ctx, int grp)
{
    (void)ctx;
    int ret = IMP_OSD_Stop(grp);
    if (ret != 0)
        HAL_LOG_ERR("IMP_OSD_Stop(%d) failed: %d", grp, ret);
    return ret;
}

/* ================================================================
 * hal_osd_set_region_attr_with_timestamp
 *
 * Sets region attr and associates a timestamp. On SDKs that don't
 * have a timestamp variant, falls back to regular SetRgnAttr.
 * ================================================================ */

int hal_osd_set_region_attr_with_timestamp(void *ctx, int handle, const rss_osd_region_t *attr,
                                           uint64_t timestamp)
{
    (void)ctx;
    IMPOSDRgnAttr rgn_attr;
    int ret;

    if (!attr)
        return -EINVAL;

    hal_build_rgn_attr(&rgn_attr, attr);

    /*
     * IMP_OSD_SetRgnAttrWithTimestamp is present in libimp.so on
     * all SDK versions.  The timestamp parameter is a pointer to
     * an opaque structure containing the 64-bit timestamp.
     */
    (void)timestamp;
    ret = IMP_OSD_SetRgnAttrWithTimestamp((IMPRgnHandle)handle, &rgn_attr, (void *)&timestamp);
    if (ret != 0)
        HAL_LOG_ERR("IMP_OSD_SetRgnAttrWithTimestamp(%d) failed: %d", handle, ret);

    return ret;
}

/* ================================================================
 * hal_osd_attach_to_group
 *
 * Attaches an OSD region to a group. On some SDKs this is done
 * via IMP_OSD_AttachToGroup, on others via RegisterRgn.
 * The register_region function above already handles basic
 * registration. This function provides the explicit AttachToGroup
 * call which may handle additional binding on some SDK versions.
 * ================================================================ */

int hal_osd_attach_to_group(void *ctx, int handle, int grp)
{
    (void)ctx;
    int ret;

    /*
     * IMP_OSD_AttachToGroup may not be in all SDK header versions.
     * Fall back to RegisterRgn if AttachToGroup is not available.
     * Both achieve the same effect: bind a region handle to a group.
     */
    ret = IMP_OSD_RegisterRgn((IMPRgnHandle)handle, grp, NULL);
    if (ret != 0)
        HAL_LOG_ERR("IMP_OSD_RegisterRgn(%d, %d) [attach] failed: %d", handle, grp, ret);

    return ret;
}

/* ================================================================
 * ISP OSD — hardware overlay in the ISP pipeline
 *
 * Only available on T23/T32/T40/T41 (SoCs with isp_osd.h).
 * Provides OSD burned directly into ISP output, before encoding.
 * ================================================================ */

#if defined(PLATFORM_T23) || defined(PLATFORM_T32) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
#include <imp/isp_osd.h>
#define HAL_HAS_ISP_OSD
#endif

int hal_isp_osd_init(void *ctx)
{
    (void)ctx;
#ifdef HAL_HAS_ISP_OSD
    return IMP_OSD_Init_ISP();
#else
    return RSS_ERR_NOTSUP;
#endif
}

int hal_isp_osd_exit(void *ctx)
{
    (void)ctx;
#ifdef HAL_HAS_ISP_OSD
    IMP_OSD_Exit_ISP();
    return RSS_OK;
#else
    return RSS_ERR_NOTSUP;
#endif
}

int hal_isp_osd_set_pool_size(void *ctx, int size)
{
    (void)ctx;
#ifdef HAL_HAS_ISP_OSD
    return IMP_OSD_SetPoolSize_ISP(size);
#else
    (void)size;
    return RSS_ERR_NOTSUP;
#endif
}

int hal_isp_osd_create_region(void *ctx, int chn, void *attr)
{
    (void)ctx;
#ifdef HAL_HAS_ISP_OSD
    if (!attr)
        return RSS_ERR_INVAL;
    return IMP_OSD_CreateRgn_ISP(chn, (IMPIspOsdAttrAsm *)attr);
#else
    (void)chn;
    (void)attr;
    return RSS_ERR_NOTSUP;
#endif
}

int hal_isp_osd_destroy_region(void *ctx, int chn, int handle)
{
    (void)ctx;
#ifdef HAL_HAS_ISP_OSD
    return IMP_OSD_DestroyRgn_ISP(chn, handle);
#else
    (void)chn;
    (void)handle;
    return RSS_ERR_NOTSUP;
#endif
}

int hal_isp_osd_set_region_attr(void *ctx, int chn, int handle, void *attr)
{
    (void)ctx;
#ifdef HAL_HAS_ISP_OSD
    if (!attr)
        return RSS_ERR_INVAL;
    return IMP_OSD_SetRgnAttr_PicISP(chn, handle, (IMPIspOsdAttrAsm *)attr);
#else
    (void)chn;
    (void)handle;
    (void)attr;
    return RSS_ERR_NOTSUP;
#endif
}

int hal_isp_osd_get_region_attr(void *ctx, int chn, int handle, void *attr)
{
    (void)ctx;
#ifdef HAL_HAS_ISP_OSD
    if (!attr)
        return RSS_ERR_INVAL;
    return IMP_OSD_GetRgnAttr_ISPPic(chn, handle, (IMPIspOsdAttrAsm *)attr);
#else
    (void)chn;
    (void)handle;
    (void)attr;
    return RSS_ERR_NOTSUP;
#endif
}

int hal_isp_osd_show_region(void *ctx, int chn, int handle, int show)
{
    (void)ctx;
#ifdef HAL_HAS_ISP_OSD
    return IMP_OSD_ShowRgn_ISP(chn, handle, show);
#else
    (void)chn;
    (void)handle;
    (void)show;
    return RSS_ERR_NOTSUP;
#endif
}

int hal_isp_osd_update_region_data(void *ctx, int chn, int handle, void *data)
{
    (void)ctx;
    /* UpdateRgnAttrData_ISP exists in .so but not in headers.
     * Use isp_osd_set_region_attr to update data instead. */
    (void)chn;
    (void)handle;
    (void)data;
    return RSS_ERR_NOTSUP;
}
