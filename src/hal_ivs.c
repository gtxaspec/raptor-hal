/*
 * hal_ivs.c -- Raptor HAL IVS (Intelligent Video Surveillance) implementation
 *
 * Implements all IVS vtable functions: group/channel lifecycle,
 * result polling, parameter get/set, and motion detection interface
 * creation/destruction.
 *
 * The IVS API is identical across all Ingenic SoCs (T20-T41).
 * All algo handles, params, and results use void* since the IVS
 * algorithm interface is plugin-based and opaque to HAL consumers.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: MIT
 */

#include "hal_internal.h"

#include <imp/imp_ivs.h>
#include <imp/imp_ivs_move.h>

/*
 * PersonDet SDK types — from ivs_common.h / ivs_inf_personDet.h.
 * Declared here rather than including vendor headers that may not
 * be in the standard SDK include path on all build environments.
 */

typedef struct {
    int x;
    int y;
} IVSPoint;

typedef struct {
    IVSPoint ul; /* upper-left */
    IVSPoint br; /* bottom-right */
} IVSRect;

typedef struct {
    unsigned char *data;
    int width;
    int height;
    int pixfmt;
    int64_t timeStamp;
} IVSFrameInfo;

#define NUM_OF_PERSONS 20
#define IVS_PERSONDET_PERM_MAX_ROI 4

typedef struct {
    IVSRect box;      /* reserved */
    IVSRect show_box; /* detection bounding box */
    float confidence;
} person_info;

typedef struct {
    IVSPoint *p;
    int pcnt;
    uint64_t alarm_last_time;
} persondet_perm_t;

typedef struct {
    bool ptime;
    int skip_num;
    IVSFrameInfo frameInfo;
    int sense;   /* 0-5 (default 4) */
    int detdist; /* 0-4: 6m/8m/10m/11m/13m (default 2) */
    bool enable_move;
    bool enable_perm;
    persondet_perm_t perms[IVS_PERSONDET_PERM_MAX_ROI];
    int permcnt;
} persondet_param_input_t;

typedef struct {
    int count;
    int count_move;
    person_info person[NUM_OF_PERSONS];
    int64_t timeStamp;
} persondet_param_output_t;

/* PersonDet interface — provided by libpersonDet_inf.so (all SoCs) */
extern IMPIVSInterface *PersonDetInterfaceInit(void *param);
extern void PersonDetInterfaceExit(IMPIVSInterface *inf);

/* ================================================================
 * GROUP LIFECYCLE
 * ================================================================ */

int hal_ivs_create_group(void *ctx, int grp)
{
    (void)ctx;
    return IMP_IVS_CreateGroup(grp);
}

int hal_ivs_destroy_group(void *ctx, int grp)
{
    (void)ctx;
    return IMP_IVS_DestroyGroup(grp);
}

/* ================================================================
 * CHANNEL LIFECYCLE
 *
 * IMP_IVS_CreateChn takes an IMPIVSInterface* as the algorithm
 * handler.  The HAL passes this through as void* to keep the
 * public API free of SDK types.
 * ================================================================ */

int hal_ivs_create_channel(void *ctx, int chn, void *algo_handle)
{
    (void)ctx;
    return IMP_IVS_CreateChn(chn, (IMPIVSInterface *)algo_handle);
}

int hal_ivs_destroy_channel(void *ctx, int chn)
{
    (void)ctx;
    return IMP_IVS_DestroyChn(chn);
}

int hal_ivs_register_channel(void *ctx, int grp, int chn)
{
    (void)ctx;
    return IMP_IVS_RegisterChn(grp, chn);
}

int hal_ivs_unregister_channel(void *ctx, int chn)
{
    (void)ctx;
    return IMP_IVS_UnRegisterChn(chn);
}

/* ================================================================
 * START / STOP
 *
 * IMP_IVS_StartRecvPic / IMP_IVS_StopRecvPic control whether a
 * channel actively receives frames for algorithm processing.
 * ================================================================ */

int hal_ivs_start(void *ctx, int chn)
{
    (void)ctx;
    return IMP_IVS_StartRecvPic(chn);
}

int hal_ivs_stop(void *ctx, int chn)
{
    (void)ctx;
    return IMP_IVS_StopRecvPic(chn);
}

/* ================================================================
 * RESULT POLLING / GET / RELEASE
 *
 * The polling/get/release cycle must be strictly paired:
 *   1. IMP_IVS_PollingResult() -- block until result available
 *   2. IMP_IVS_GetResult()    -- retrieve result pointer
 *   3. IMP_IVS_ReleaseResult()-- release result back to SDK
 * ================================================================ */

int hal_ivs_poll_result(void *ctx, int chn, uint32_t timeout_ms)
{
    (void)ctx;
    return IMP_IVS_PollingResult(chn, (int)timeout_ms);
}

int hal_ivs_get_result(void *ctx, int chn, void **result)
{
    (void)ctx;
    if (!result)
        return RSS_ERR_INVAL;
    return IMP_IVS_GetResult(chn, result);
}

int hal_ivs_release_result(void *ctx, int chn, void *result)
{
    (void)ctx;
    return IMP_IVS_ReleaseResult(chn, result);
}

/* ================================================================
 * PARAMETER GET / SET
 *
 * Get/set algorithm parameters on a running channel.
 * The param pointer is algorithm-specific (opaque to HAL).
 * ================================================================ */

int hal_ivs_get_param(void *ctx, int chn, void *param)
{
    (void)ctx;
    if (!param)
        return RSS_ERR_INVAL;
    return IMP_IVS_GetParam(chn, param);
}

int hal_ivs_set_param(void *ctx, int chn, void *param)
{
    (void)ctx;
    if (!param)
        return RSS_ERR_INVAL;
    return IMP_IVS_SetParam(chn, param);
}

/* ================================================================
 * RELEASE DATA
 *
 * IMP_IVS_ReleaseData releases frame data cached by the algorithm's
 * processAsync callback.  Must be assigned as the free_data member
 * of IMPIVSInterface to avoid deadlocks.
 * ================================================================ */

int hal_ivs_release_data(void *ctx, int chn, void *data)
{
    (void)ctx;
    (void)chn;
    if (!data)
        return RSS_ERR_INVAL;
    return IMP_IVS_ReleaseData(data);
}

/* ================================================================
 * MOTION DETECTION INTERFACE
 *
 * IMP_IVS_CreateMoveInterface creates a standard motion detection
 * algorithm interface from an IMP_IVS_MoveParam struct.  The
 * returned IMPIVSInterface* can be passed to ivs_create_channel.
 *
 * IMP_IVS_DestroyMoveInterface frees the interface and its
 * internal resources.  The void return from the SDK function
 * means we always return RSS_OK.
 * ================================================================ */

void *hal_ivs_create_move_interface(void *ctx, void *param)
{
    (void)ctx;
    if (!param)
        return NULL;

    /* Translate RSS wrapper to SDK type */
    const rss_ivs_move_param_t *rp = (const rss_ivs_move_param_t *)param;
    IMP_IVS_MoveParam mp;
    memset(&mp, 0, sizeof(mp));

    for (int i = 0; i < RSS_IVS_MAX_ROI; i++)
        mp.sense[i] = rp->sense[i];
    mp.skipFrameCnt = rp->skip_frame_count;
    mp.frameInfo.width = rp->width;
    mp.frameInfo.height = rp->height;
    mp.roiRectCnt = rp->roi_count;
    for (int i = 0; i < rp->roi_count && i < IMP_IVS_MOVE_MAX_ROI_CNT; i++) {
        mp.roiRect[i].p0.x = rp->roi[i].p0_x;
        mp.roiRect[i].p0.y = rp->roi[i].p0_y;
        mp.roiRect[i].p1.x = rp->roi[i].p1_x;
        mp.roiRect[i].p1.y = rp->roi[i].p1_y;
    }

    HAL_LOG_INFO("IVS move param: %dx%d, roi_count=%d, sense[0]=%d, skip=%d", mp.frameInfo.width,
                 mp.frameInfo.height, mp.roiRectCnt, mp.sense[0], mp.skipFrameCnt);
    HAL_LOG_INFO("IVS roi[0]: (%d,%d)-(%d,%d)", mp.roiRect[0].p0.x, mp.roiRect[0].p0.y,
                 mp.roiRect[0].p1.x, mp.roiRect[0].p1.y);
    HAL_LOG_INFO("IVS sizeof(IMP_IVS_MoveParam)=%zu sizeof(IMPFrameInfo)=%zu",
                 sizeof(IMP_IVS_MoveParam), sizeof(IMPFrameInfo));

    return (void *)IMP_IVS_CreateMoveInterface(&mp);
}

int hal_ivs_destroy_move_interface(void *ctx, void *handle)
{
    (void)ctx;
    if (!handle)
        return RSS_ERR_INVAL;
    IMP_IVS_DestroyMoveInterface((IMPIVSInterface *)handle);
    return RSS_OK;
}

/* ================================================================
 * BASE MOTION DETECTION INTERFACE
 *
 * IMP_IVS_CreateBaseMoveInterface / DestroyBaseMoveInterface are
 * available on all SoCs via imp_ivs_base_move.h.  They provide a
 * simpler motion detection algorithm without ROI support.
 *
 * These are declared in imp_ivs_base_move.h.  Since that header
 * may not be present in all SDK versions, we declare them extern
 * here and let the linker resolve them from libimp.so.
 * ================================================================ */

/* Forward declarations -- present in libimp.so on all SoCs */
extern IMPIVSInterface *IMP_IVS_CreateBaseMoveInterface(void *param);
extern void IMP_IVS_DestroyBaseMoveInterface(IMPIVSInterface *iface);

void *hal_ivs_create_base_move_interface(void *ctx, void *param)
{
    (void)ctx;
    if (!param)
        return NULL;
    return (void *)IMP_IVS_CreateBaseMoveInterface(param);
}

int hal_ivs_destroy_base_move_interface(void *ctx, void *handle)
{
    (void)ctx;
    if (!handle)
        return RSS_ERR_INVAL;
    IMP_IVS_DestroyBaseMoveInterface((IMPIVSInterface *)handle);
    return RSS_OK;
}

/* ================================================================
 * PERSON DETECTION INTERFACE
 *
 * PersonDetInterfaceInit / PersonDetInterfaceExit are provided by
 * libpersonDet_inf.so (T20-T32) or libpersonvehiclepetDet_inf.so
 * (T40/T41/A1).  The HAL translates RSS param types to the SDK's
 * persondet_param_input_t.
 * ================================================================ */

void *hal_ivs_create_persondet_interface(void *ctx, void *param)
{
    (void)ctx;
    if (!param)
        return NULL;

    const rss_ivs_persondet_param_t *rp = (const rss_ivs_persondet_param_t *)param;
    persondet_param_input_t pp;
    memset(&pp, 0, sizeof(pp));

    pp.frameInfo.width = rp->width;
    pp.frameInfo.height = rp->height;
    pp.skip_num = rp->skip_frame_count;
    pp.sense = rp->sensitivity > 5 ? 5 : (rp->sensitivity < 0 ? 0 : rp->sensitivity);
    pp.detdist = rp->det_distance > 4 ? 4 : (rp->det_distance < 0 ? 0 : rp->det_distance);
    pp.enable_move = rp->motion_trigger;
    pp.enable_perm = false;
    pp.permcnt = 0;
    pp.ptime = false;

    HAL_LOG_INFO("IVS persondet param: %dx%d, sense=%d, detdist=%d, skip=%d, move_trig=%d",
                 pp.frameInfo.width, pp.frameInfo.height, pp.sense, pp.detdist, pp.skip_num,
                 pp.enable_move);

    return (void *)PersonDetInterfaceInit(&pp);
}

int hal_ivs_destroy_persondet_interface(void *ctx, void *handle)
{
    (void)ctx;
    if (!handle)
        return RSS_ERR_INVAL;
    PersonDetInterfaceExit((IMPIVSInterface *)handle);
    return RSS_OK;
}
