/*
 * hal_internal.h -- Raptor HAL internal header
 *
 * Included by all HAL .c files. Provides SDK generation macros,
 * vendor SDK includes, type compatibility shims, logging, error
 * handling, and the opaque rss_hal_ctx definition.
 *
 * NOT part of the public API -- never include from RSS daemons.
 */

#ifndef HAL_INTERNAL_H
#define HAL_INTERNAL_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>

#include "raptor_hal.h"

/* ═══════════════════════════════════════════════════════════════════════
 * 1. SDK Generation Macros
 *
 * The build system defines exactly one PLATFORM_* macro per target.
 * These derived macros group SoCs by API compatibility.
 * ═══════════════════════════════════════════════════════════════════════ */

/* Old-style encoder structs, per-codec RC unions, packs with direct virAddr */
#if defined(PLATFORM_T20) || defined(PLATFORM_T21) || defined(PLATFORM_T23) || defined(PLATFORM_T30)
#define HAL_OLD_SDK
#endif

/* New-style encoder structs, unified RC, packs with offset into ring buffer */
#if defined(PLATFORM_T31) || defined(PLATFORM_T32) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
#define HAL_NEW_SDK
#endif

/* ISP functions take IMPVI_NUM as first parameter */
#if defined(PLATFORM_T40) || defined(PLATFORM_T41)
#define HAL_IMPVI_SDK
#endif

/* T32 is a hybrid: new-style encoder internals but old-style type names */
#if defined(PLATFORM_T32)
#define HAL_HYBRID_SDK
#endif

/* ISP tuning functions that take pointer args (not scalar) */
#if defined(PLATFORM_T32) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
#define HAL_ISP_PTR_ARGS
#endif

/* Extended OSD region types (enum values shifted) */
#if defined(PLATFORM_T23) || defined(PLATFORM_T32) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
#define HAL_EXTENDED_OSD
#endif

/* Multi-sensor support via IMPVI_NUM parameter */
#if defined(PLATFORM_T32) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
#define HAL_MULTI_SENSOR
#endif

/* T23 SDK 1.3.0 MultiCamera API (IMP_ISP_MultiCamera_Tuning_*) */
#if defined(PLATFORM_T23)
#define HAL_T23_MULTICAM
#endif

/* ═══════════════════════════════════════════════════════════════════════
 * 2. Stub Types for Missing ISP Structs
 *
 * Some ISP attribute structs are referenced in headers but not defined
 * on certain SoCs. These zero-size stubs must be defined BEFORE
 * including SDK headers.
 * ═══════════════════════════════════════════════════════════════════════ */

#if defined(PLATFORM_T20) || defined(PLATFORM_T21) || defined(PLATFORM_T30) ||                     \
    defined(PLATFORM_T40) || defined(PLATFORM_T41)
struct IMPISPAEAttr {
}; /* not defined on these SoCs */
#endif

#if defined(PLATFORM_T40) || defined(PLATFORM_T41)
struct IMPISPEVAttr {
}; /* not defined on T40/T41 */
#endif

/* ═══════════════════════════════════════════════════════════════════════
 * 3. Vendor SDK Includes
 * ═══════════════════════════════════════════════════════════════════════ */

#include <imp/imp_system.h>
#include <imp/imp_common.h>
#include <imp/imp_framesource.h>
#include <imp/imp_encoder.h>
#include <imp/imp_isp.h>
#include <imp/imp_audio.h>
#include <imp/imp_osd.h>
#include <sysutils/su_base.h>

/* ═══════════════════════════════════════════════════════════════════════
 * 4. Type Name Normalization
 *
 * New SDK uses mixed-case names (ChnAttr vs CHNAttr). The HAL
 * consistently uses old-style capitalization; these defines handle
 * the translation.
 * ═══════════════════════════════════════════════════════════════════════ */

#if defined(HAL_NEW_SDK) && !defined(PLATFORM_T32)
#define IMPEncoderCHNAttr IMPEncoderChnAttr
#define IMPEncoderCHNStat IMPEncoderChnStat
#endif

/* T32 is a hybrid: new-style SetDefaultParam/Profile, but old-style
 * RC mode enum names (ENC_RC_MODE_* not IMP_ENC_RC_MODE_*) and old-style
 * struct layout (no gopAttr, old RC attr union members).  Define
 * compatibility macros so the new-SDK code path can use IMP_ENC_RC_MODE_*. */
#if defined(PLATFORM_T32)
#define IMP_ENC_RC_MODE_FIXQP ENC_RC_MODE_FIXQP
#define IMP_ENC_RC_MODE_CBR ENC_RC_MODE_CBR
#define IMP_ENC_RC_MODE_VBR ENC_RC_MODE_VBR
#define IMP_ENC_RC_MODE_SMART ENC_RC_MODE_SMART
#define IMP_ENC_RC_MODE_CAPPED_VBR ENC_RC_MODE_CVBR
#define IMP_ENC_RC_MODE_CAPPED_QUALITY ENC_RC_MODE_AVBR
#define IMP_ENC_GOP_CTRL_MODE_DEFAULT 0
#endif

/* T40/T41 use IMPISPCoefftWb (mixed case), older SDKs use IMPISPCOEFFTWB.
 * Normalize so hal_isp.c can use IMPISPCOEFFTWB everywhere. */
#if defined(PLATFORM_T40) || defined(PLATFORM_T41)
#define IMPISPCOEFFTWB IMPISPCoefftWb
#endif

/* T32/T40/T41 use IMPISPAEExprInfo (capital AE), not IMPISPAeExprInfo.
 * Define the name used in hal_isp.c for consistency. */
#if defined(PLATFORM_T32) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
#define IMPISPAeExprInfo IMPISPAEExprInfo
#endif

/* T32/T41 use IMPISPSensorFps struct for GetSensorFPS.
 * T40 uses separate uint32_t* pointers. Handled per-platform in hal_isp.c. */

/* T32/T41 use IMPISPSensorRegister struct for Set/GetSensorRegister.
 * T40 uses separate uint32_t* pointers. Handled per-platform in hal_isp.c. */

/* ═══════════════════════════════════════════════════════════════════════
 * 5. Extern Declarations for Symbols in .so but Missing from Headers
 * ═══════════════════════════════════════════════════════════════════════ */

#ifdef __cplusplus
extern "C" {
#endif

/* IMP_OSD_SetPoolSize is present in libimp.so on all SoCs
 * but missing from some SDK header versions */
int IMP_OSD_SetPoolSize(int size);

/* IMP_ISP_Tuning_GetAwbHist is present in libimp.so on T20-T31
 * but the header prototype is absent on some versions */
#if defined(PLATFORM_T20) || defined(PLATFORM_T21) || defined(PLATFORM_T23) ||                     \
    defined(PLATFORM_T30) || defined(PLATFORM_T31)
int IMP_ISP_Tuning_GetAwbHist(IMPISPAWBHist *awb_hist);
#endif

/* T41: IMP_Encoder_SetChnAttrRcMode exists in libimp.so but is missing
 * from the 1.2.5 header (only Get is declared). */
#if defined(PLATFORM_T41)
int IMP_Encoder_SetChnAttrRcMode(int encChn, const IMPEncoderAttrRcMode *pstRcModeCfg);
#endif

/* Note: IMP_Alloc/Free, IMP_Pool*, IMP_System_* and IMP_OSD_SetRgnAttrWithTimestamp
 * are already declared in the vendor SDK headers included above.
 * Do NOT redeclare them here — it causes conflicting type errors. */

#ifdef __cplusplus
}
#endif

/* ═══════════════════════════════════════════════════════════════════════
 * 6. Logging
 *
 * Calls through a function pointer (default: fprintf to stderr).
 * Daemons call rss_hal_set_log_func() at init to redirect to syslog.
 * ═══════════════════════════════════════════════════════════════════════ */

/* Log levels matching RSS: 0=fatal, 1=error, 2=warn, 3=info, 4=debug */
#define HAL_LOG_LVL_ERR 1
#define HAL_LOG_LVL_WARN 2
#define HAL_LOG_LVL_INFO 3
#define HAL_LOG_LVL_DBG 4

extern rss_hal_log_func_t rss_hal_log_fn;

#define HAL_LOG_ERR(fmt, ...)                                                                      \
    rss_hal_log_fn(HAL_LOG_LVL_ERR, __FILE__, __LINE__, fmt, ##__VA_ARGS__)
#define HAL_LOG_WARN(fmt, ...)                                                                     \
    rss_hal_log_fn(HAL_LOG_LVL_WARN, __FILE__, __LINE__, fmt, ##__VA_ARGS__)
#define HAL_LOG_INFO(fmt, ...)                                                                     \
    rss_hal_log_fn(HAL_LOG_LVL_INFO, __FILE__, __LINE__, fmt, ##__VA_ARGS__)

#ifdef HAL_DEBUG
#define HAL_LOG_DBG(fmt, ...)                                                                      \
    rss_hal_log_fn(HAL_LOG_LVL_DBG, __FILE__, __LINE__, fmt, ##__VA_ARGS__)
#else
#define HAL_LOG_DBG(fmt, ...) ((void)0)
#endif

/* ═══════════════════════════════════════════════════════════════════════
 * 7. Error Check Macro
 *
 * Calls an SDK function, logs on failure, and jumps to a cleanup label.
 * ═══════════════════════════════════════════════════════════════════════ */

/* Requires an 'int ret' in the calling scope — sets it on failure
 * so the cleanup path can propagate the error to the caller. */
#define HAL_CHECK(call, label)                                                                     \
    do {                                                                                           \
        int hal_rc = (call);                                                                       \
        if (hal_rc != 0) {                                                                         \
            HAL_LOG_ERR("%s failed: %d", #call, hal_rc);                                           \
            ret = hal_rc;                                                                          \
            goto label;                                                                            \
        }                                                                                          \
    } while (0)

/* ═══════════════════════════════════════════════════════════════════════
 * 8. HAL Context -- Internal Definition
 *
 * This is the concrete definition of the opaque rss_hal_ctx_t declared
 * in raptor_hal.h. Only HAL .c files see the full struct layout.
 * ═══════════════════════════════════════════════════════════════════════ */

#ifndef RSS_MAX_ENC_CHANNELS
#define RSS_MAX_ENC_CHANNELS 8
#endif

struct rss_hal_ctx {
    const rss_hal_ops_t *ops;
    rss_hal_caps_t caps;

    /* OSD pool size — set via osd_set_pool_size before init */
    uint32_t osd_pool_size;

    /* Multi-sensor state */
    int sensor_count;
    rss_sensor_config_t sensors[RSS_MAX_SENSORS];
    IMPSensorInfo imp_sensors[RSS_MAX_SENSORS];

    /* Flip state per sensor (needed for SoCs with combined H/V flip) */
    int hflip_state[RSS_MAX_SENSORS];
    int vflip_state[RSS_MAX_SENSORS];

    /* Full multi-sensor config (stored for deinit ordering) */
    rss_multi_sensor_config_t multi_cfg;

    /* Frame linearization scratch buffer (new SDK ring-buffer wrap) */
    uint8_t *scratch_buf;
    size_t scratch_size;

    /* Per-channel NAL unit arrays (reused across get_frame calls) */
    rss_nal_unit_t *nal_arrays[RSS_MAX_ENC_CHANNELS];
    int nal_array_caps[RSS_MAX_ENC_CHANNELS];

    /* Per-channel vendor stream struct (reused across get/release_frame) */
    IMPEncoderStream stream_priv[RSS_MAX_ENC_CHANNELS];

    /* Platform-specific opaque data */
    void *platform;

    bool initialized;
};

/* ═══════════════════════════════════════════════════════════════════════
 * 9. Internal Function Declarations
 * ═══════════════════════════════════════════════════════════════════════ */

/* hal_caps.c */
const rss_hal_caps_t *hal_caps_get(void);

/* ═══════════════════════════════════════════════════════════════════════
 * 10. Input Clamping
 *
 * HAL ISP setters accept int from callers (config values, JSON commands)
 * and clamp to the SDK's expected range before forwarding.
 * ═══════════════════════════════════════════════════════════════════════ */

static inline uint8_t hal_clamp_u8(int val)
{
    if (val < 0)
        return 0;
    if (val > 255)
        return 255;
    return (uint8_t)val;
}

static inline uint32_t hal_clamp_u32(int val)
{
    if (val < 0)
        return 0;
    return (uint32_t)val;
}

#endif /* HAL_INTERNAL_H */
