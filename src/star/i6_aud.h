/*
 * star/i6_aud.h -- MI_AI (audio input) ABI for SigmaStar Infinity6E
 *
 * Layouts and symbol names come from divinus's src/hal/star/i6_aud.h
 * (MIT), independently corroborated by waybeam's star6e_audio.c, which
 * declares the same device-config and frame structs field for field.
 *
 * That corroboration matters because the vendored SigmaStar MI_AI
 * reference (ref/sigmastar-docs, MI AI API 2.19) documents
 * MI_AUDIO_Attr_t with a *different field order* -- eBitwidth first,
 * then eSamplerate, eSoundmode, eWorkmode, u32PtNumPerFrm, u32ChnCnt.
 * Those docs are for SSD20X, a different chip generation, so the doc is
 * the authority on **semantics** (what the fields mean, what values are
 * legal, what the calls guarantee) and the two references are the
 * authority on **layout** for this SoC. Do not "fix" the field order to
 * match the documentation.
 *
 * CAPTURE ONLY. There is deliberately no MI_AO here and no AI encoder:
 *
 *   - Nothing in scope plays audio out, so libmi_ao is never loaded.
 *   - MI's own encoder (MI_AI_SetAencAttr / EnableAenc / DisableAenc) is
 *     unused. divinus dlsym's all three and has zero call sites for any
 *     of them; waybeam does not load them at all. Both capture raw PCM
 *     and encode in userspace, and so does raptor -- rad has its own
 *     G.711/L16/Opus/AAC encoders. Omitting them keeps the symbol set to
 *     what is actually called.
 *   - The VQE / AEC / ANR / AGC / EQ / AED / SSL / BF entry points exist
 *     in libmi_ai.so but their implementations do not: the algorithm
 *     packs (IaaXxx_*, MI_AED_*, the SDK-side G711 and G726 helpers) are
 *     declared **weak undefined**, so they resolve to NULL rather than
 *     failing the link.
 *     Calling into them would jump to address 0. See hal_audio.c's OP
 *     COVERAGE comment.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef I6_AUD_H
#define I6_AUD_H

#include "i6_common.h"

/* Tracks per AI channel. divinus's I6_AUD_CHN_NUM; the frame struct
 * carries one address per track, so the array bound is part of the ABI
 * and must not be trimmed even though this backend uses one or two. */
#define I6_AUD_CHN_NUM 16

typedef enum {
    I6_AUD_CLK_OFF,
    I6_AUD_CLK_12_288M,
    I6_AUD_CLK_16_384M,
    I6_AUD_CLK_18_432M,
    I6_AUD_CLK_24_576M,
    I6_AUD_CLK_24M,
    I6_AUD_CLK_48M
} i6_aud_clk;

typedef enum {
    I6_AUD_INTF_I2S_MASTER,
    I6_AUD_INTF_I2S_SLAVE,
    I6_AUD_INTF_TDM_MASTER,
    I6_AUD_INTF_TDM_SLAVE,
    I6_AUD_INTF_END
} i6_aud_intf;

typedef enum {
    I6_AUD_SND_MONO,
    I6_AUD_SND_STEREO,
    /* Several physical channels carried in one AI channel. Unused here:
     * per the MI docs every channel then shares one set of algorithm
     * parameters, and it only makes sense for a mic array. */
    I6_AUD_SND_QUEUE,
    I6_AUD_SND_END
} i6_aud_snd;

typedef struct {
    int leftJustOn;
    i6_aud_clk clock;
    char syncRxClkOn;
    unsigned int tdmSlotNum;
    int bit24On;
} i6_aud_i2s;

typedef struct {
    /* MI accepts 8/16/32/48 kHz only -- the docs say so explicitly, and
     * the resampler that would cover anything else (IaaSrc_*) is one of
     * the weak-undefined algorithm entry points. hal_audio.c rejects
     * other rates rather than letting MI resample into a NULL call. */
    int rate;
    int bit24On;
    i6_aud_intf intf;
    i6_aud_snd sound;
    /* DMA ring depth in frames, and samples per frame per channel. */
    unsigned int frmNum;
    unsigned int packNumPerFrm;
    /* Zero: no SDK codec channel. Both references set this to 0. */
    unsigned int codecChnNum;
    unsigned int chnNum;
    i6_aud_i2s i2s;
} i6_aud_cnf;

typedef struct {
    int bit24On;
    i6_aud_snd sound;
    unsigned char *addr[I6_AUD_CHN_NUM];
    unsigned long long timestamp;
    unsigned int sequence;
    unsigned int length;
    unsigned int poolId[2];
    unsigned char *pcmAddr[I6_AUD_CHN_NUM];
    unsigned int pcmLength;
} i6_aud_frm;

/* The AEC reference frame. Passed as NULL by both references and by this
 * backend -- echo cancellation is one of the absent algorithms -- but it
 * is part of the GetFrame/ReleaseFrame signatures, so the type has to
 * exist. */
typedef struct {
    i6_aud_frm frame;
    char isValid;
} i6_aud_efrm;

typedef struct {
    void *handle;

    int (*fnDisableDevice)(int device);
    int (*fnEnableDevice)(int device);
    int (*fnSetDeviceConfig)(int device, i6_aud_cnf *config);

    int (*fnDisableChannel)(int device, int channel);
    int (*fnEnableChannel)(int device, int channel);

    int (*fnSetMute)(int device, int channel, char active);
    /* Despite the name this takes an *index* into a per-device gain
     * table, not a decibel value -- see star_audio_volume_index in
     * hal_audio.c. Both references pass dB-shaped numbers here. */
    int (*fnSetVolume)(int device, int channel, int level);

    int (*fnGetFrame)(int device, int channel, i6_aud_frm *frame, i6_aud_efrm *encFrame,
                      int millis);
    int (*fnFreeFrame)(int device, int channel, i6_aud_frm *frame, i6_aud_efrm *encFrame);
} i6_aud_impl;

/*
 * libmi_ai.so needs libcam_os_wrapper and libmi_sys, and declares no
 * DT_NEEDED for either, so both must already be RTLD_GLOBAL in the
 * process. i6_sys_load does that, and hal_audio.c calls it first.
 *
 * RTLD_LAZY, like every other loader here: libmi_ai.so's optional
 * algorithm symbols are weak undefined and PLT-lazy, so binding eagerly
 * buys nothing and only risks turning an unused NULL into a load
 * failure.
 */
static inline int i6_aud_load(i6_aud_impl *aud_lib)
{
    if (!(aud_lib->handle = dlopen("libmi_ai.so", RTLD_LAZY | RTLD_GLOBAL))) {
        HAL_LOG_ERR("i6_aud: dlopen(libmi_ai.so) failed: %s", dlerror());
        return RSS_ERR_NOENT;
    }

    if (!(aud_lib->fnDisableDevice =
              (int (*)(int))hal_symbol_load("i6_aud", aud_lib->handle, "MI_AI_Disable")))
        return RSS_ERR_NOTSUP;

    if (!(aud_lib->fnEnableDevice =
              (int (*)(int))hal_symbol_load("i6_aud", aud_lib->handle, "MI_AI_Enable")))
        return RSS_ERR_NOTSUP;

    if (!(aud_lib->fnSetDeviceConfig = (int (*)(int, i6_aud_cnf *))hal_symbol_load(
              "i6_aud", aud_lib->handle, "MI_AI_SetPubAttr")))
        return RSS_ERR_NOTSUP;

    if (!(aud_lib->fnDisableChannel =
              (int (*)(int, int))hal_symbol_load("i6_aud", aud_lib->handle, "MI_AI_DisableChn")))
        return RSS_ERR_NOTSUP;

    if (!(aud_lib->fnEnableChannel =
              (int (*)(int, int))hal_symbol_load("i6_aud", aud_lib->handle, "MI_AI_EnableChn")))
        return RSS_ERR_NOTSUP;

    if (!(aud_lib->fnSetMute = (int (*)(int, int, char))hal_symbol_load(
              "i6_aud", aud_lib->handle, "MI_AI_SetMute")))
        return RSS_ERR_NOTSUP;

    /* MI_AI_SetVqeVolume, not MI_AI_SetVolume: both references use the
     * VQE variant and it works with the VQE algorithm libraries absent,
     * because the volume is a codec register write rather than part of
     * the sound-quality pipeline. */
    if (!(aud_lib->fnSetVolume = (int (*)(int, int, int))hal_symbol_load(
              "i6_aud", aud_lib->handle, "MI_AI_SetVqeVolume")))
        return RSS_ERR_NOTSUP;

    if (!(aud_lib->fnGetFrame = (int (*)(int, int, i6_aud_frm *, i6_aud_efrm *, int))
              hal_symbol_load("i6_aud", aud_lib->handle, "MI_AI_GetFrame")))
        return RSS_ERR_NOTSUP;

    if (!(aud_lib->fnFreeFrame = (int (*)(int, int, i6_aud_frm *, i6_aud_efrm *))hal_symbol_load(
              "i6_aud", aud_lib->handle, "MI_AI_ReleaseFrame")))
        return RSS_ERR_NOTSUP;

    return RSS_OK;
}

static inline void i6_aud_unload(i6_aud_impl *aud_lib)
{
    if (aud_lib->handle)
        dlclose(aud_lib->handle);
    memset(aud_lib, 0, sizeof(*aud_lib));
}

#endif /* I6_AUD_H */
