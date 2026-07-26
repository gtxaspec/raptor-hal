/*
 * i6_rgn.h -- MI_RGN (region/overlay) ABI for Infinity6E, with a dlopen
 * loader.
 *
 * Vendored from divinus (MIT) `src/hal/star/i6_rgn.h`, corroborated
 * field for field against waybeam's `src/debug_osd.c`. There is **no
 * MI_RGN chapter in the vendored SSD20X documentation** -- unlike MI_SYS,
 * MI_VENC, MI_AI and MI_AO, RGN was never part of that doc set -- so
 * unlike i6_aud.h there is no "semantics from the docs" half here. Both
 * references are the whole authority, which is exactly why the two traps
 * below are called out rather than left to be rediscovered.
 *
 * TRAP 1: THE MODULE ID IN AttachToChn IS NOT i6_sys_mod.
 *
 * MI_RGN_AttachToChn takes what looks like an ordinary i6_sys_bind, but
 * its `module` field uses **RGN's own private module enum** (see
 * i6_rgn_mod below): VPE is 0 and VENC is 2. Under i6_sys_mod, 0 is IVE
 * and 2 is VENC. That is why divinus's region code sets `.module = 0`
 * while filling in VPE's device and channel -- it reads like a bug and
 * is not. waybeam documents the same two ids explicitly and warns that
 * the values come from the device's libmi_rgn build. If attach starts
 * failing after a vendor library update, look here first.
 *
 * TRAP 2: THE PIXEL FORMAT SUPPORT SET IS NOT KNOWN STATICALLY.
 *
 * `i6_rgn_pixfmt` below is the full enum the vendor library declares,
 * but which entries a given chip accepts for an OSD region is decided in
 * mi_rgn.ko (it maps MI formats onto MHAL ones and rejects the rest with
 * "ePixelFmt not support"). mhal.ko names ARGB1555, ARGB4444, ARGB8888
 * and the I4/I8 palettes, but the accepted *OSD* subset could not be
 * pinned down by disassembly cheaply, and the two references only ever
 * use ARGB1555 (divinus) and I4 (waybeam) -- neither of which is
 * evidence about the 32-bit formats. hal_osd.c therefore probes at
 * create time and keeps whichever format the driver accepts, rather than
 * this header pretending to know.
 *
 * Note ARGB888 is the vendor's spelling of a 32-bit ARGB8888 format; the
 * name is kept as the SDK has it so the enum ordering stays checkable
 * against the reference sources.
 */
#pragma once

#include <dlfcn.h>

#include "i6_common.h"
#include "i6_sys.h"

/*
 * RGN's private module enum for AttachToChn/DetachFromChn. See TRAP 1.
 * Only the two ids both references confirm are named; the rest of the
 * enum is not needed and guessing at it would be fiction.
 */
typedef enum {
    I6_RGN_MOD_VPE = 0,
    I6_RGN_MOD_VENC = 2,
} i6_rgn_mod;

typedef enum {
    I6_RGN_PIXFMT_ARGB1555,
    I6_RGN_PIXFMT_ARGB4444,
    I6_RGN_PIXFMT_I2,
    I6_RGN_PIXFMT_I4,
    I6_RGN_PIXFMT_I8,
    I6_RGN_PIXFMT_RGB565,
    I6_RGN_PIXFMT_ARGB888,
    I6_RGN_PIXFMT_END
} i6_rgn_pixfmt;

typedef enum {
    I6_RGN_TYPE_OSD,
    I6_RGN_TYPE_COVER,
    I6_RGN_TYPE_END
} i6_rgn_type;

typedef struct {
    unsigned int width;
    unsigned int height;
} i6_rgn_size;

typedef struct {
    i6_rgn_pixfmt pixFmt;
    i6_rgn_size size;
    void *data;
} i6_rgn_bmp;

typedef struct {
    i6_rgn_type type;
    i6_rgn_pixfmt pixFmt;
    i6_rgn_size size;
} i6_rgn_cnf;

typedef struct {
    unsigned int layer;
    i6_rgn_size size;
    unsigned int color;
} i6_rgn_cov;

typedef struct {
    int invColOn;
    int lowThanThresh;
    unsigned int lumThresh;
    unsigned short divWidth;
    unsigned short divHeight;
} i6_rgn_inv;

typedef struct {
    unsigned int layer;
    int constAlphaOn;
    union {
        unsigned char bgFgAlpha[2];
        unsigned char constAlpha[2];
    };
    i6_rgn_inv invert;
} i6_rgn_osd;

typedef struct {
    unsigned int x;
    unsigned int y;
} i6_rgn_pnt;

typedef struct {
    int show;
    i6_rgn_pnt point;
    union {
        i6_rgn_cov cover;
        i6_rgn_osd osd;
    };
} i6_rgn_chn;

typedef struct {
    unsigned char alpha;
    unsigned char red;
    unsigned char green;
    unsigned char blue;
} i6_rgn_pale;

typedef struct {
    i6_rgn_pale element[256];
} i6_rgn_pal;

_Static_assert(sizeof(i6_rgn_cnf) == 16, "i6_rgn_cnf must be type + pixFmt + 2x u32 size");
_Static_assert(sizeof(i6_rgn_bmp) == 16, "i6_rgn_bmp must be pixFmt + 2x u32 size + pointer");
_Static_assert(sizeof(i6_rgn_pal) == 1024, "i6_rgn_pal is 256 x 4-byte entries");

typedef struct {
    void *handle;

    int (*fnInit)(i6_rgn_pal *palette);
    int (*fnDeinit)(void);

    int (*fnCreateRegion)(unsigned int handle, i6_rgn_cnf *config);
    int (*fnDestroyRegion)(unsigned int handle);
    int (*fnGetRegionConfig)(unsigned int handle, i6_rgn_cnf *config);

    int (*fnAttachChannel)(unsigned int handle, i6_sys_bind *dest, i6_rgn_chn *config);
    int (*fnDetachChannel)(unsigned int handle, i6_sys_bind *dest);
    int (*fnGetChannelConfig)(unsigned int handle, i6_sys_bind *dest, i6_rgn_chn *config);
    int (*fnSetChannelConfig)(unsigned int handle, i6_sys_bind *dest, i6_rgn_chn *config);

    int (*fnSetBitmap)(unsigned int handle, i6_rgn_bmp *bitmap);
} i6_rgn_impl;

/*
 * libmi_rgn.so declares no cross-library DT_NEEDED, like every other MI
 * module, but its dependency closure is only libcam_os_wrapper +
 * libmi_sys -- both of which i6_sys_load has already opened RTLD_GLOBAL
 * by the time any OSD op can run. So there is nothing to preload here.
 */
static inline int i6_rgn_load(i6_rgn_impl *rgn_lib)
{
    if (!(rgn_lib->handle = dlopen("libmi_rgn.so", RTLD_LAZY | RTLD_GLOBAL))) {
        HAL_LOG_ERR("i6_rgn: dlopen(libmi_rgn.so) failed: %s", dlerror());
        return RSS_ERR_NOENT;
    }

    if (!(rgn_lib->fnInit = (int (*)(i6_rgn_pal *))hal_symbol_load("i6_rgn", rgn_lib->handle,
                                                                  "MI_RGN_Init")))
        return RSS_ERR_NOTSUP;

    if (!(rgn_lib->fnDeinit =
              (int (*)(void))hal_symbol_load("i6_rgn", rgn_lib->handle, "MI_RGN_DeInit")))
        return RSS_ERR_NOTSUP;

    if (!(rgn_lib->fnCreateRegion = (int (*)(unsigned int, i6_rgn_cnf *))hal_symbol_load(
              "i6_rgn", rgn_lib->handle, "MI_RGN_Create")))
        return RSS_ERR_NOTSUP;

    if (!(rgn_lib->fnDestroyRegion = (int (*)(unsigned int))hal_symbol_load(
              "i6_rgn", rgn_lib->handle, "MI_RGN_Destroy")))
        return RSS_ERR_NOTSUP;

    if (!(rgn_lib->fnGetRegionConfig = (int (*)(unsigned int, i6_rgn_cnf *))hal_symbol_load(
              "i6_rgn", rgn_lib->handle, "MI_RGN_GetAttr")))
        return RSS_ERR_NOTSUP;

    if (!(rgn_lib->fnAttachChannel =
              (int (*)(unsigned int, i6_sys_bind *, i6_rgn_chn *))hal_symbol_load(
                  "i6_rgn", rgn_lib->handle, "MI_RGN_AttachToChn")))
        return RSS_ERR_NOTSUP;

    if (!(rgn_lib->fnDetachChannel = (int (*)(unsigned int, i6_sys_bind *))hal_symbol_load(
              "i6_rgn", rgn_lib->handle, "MI_RGN_DetachFromChn")))
        return RSS_ERR_NOTSUP;

    if (!(rgn_lib->fnGetChannelConfig =
              (int (*)(unsigned int, i6_sys_bind *, i6_rgn_chn *))hal_symbol_load(
                  "i6_rgn", rgn_lib->handle, "MI_RGN_GetDisplayAttr")))
        return RSS_ERR_NOTSUP;

    if (!(rgn_lib->fnSetChannelConfig =
              (int (*)(unsigned int, i6_sys_bind *, i6_rgn_chn *))hal_symbol_load(
                  "i6_rgn", rgn_lib->handle, "MI_RGN_SetDisplayAttr")))
        return RSS_ERR_NOTSUP;

    if (!(rgn_lib->fnSetBitmap = (int (*)(unsigned int, i6_rgn_bmp *))hal_symbol_load(
              "i6_rgn", rgn_lib->handle, "MI_RGN_SetBitMap")))
        return RSS_ERR_NOTSUP;

    return RSS_OK;
}

static inline void i6_rgn_unload(i6_rgn_impl *rgn_lib)
{
    if (rgn_lib->handle)
        dlclose(rgn_lib->handle);
    memset(rgn_lib, 0, sizeof(*rgn_lib));
}
