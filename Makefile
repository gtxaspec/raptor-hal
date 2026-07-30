# Raptor HAL - Hardware Abstraction Layer for Ingenic and SigmaStar SoCs
#
# Usage:
#   make PLATFORM=T31 CROSS_COMPILE=mipsel-linux-
#   make PLATFORM=T40 CROSS_COMPILE=mipsel-linux- INGENIC_HEADERS=/path/to/headers
#   make PLATFORM=INFINITY6E CROSS_COMPILE=arm-linux-gnueabihf-
#   make PLATFORM=T31 clean
#
# Required variables:
#   PLATFORM        - Target SoC:
#                       Ingenic   - T10, T20, T21, T23, T30, T31, T32, T33, T40, T41
#                       SigmaStar - INFINITY6E
#   CROSS_COMPILE   - Cross-compiler prefix (e.g. mipsel-linux-)
#
# Optional variables:
#   INGENIC_HEADERS - Path to ingenic-headers repo (default: ../ingenic-headers)
#   INGENIC_LIB     - Path to ingenic-lib repo (default: ../ingenic-lib)
#   DEBUG           - Set to 1 for debug build
#   V               - Set to 1 for verbose output

ifeq ($(filter clean,$(MAKECMDGOALS)),)
ifndef PLATFORM
$(error PLATFORM not set. Use: make PLATFORM=T31)
endif

# Validate platform
VALID_PLATFORMS := T10 T20 T21 T23 T30 T31 T32 T33 T40 T41 INFINITY6E
ifeq ($(filter $(PLATFORM),$(VALID_PLATFORMS)),)
$(error Invalid PLATFORM=$(PLATFORM). Valid: $(VALID_PLATFORMS))
endif
endif # clean guard

# Vendor selection — which SDK family this platform belongs to.
# Ingenic parts use the single-library IMP SDK; SigmaStar parts use the
# per-module MI SDK. Set unconditionally (not inside the clean guard) so
# `make clean` still resolves the right object paths.
SIGMASTAR_PLATFORMS := INFINITY6E
ifneq ($(filter $(PLATFORM),$(SIGMASTAR_PLATFORMS)),)
VENDOR := sigmastar
else
VENDOR := ingenic
endif

# SDK version mapping
HEADER_VER_T10 := 3.12.0
HEADER_VER_T20 := 3.12.0
HEADER_VER_T21 := 1.0.33
HEADER_VER_T23 := 1.3.0
HEADER_VER_T30 := 1.0.5
HEADER_VER_T31 := 1.1.6
HEADER_VER_T32 := 1.0.6
HEADER_VER_T33 := 2.0.2.1
HEADER_VER_T40 := 1.3.1
HEADER_VER_T41 := 1.2.5

# Language preference (en if available, zh otherwise)
HEADER_LANG_T10 := zh
HEADER_LANG_T20 := zh
HEADER_LANG_T21 := zh
HEADER_LANG_T23 := en
HEADER_LANG_T30 := zh
HEADER_LANG_T31 := en
HEADER_LANG_T32 := en
HEADER_LANG_T33 := en
HEADER_LANG_T40 := en
HEADER_LANG_T41 := en

HEADER_VER  := $(HEADER_VER_$(PLATFORM))
HEADER_LANG := $(HEADER_LANG_$(PLATFORM))

# Paths
INGENIC_HEADERS ?= ingenic-headers
INGENIC_LIB     ?= ../ingenic-lib

# SigmaStar needs no SDK path: the MI ABI declarations and their dlopen
# loaders live in src/star/i6_*.h, so there are no vendor headers to point at
# and no MI libraries to link. SDK_INCLUDE stays empty for that vendor.
ifeq ($(VENDOR),sigmastar)
SDK_INCLUDE     :=
else
SDK_INCLUDE     := $(INGENIC_HEADERS)/$(PLATFORM)/$(HEADER_VER)/$(HEADER_LANG)
endif

# Toolchain
CC      := $(CROSS_COMPILE)gcc
CXX     := $(CROSS_COMPILE)g++
AR      := $(CROSS_COMPILE)gcc-ar
RANLIB  := $(CROSS_COMPILE)gcc-ranlib

# JZDL inference (optional — set JZDL_INCLUDE to enable)
JZDL_INCLUDE ?=

# Flags
CFLAGS  := -Wall -Wextra -Werror
CFLAGS  += -std=c11
CFLAGS  += -ffunction-sections -fdata-sections -flto
CFLAGS  += -fno-asynchronous-unwind-tables -fmerge-all-constants -fno-ident
CFLAGS  += -DPLATFORM_$(PLATFORM)
ifneq ($(SDK_INCLUDE),)
CFLAGS  += -I$(SDK_INCLUDE)
# IMP headers live in an imp/ subdir and are included as <imp/imp_system.h>.
# Guarded on SDK_INCLUDE being non-empty rather than on the vendor: a bare
# -I would swallow the following flag as its argument.
CFLAGS  += -I$(SDK_INCLUDE)/imp
endif
CFLAGS  += -Iinclude
CFLAGS  += -Isrc

ifeq ($(DEBUG),1)
CFLAGS  += -O0 -g -DHAL_DEBUG
else
CFLAGS  += -Os
endif

ifeq ($(PERSONDET),1)
CFLAGS  += -DPERSONDET
endif

# Verbose
ifeq ($(V),1)
Q :=
else
Q := @
endif

# Sources — shared across both archives.
# hal_caps.c is pure per-SoC capability data and hal_gpio.c is plain sysfs
# GPIO with no SDK dependency, so both are vendor-neutral.
CORE_SRCS := src/hal_caps.c

ifeq ($(VENDOR),sigmastar)

# SigmaStar MI backend. Subsystems not yet ported simply omit their ops from
# the vtable — RSS_HAL_CALL NULL-guards every entry and returns RSS_ERR_NOTSUP,
# so there is no need for stub translation units per unimplemented subsystem.
HAL_COMMON_SRC := src/star/hal_common.c

VIDEO_SRCS := src/star/hal_encoder.c \
              src/star/hal_framesource.c \
              src/star/hal_isp.c \
              src/star/hal_osd.c \
              src/hal_gpio.c

AUDIO_SRCS := src/star/hal_audio.c

else

HAL_COMMON_SRC := src/hal_common.c

VIDEO_SRCS := src/hal_encoder.c \
              src/hal_framesource.c \
              src/hal_isp.c \
              src/hal_osd.c \
              src/hal_gpio.c \
              src/hal_ivs.c \
              src/hal_memory.c

AUDIO_SRCS := src/hal_audio.c \
              src/hal_dmic.c

endif

CXX_SRCS :=
ifneq ($(JZDL_INCLUDE),)
CXX_SRCS += src/hal_ivs_jzdl.cpp
CXXFLAGS := $(filter-out -std=c11,$(CFLAGS)) -std=c++11 -Wno-missing-field-initializers -DJZ_MXU=0 -I$(JZDL_INCLUDE) -fno-exceptions -fno-rtti
endif

CORE_OBJS  := $(CORE_SRCS:.c=.o)
VIDEO_OBJS := $(VIDEO_SRCS:.c=.o) $(CXX_SRCS:.cpp=.o)
AUDIO_OBJS := $(AUDIO_SRCS:.c=.o)

ALL_OBJS := src/hal_common_video.o src/hal_common_audio.o \
            $(CORE_OBJS) $(VIDEO_OBJS) $(AUDIO_OBJS)
DEPS := $(ALL_OBJS:.o=.d)

# Output — two archives, one per module set
LIB_VIDEO := libraptor_hal_video.a
LIB_AUDIO := libraptor_hal_audio.a

.PHONY: all clean info

all: $(LIB_VIDEO) $(LIB_AUDIO)

# Compile hal_common.c twice with different module defines
src/hal_common_video.o: $(HAL_COMMON_SRC)
	@echo "  CC      $< (video)"
	$(Q)$(CC) $(CFLAGS) -DHAL_MODULE_VIDEO -MMD -MP -c $< -o $@

src/hal_common_audio.o: $(HAL_COMMON_SRC)
	@echo "  CC      $< (audio)"
	$(Q)$(CC) $(CFLAGS) -DHAL_MODULE_AUDIO -MMD -MP -c $< -o $@

$(LIB_VIDEO): src/hal_common_video.o $(CORE_OBJS) $(VIDEO_OBJS)
	@echo "  AR      $@"
	$(Q)$(AR) rcs $@ $^
	$(Q)$(RANLIB) $@

$(LIB_AUDIO): src/hal_common_audio.o $(CORE_OBJS) $(AUDIO_OBJS)
	@echo "  AR      $@"
	$(Q)$(AR) rcs $@ $^
	$(Q)$(RANLIB) $@

%.o: %.c
	@echo "  CC      $<"
	$(Q)$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

%.o: %.cpp
	@echo "  CXX     $<"
	$(Q)$(CXX) $(CXXFLAGS) -MMD -MP -c $< -o $@

clean:
	@echo "  CLEAN"
	# .built is the stamp raptor/Makefile builds this directory through. It has
	# to go whenever the archives do: left behind, it tells the parent make that
	# archives it can no longer see have already been produced, and the next
	# daemon link fails on a missing .a.
	$(Q)rm -f $(ALL_OBJS) $(DEPS) $(LIB_VIDEO) $(LIB_AUDIO) .built
	# `make clean` runs without PLATFORM, so VENDOR defaults to ingenic and
	# $(ALL_OBJS) names only that backend's objects. Sweep the other
	# vendors' subdirs explicitly so a clean is vendor-independent.
	$(Q)rm -f src/*/*.o src/*/*.d

info:
	@echo "Platform:        $(PLATFORM)"
	@echo "Vendor:          $(VENDOR)"
	@echo "SDK version:     $(HEADER_VER)"
	@echo "SDK language:    $(HEADER_LANG)"
	@echo "SDK include:     $(SDK_INCLUDE)"
	@echo "Cross-compile:   $(CROSS_COMPILE)"
	@echo "CFLAGS:          $(CFLAGS)"

-include $(DEPS)
