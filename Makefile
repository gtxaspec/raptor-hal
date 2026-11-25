# Raptor HAL - Hardware Abstraction Layer for Ingenic and Novatek SoCs
#
# Usage:
#   make PLATFORM=T31 CROSS_COMPILE=mipsel-linux-
#   make PLATFORM=T40 CROSS_COMPILE=mipsel-linux- INGENIC_HEADERS=/path/to/headers
#   make PLATFORM=NA51055 CROSS_COMPILE=arm-linux- HDAL_INCLUDE=/path/to/hdal/include
#   make PLATFORM=T31 clean
#
# Required variables:
#   PLATFORM        - Target SoC: T10, T20, T21, T23, T30, T31, T32, T40, T41, NA51055
#   CROSS_COMPILE   - Cross-compiler prefix (e.g. mipsel-linux-)
#
# Optional variables:
#   INGENIC_HEADERS - Path to ingenic-headers repo (default: ../ingenic-headers)
#   HDAL_INCLUDE    - Path to the Novatek HDAL headers (NA51055 only)
#   INGENIC_LIB     - Path to ingenic-lib repo (default: ../ingenic-lib)
#   V4L2_OPENIMP    - Set to 1 for the optional V4L2/OpenIMP AVC bridge
#   JZDL_INCLUDE    - Path to Ingenic jzdl headers: build hal_jzdl_* on libjzdl.m.so
#   OJZDL           - Set to 1 to build hal_jzdl_* on ojzdl, the open runtime in
#                     src/ojzdl (no libjzdl, no libstdc++); exclusive with JZDL_INCLUDE
#   ONNA            - Set to 1 (T41) to build hal_jzdl_* on onna, the open NNA runtime in
#                     src/onna (no venus/Magik libraries, no nmem); exclusive with the two above
#   DEBUG           - Set to 1 for debug build
#   V               - Set to 1 for verbose output

ifeq ($(filter clean,$(MAKECMDGOALS)),)
ifndef PLATFORM
$(error PLATFORM not set. Use: make PLATFORM=T31)
endif

# Validate platform
VALID_PLATFORMS := T10 T20 T21 T23 T30 T31 T32 T33 T40 T41 NA51055
ifeq ($(filter $(PLATFORM),$(VALID_PLATFORMS)),)
$(error Invalid PLATFORM=$(PLATFORM). Valid: $(VALID_PLATFORMS))
endif
endif # clean guard

# SDK version mapping
HEADER_VER_T10 := 3.12.0
HEADER_VER_T20 := 3.12.0
HEADER_VER_T21 := 1.0.33
HEADER_VER_T23 := 1.3.0
HEADER_VER_T30 := 1.0.5
HEADER_VER_T31 := 1.1.6
HEADER_VER_T32 := 2.2.0
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
SDK_INCLUDE     := $(INGENIC_HEADERS)/$(PLATFORM)/$(HEADER_VER)/$(HEADER_LANG)

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
ifeq ($(PLATFORM),NA51055)
NOVATEK := 1
HDAL_INCLUDE ?= hdal-include
# Vendor headers are not -Wextra clean.
CFLAGS  += -isystem $(HDAL_INCLUDE) -D__LINUX
else
CFLAGS  += -I$(SDK_INCLUDE)
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

# Sources — shared across both archives
CORE_SRCS := src/hal_caps.c

VIDEO_SRCS := src/hal_encoder.c \
              src/hal_tune.c \
              src/hal_framesource.c \
              src/hal_isp.c \
              src/hal_osd.c \
              src/hal_gpio.c \
              src/hal_ivs.c \
              src/hal_memory.c

ifeq ($(V4L2_OPENIMP),1)
VIDEO_SRCS += src/hal_v4l2.c
CFLAGS += -DV4L2_OPENIMP
endif

AUDIO_SRCS := src/hal_audio.c \
              src/hal_dmic.c

# Open JZDL inference: same hal_jzdl_* API, results bit-identical to libjzdl.
# MXU2 kernels on the T31 (the only SoC they are tested on); C elsewhere.
OJZDL ?=
ifeq ($(OJZDL),1)
ifneq ($(JZDL_INCLUDE),)
$(error OJZDL=1 and JZDL_INCLUDE are mutually exclusive)
endif
OJZDL_SRCS := src/hal_ivs_ojzdl.c src/ojzdl/ojzdl.c src/ojzdl/ojzdl_mxu2.c src/ojzdl/ojzdl_resize.c
VIDEO_SRCS += $(OJZDL_SRCS) src/hal_ivs_yolo.c
# -O2: the runtime's loops lose ~12 % at -Os on the T31 (measured)
OJZDL_FLAGS := -D_POSIX_C_SOURCE=200809L -ffp-contract=off -O2
ifeq ($(PLATFORM),T31)
OJZDL_FLAGS += -DOJZDL_MXU2
endif
$(OJZDL_SRCS:.c=.o): CFLAGS += $(OJZDL_FLAGS)
endif

# onna: hal_jzdl_* on the T41 NNA through the open onna runtime (src/onna): venus
# .bin models of the QSUM kind, no venus/Magik libraries, no nmem. Uses ojzdl's
# resize, so the input matches the T31 path.
ONNA ?=
ifeq ($(ONNA),1)
ifneq ($(PLATFORM),T41)
$(error ONNA=1 is for the T41's NNA)
endif
ifneq ($(JZDL_INCLUDE)$(OJZDL),)
$(error ONNA=1, OJZDL=1 and JZDL_INCLUDE are mutually exclusive)
endif
ONNA_SRCS := src/hal_ivs_onna.c src/onna/onnaq.c src/onna/nnaeng.c src/onna/nnacpu.c \
             src/onna/be_mxu3.c src/ojzdl/ojzdl_resize.c
VIDEO_SRCS += $(ONNA_SRCS) src/hal_ivs_yolo.c
ONNA_FLAGS := -D_POSIX_C_SOURCE=200809L -ffp-contract=off -O2 -DONNA_MXU3 -DONNAQ_LIB -DOJZDL_MXU3
$(ONNA_SRCS:.c=.o): CFLAGS += $(ONNA_FLAGS)
endif

CXX_SRCS :=
ifneq ($(JZDL_INCLUDE),)
CXX_SRCS += src/hal_ivs_jzdl.cpp
VIDEO_SRCS += src/hal_ivs_yolo.c
CXXFLAGS := $(filter-out -std=c11,$(CFLAGS)) -std=c++11 -Wno-missing-field-initializers -DJZ_MXU=0 -I$(JZDL_INCLUDE) -fno-exceptions -fno-rtti
endif

# Novatek: one self-contained backend in both archives, no IMP sources.
ifeq ($(NOVATEK),1)
CORE_SRCS  := src/hal_novatek.c
VIDEO_SRCS :=
AUDIO_SRCS :=
CXX_SRCS   :=
endif

CORE_OBJS  := $(CORE_SRCS:.c=.o)
VIDEO_OBJS := $(VIDEO_SRCS:.c=.o) $(CXX_SRCS:.cpp=.o)
AUDIO_OBJS := $(AUDIO_SRCS:.c=.o)

ifeq ($(NOVATEK),1)
COMMON_VIDEO_OBJ :=
COMMON_AUDIO_OBJ :=
else
COMMON_VIDEO_OBJ := src/hal_common_video.o
COMMON_AUDIO_OBJ := src/hal_common_audio.o
endif

ALL_OBJS := $(COMMON_VIDEO_OBJ) $(COMMON_AUDIO_OBJ) \
            $(CORE_OBJS) $(VIDEO_OBJS) $(AUDIO_OBJS)
DEPS := $(ALL_OBJS:.o=.d)

# Output — two archives, one per module set
LIB_VIDEO := libraptor_hal_video.a
LIB_AUDIO := libraptor_hal_audio.a

.PHONY: all clean info

all: $(LIB_VIDEO) $(LIB_AUDIO)

# Compile hal_common.c twice with different module defines
src/hal_common_video.o: src/hal_common.c
	@echo "  CC      $< (video)"
	$(Q)$(CC) $(CFLAGS) -DHAL_MODULE_VIDEO -MMD -MP -c $< -o $@

src/hal_common_audio.o: src/hal_common.c
	@echo "  CC      $< (audio)"
	$(Q)$(CC) $(CFLAGS) -DHAL_MODULE_AUDIO -MMD -MP -c $< -o $@

$(LIB_VIDEO): $(COMMON_VIDEO_OBJ) $(CORE_OBJS) $(VIDEO_OBJS)
	@echo "  AR      $@"
	$(Q)$(AR) rcs $@ $^
	$(Q)$(RANLIB) $@

$(LIB_AUDIO): $(COMMON_AUDIO_OBJ) $(CORE_OBJS) $(AUDIO_OBJS)
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
	$(Q)rm -f $(ALL_OBJS) $(DEPS) src/hal_v4l2.o src/hal_v4l2.d src/hal_novatek.o src/hal_novatek.d \
		$(LIB_VIDEO) $(LIB_AUDIO)

info:
	@echo "Platform:        $(PLATFORM)"
	@echo "SDK version:     $(HEADER_VER)"
	@echo "SDK language:    $(HEADER_LANG)"
	@echo "SDK include:     $(SDK_INCLUDE)"
	@echo "Cross-compile:   $(CROSS_COMPILE)"
	@echo "CFLAGS:          $(CFLAGS)"

-include $(DEPS)
