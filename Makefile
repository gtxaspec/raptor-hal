# Raptor HAL - Hardware Abstraction Layer for Ingenic SoCs
#
# Usage:
#   make PLATFORM=T31 CROSS_COMPILE=mipsel-linux-
#   make PLATFORM=T40 CROSS_COMPILE=mipsel-linux- INGENIC_HEADERS=/path/to/headers
#   make PLATFORM=T31 clean
#
# Required variables:
#   PLATFORM        - Target SoC: T20, T21, T23, T30, T31, T32, T40, T41
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
VALID_PLATFORMS := T20 T21 T23 T30 T31 T32 T33 T40 T41
ifeq ($(filter $(PLATFORM),$(VALID_PLATFORMS)),)
$(error Invalid PLATFORM=$(PLATFORM). Valid: $(VALID_PLATFORMS))
endif
endif # clean guard

# SDK version mapping
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
CFLAGS  += -I$(SDK_INCLUDE)
CFLAGS  += -I$(SDK_INCLUDE)/imp
CFLAGS  += -Iinclude
CFLAGS  += -Isrc

ifeq ($(DEBUG),1)
CFLAGS  += -O0 -g -DHAL_DEBUG
else
CFLAGS  += -Os
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
              src/hal_framesource.c \
              src/hal_isp.c \
              src/hal_osd.c \
              src/hal_gpio.c \
              src/hal_ivs.c \
              src/hal_memory.c

AUDIO_SRCS := src/hal_audio.c \
              src/hal_dmic.c

CXX_SRCS :=
ifneq ($(JZDL_INCLUDE),)
CXX_SRCS += src/hal_ivs_jzdl.cpp
CXXFLAGS := $(CFLAGS) -std=c++11 -DJZ_MXU=0 -I$(JZDL_INCLUDE) -fno-exceptions -fno-rtti
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
src/hal_common_video.o: src/hal_common.c
	@echo "  CC      $< (video)"
	$(Q)$(CC) $(CFLAGS) -DHAL_MODULE_VIDEO -MMD -MP -c $< -o $@

src/hal_common_audio.o: src/hal_common.c
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
	$(Q)rm -f $(ALL_OBJS) $(DEPS) $(LIB_VIDEO) $(LIB_AUDIO)

info:
	@echo "Platform:        $(PLATFORM)"
	@echo "SDK version:     $(HEADER_VER)"
	@echo "SDK language:    $(HEADER_LANG)"
	@echo "SDK include:     $(SDK_INCLUDE)"
	@echo "Cross-compile:   $(CROSS_COMPILE)"
	@echo "CFLAGS:          $(CFLAGS)"

-include $(DEPS)
