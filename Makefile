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

ifndef PLATFORM
$(error PLATFORM not set. Use: make PLATFORM=T31)
endif

# Validate platform
VALID_PLATFORMS := T20 T21 T23 T30 T31 T32 T40 T41
ifeq ($(filter $(PLATFORM),$(VALID_PLATFORMS)),)
$(error Invalid PLATFORM=$(PLATFORM). Valid: $(VALID_PLATFORMS))
endif

# SDK version mapping
HEADER_VER_T20 := 3.12.0
HEADER_VER_T21 := 1.0.33
HEADER_VER_T23 := 1.3.0
HEADER_VER_T30 := 1.0.5
HEADER_VER_T31 := 1.1.6
HEADER_VER_T32 := 1.0.6
HEADER_VER_T40 := 1.3.1
HEADER_VER_T41 := 1.2.5

# Language preference (en if available, zh otherwise)
HEADER_LANG_T20 := zh
HEADER_LANG_T21 := zh
HEADER_LANG_T23 := en
HEADER_LANG_T30 := zh
HEADER_LANG_T31 := en
HEADER_LANG_T32 := en
HEADER_LANG_T40 := en
HEADER_LANG_T41 := en

HEADER_VER  := $(HEADER_VER_$(PLATFORM))
HEADER_LANG := $(HEADER_LANG_$(PLATFORM))

# Paths
INGENIC_HEADERS ?= ../ingenic-headers
INGENIC_LIB     ?= ../ingenic-lib
SDK_INCLUDE     := $(INGENIC_HEADERS)/$(PLATFORM)/$(HEADER_VER)/$(HEADER_LANG)

# Toolchain
CC      := $(CROSS_COMPILE)gcc
AR      := $(CROSS_COMPILE)ar
RANLIB  := $(CROSS_COMPILE)ranlib

# Flags
CFLAGS  := -Wall -Wextra -Werror=implicit-function-declaration
CFLAGS  += -std=c11
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

# Sources
SRCS := src/hal_caps.c \
        src/hal_common.c \
        src/hal_encoder.c \
        src/hal_framesource.c \
        src/hal_isp.c \
        src/hal_audio.c \
        src/hal_osd.c \
        src/hal_gpio.c \
        src/hal_ivs.c \
        src/hal_dmic.c \
        src/hal_memory.c

OBJS := $(SRCS:.c=.o)
DEPS := $(SRCS:.c=.d)

# Output
LIB := libraptor_hal.a

.PHONY: all clean info

all: $(LIB)

$(LIB): $(OBJS)
	@echo "  AR      $@"
	$(Q)$(AR) rcs $@ $^
	$(Q)$(RANLIB) $@

%.o: %.c
	@echo "  CC      $<"
	$(Q)$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

clean:
	@echo "  CLEAN"
	$(Q)rm -f $(OBJS) $(DEPS) $(LIB)

info:
	@echo "Platform:        $(PLATFORM)"
	@echo "SDK version:     $(HEADER_VER)"
	@echo "SDK language:    $(HEADER_LANG)"
	@echo "SDK include:     $(SDK_INCLUDE)"
	@echo "Cross-compile:   $(CROSS_COMPILE)"
	@echo "CFLAGS:          $(CFLAGS)"

-include $(DEPS)
