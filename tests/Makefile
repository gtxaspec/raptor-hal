# Host-side logic tests for the SigmaStar (Infinity6E) backend.
#
# Each test #includes the real translation unit it exercises, so the code
# under test is the code that ships, and stubs MI by filling in the
# function pointers in star_state_t. That is the whole reason these can
# run on a host at all: nothing in src/star links libmi_*.so directly,
# every call goes through a dlopen'd pointer.
#
# WHY -D'_Static_assert(c,m)='
#
# The i6_*.h headers assert the vendor ABI's 32-bit struct layouts
# (sizeof(pointer) == 4 and the offsets that follow from it). On an x86-64
# host every one of those is false, and they are *supposed* to be -- they
# exist to catch an ABI drift in the ARM build, which still checks all of
# them with -Werror. Suppressing them here is what lets the logic be
# tested at all. -m32 would be the alternative and is not available: this
# host has no 32-bit libc headers.
#
#   make -C tests test      build and run everything
#   make -C tests test-osd   just one suite
#
# Each suite is its own binary with its own main(), so a crash in one does
# not hide the others.

CC ?= gcc
SAN ?= address

CFLAGS = -Wall -Wextra -std=c11 -D_GNU_SOURCE -g -fsanitize=$(SAN) \
	 -D'_Static_assert(c,m)=' -DPLATFORM_INFINITY6E=1 \
	 -I../src -I../include -I../../raptor-common/include
LDFLAGS = -fsanitize=$(SAN) -lpthread

# gpio is the odd one out: src/hal_gpio.c is platform-independent sysfs
# rather than a star/ file. It is here because phase 6 (IR-cut day/night)
# rests on it and its behaviour is which file gets written in what order,
# which a host can check once the sysfs root is redirected.
SUITES = osd isp audio gpio
BINS = $(addprefix t_,$(SUITES))

test: $(addprefix test-,$(SUITES))

$(addprefix test-,$(SUITES)): test-%: t_%
	@./$<

t_%: t_%.c
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

clean:
	rm -f $(BINS)

.PHONY: test clean $(addprefix test-,$(SUITES))
