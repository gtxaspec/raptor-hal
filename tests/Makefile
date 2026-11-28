CC ?= gcc
SAN ?= address
# hal_ivs_yolo.c includes hal_internal.h, which wants one PLATFORM_* and the
# SDK headers (the ingenic-headers submodule); the decode is the same on all
INGENIC_HEADERS ?= ../ingenic-headers
SDK_INCLUDE = $(INGENIC_HEADERS)/T31/1.1.6/en
CFLAGS = -Wall -Wextra -std=c11 -D_GNU_SOURCE -g -fsanitize=$(SAN) -I../include -I../src -I.
CFLAGS += -DPLATFORM_T31 -I$(SDK_INCLUDE) -I$(SDK_INCLUDE)/imp
LDFLAGS = -fsanitize=$(SAN) -lm

TEST_SRCS = main.c log_capture.c test_tune.c test_yolo.c test_direct.c test_mode.c test_ojzdl.c
LIB_SRCS = ../src/hal_tune.c ../src/hal_ivs_yolo.c ../src/hal_direct.c ../src/hal_mode.c
# ojzdl (OJZDL=1), plain C path; no fused multiply-add, or the head floats round differently
OJZDL_SRCS = ../src/ojzdl/ojzdl.c ../src/ojzdl/ojzdl_mxu2.c
CFLAGS += -ffp-contract=off

tests: $(TEST_SRCS) $(LIB_SRCS) $(OJZDL_SRCS) ../src/hal_tune.h ../src/hal_ivs_yolo.h ../src/hal_direct.h ../src/hal_mode.h ../src/hal_log.h ../src/ojzdl/ojzdl.h log_capture.h yolo_vectors.h greatest.h
	$(CC) $(CFLAGS) -o $@ $(TEST_SRCS) $(LIB_SRCS) $(OJZDL_SRCS) $(LDFLAGS)

test: tests
	./tests

clean:
	rm -f tests

.PHONY: test clean
