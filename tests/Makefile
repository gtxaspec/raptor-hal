CC ?= gcc
SAN ?= address
CFLAGS = -Wall -Wextra -std=c11 -D_GNU_SOURCE -g -fsanitize=$(SAN) -I../include -I../src -I.
LDFLAGS = -fsanitize=$(SAN)

TEST_SRCS = main.c test_tune.c test_direct.c test_ojzdl.c
LIB_SRCS = ../src/hal_tune.c ../src/hal_direct.c
# ojzdl (OJZDL=1), plain C path; no fused multiply-add, or the head floats round differently
OJZDL_SRCS = ../src/ojzdl/ojzdl.c ../src/ojzdl/ojzdl_mxu2.c
CFLAGS += -ffp-contract=off

tests: $(TEST_SRCS) $(LIB_SRCS) $(OJZDL_SRCS) ../src/hal_tune.h ../src/hal_direct.h ../src/ojzdl/ojzdl.h greatest.h
	$(CC) $(CFLAGS) -o $@ $(TEST_SRCS) $(LIB_SRCS) $(OJZDL_SRCS) $(LDFLAGS) -lm

test: tests
	./tests

clean:
	rm -f tests

.PHONY: test clean
