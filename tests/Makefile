CC ?= gcc
SAN ?= address
CFLAGS = -Wall -Wextra -std=c11 -D_GNU_SOURCE -g -fsanitize=$(SAN) -I../include -I../src -I.
LDFLAGS = -fsanitize=$(SAN)

TEST_SRCS = main.c log_capture.c test_tune.c test_direct.c test_mode.c
LIB_SRCS = ../src/hal_tune.c ../src/hal_direct.c ../src/hal_mode.c

tests: $(TEST_SRCS) $(LIB_SRCS) ../src/hal_tune.h ../src/hal_direct.h ../src/hal_mode.h ../src/hal_log.h log_capture.h greatest.h
	$(CC) $(CFLAGS) -o $@ $(TEST_SRCS) $(LIB_SRCS) $(LDFLAGS)

test: tests
	./tests

clean:
	rm -f tests

.PHONY: test clean
