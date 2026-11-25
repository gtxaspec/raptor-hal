CC ?= gcc
SAN ?= address
CFLAGS = -Wall -Wextra -std=c11 -D_GNU_SOURCE -g -fsanitize=$(SAN) -I../include -I../src -I.
LDFLAGS = -fsanitize=$(SAN)

TEST_SRCS = main.c test_tune.c
LIB_SRCS = ../src/hal_tune.c

tests: $(TEST_SRCS) $(LIB_SRCS) ../src/hal_tune.h greatest.h
	$(CC) $(CFLAGS) -o $@ $(TEST_SRCS) $(LIB_SRCS) $(LDFLAGS)

test: tests
	./tests

clean:
	rm -f tests

.PHONY: test clean
