# Host tests of code that does not need the SDK: make -C tests
CXX ?= g++
CXXFLAGS ?= -O2
CXXFLAGS += -std=c++11 -Wall -Wextra -Werror -I../src

TESTS := test_yolo

.PHONY: all run clean
all: run

test_yolo: test_yolo.cpp ../src/hal_ivs_yolo.h
	$(CXX) $(CXXFLAGS) $< -o $@

run: $(TESTS)
	@for t in $(TESTS); do ./$$t || exit 1; done

clean:
	rm -f $(TESTS)
