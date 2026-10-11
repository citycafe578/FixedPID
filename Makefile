# Host-side tests for FixedPID. Needs make and a GCC/Clang C++ compiler.
#
#   make test      unit tests (test/test.cpp) + fuzz tests, with ASan + UBSan
#   make unit      unit tests only
#   make fuzz      fuzz tests only        (make fuzz SCALE=10 for a longer run)
#   make lint      compile the library with -Werror
#   make clean

CXX      ?= g++
SAN      := -fsanitize=address,undefined -fno-sanitize-recover=all
WARN     := -Wall -Wextra
CXXFLAGS ?= -std=gnu++11 -O1 -g
SCALE    ?= 1
BUILD    := build

LIB      := src/FixedPID.cpp src/FixedPID.h

.PHONY: all test unit fuzz lint clean
all: test
test: lint unit fuzz

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/unit: $(LIB) test/test.cpp test/host/host_main.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) $(WARN) $(SAN) -Isrc -Itest/host/stubs \
	    src/FixedPID.cpp test/test.cpp test/host/host_main.cpp -o $@

$(BUILD)/fuzz: $(LIB) test/host/fuzz_test.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) $(WARN) $(SAN) -Isrc \
	    src/FixedPID.cpp test/host/fuzz_test.cpp -o $@

unit: $(BUILD)/unit
	@$(BUILD)/unit > $(BUILD)/unit.log; status=$$?; \
	    grep -E "FAIL|Expected|Actual|Total|PASS  :" $(BUILD)/unit.log; \
	    exit $$status

fuzz: $(BUILD)/fuzz
	@$(BUILD)/fuzz $(SCALE)

lint: | $(BUILD)
	$(CXX) -std=c++11 -O2 $(WARN) -Werror -Isrc -c src/FixedPID.cpp -o $(BUILD)/lint.o

clean:
	rm -rf $(BUILD)
