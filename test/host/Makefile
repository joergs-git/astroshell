# Host-side logic tests for domecontrol_JK3.ino
#   make test            build and run both site profiles
#   make test FW=path    test another firmware file (e.g. an older revision)
#   ./build/test_site1 <filter>   run only scenarios whose name contains <filter>
CXX      ?= c++
CXXFLAGS ?= -std=c++17 -O1 -g -Wall -Wextra -Wno-unused-parameter -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable
FW       ?= ../../domecontrol_JK3.ino
DEPS      = test_dome.cpp $(FW) $(wildcard mock/*.h) $(wildcard mock/*/*.h)

all: test

build/test_site1: $(DEPS)
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -Imock -DSITE=1 -DFIRMWARE_PATH='"$(FW)"' test_dome.cpp -o $@

build/test_site2: $(DEPS)
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -Imock -DSITE=2 -DFIRMWARE_PATH='"$(FW)"' test_dome.cpp -o $@

test: build/test_site1 build/test_site2
	./build/test_site1
	./build/test_site2

clean:
	rm -rf build

.PHONY: all test clean
