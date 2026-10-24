# Host-side logic tests for domecontrol_JK3.ino
#   make test            build and run both site profiles, each with and without the ToF layer
#   make test FW=path    test another firmware file (e.g. an older revision)
#   ./build/test_site2_tof <filter>   run only scenarios whose name contains <filter>
CXX      ?= c++
CXXFLAGS ?= -std=c++17 -O1 -g -Wall -Wextra -Wno-unused-parameter -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable
FW       ?= ../../domecontrol_JK3.ino
DEPS      = test_dome.cpp $(FW) $(wildcard mock/*.h) $(wildcard mock/*/*.h)
TOF       = -DENABLE_TOF_FROZEN_DETECTION=1
BINS      = build/test_site1 build/test_site2 build/test_site1_tof build/test_site2_tof

all: test

build/test_site1: $(DEPS)
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -Imock -DSITE=1 -DFIRMWARE_PATH='"$(FW)"' test_dome.cpp -o $@

build/test_site2: $(DEPS)
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -Imock -DSITE=2 -DFIRMWARE_PATH='"$(FW)"' test_dome.cpp -o $@

build/test_site1_tof: $(DEPS)
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -Imock -DSITE=1 $(TOF) -DFIRMWARE_PATH='"$(FW)"' test_dome.cpp -o $@

build/test_site2_tof: $(DEPS)
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -Imock -DSITE=2 $(TOF) -DFIRMWARE_PATH='"$(FW)"' test_dome.cpp -o $@

test: $(BINS)
	./build/test_site1
	./build/test_site2
	./build/test_site1_tof
	./build/test_site2_tof

clean:
	rm -rf build

.PHONY: all test clean
