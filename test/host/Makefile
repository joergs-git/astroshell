# Host-side logic tests for domecontrol_JK3.ino
#   make test            build and run both site profiles in three sensor configurations:
#                        default (as flashed), sensors (ToF + DS18B20 built in), bare (both compiled out)
#   make test FW=path    test another firmware file (e.g. an older revision)
#   ./build/test_site2_sensors <filter>   run only scenarios whose name contains <filter>
CXX      ?= c++
CXXFLAGS ?= -std=c++17 -O1 -g -Wall -Wextra -Wno-unused-parameter -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable
FW       ?= ../../domecontrol_JK3.ino
DEPS      = test_dome.cpp $(FW) $(wildcard mock/*.h) $(wildcard mock/*/*.h)
SENSORS   = -DENABLE_TOF_FROZEN_DETECTION=1 -DENABLE_DS18B20_DYNAMIC_TIMEOUT=1
BARE      = -DENABLE_TOF_FROZEN_DETECTION=0 -DENABLE_DS18B20_DYNAMIC_TIMEOUT=0
BINS      = build/test_site1 build/test_site1_sensors build/test_site1_bare \
            build/test_site2 build/test_site2_sensors build/test_site2_bare

all: test

build/test_site1: $(DEPS)
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -Imock -DSITE=1 -DFIRMWARE_PATH='"$(FW)"' test_dome.cpp -o $@
build/test_site1_sensors: $(DEPS)
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -Imock -DSITE=1 $(SENSORS) -DFIRMWARE_PATH='"$(FW)"' test_dome.cpp -o $@
build/test_site1_bare: $(DEPS)
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -Imock -DSITE=1 $(BARE) -DFIRMWARE_PATH='"$(FW)"' test_dome.cpp -o $@
build/test_site2: $(DEPS)
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -Imock -DSITE=2 -DFIRMWARE_PATH='"$(FW)"' test_dome.cpp -o $@
build/test_site2_sensors: $(DEPS)
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -Imock -DSITE=2 $(SENSORS) -DFIRMWARE_PATH='"$(FW)"' test_dome.cpp -o $@
build/test_site2_bare: $(DEPS)
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -Imock -DSITE=2 $(BARE) -DFIRMWARE_PATH='"$(FW)"' test_dome.cpp -o $@

test: $(BINS)
	@for b in $(BINS); do ./$$b || exit 1; done

clean:
	rm -rf build

.PHONY: all test clean
