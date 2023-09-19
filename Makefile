
CONFIG ?= release
UMD_HOME ?= .
OUT ?= $(UMD_HOME)/build
LIBDIR ?= $(OUT)/lib
ARCH_NAME ?= grayskull
OBJDIR ?= $(OUT)/obj
BINDIR ?= $(OUT)/bin
INCDIR ?= $(OUT)/include
TESTDIR ?= $(OUT)/test
DOCSDIR ?= $(OUT)/docs
UMD_VERSIM_STUB ?= 1
SHARED_LIB_FLAGS ?= -shared -fPIC
STATIC_LIB_FLAGS ?= -fPIC

TEST_TARGETS ?= device/tests
ifeq ($(ARCH_NAME), wormhole_b0)
  TEST_TARGETS += device/tests/galaxy
endif

#MAKEFLAGS := --jobs=$(shell nproc)

DEVICE_CXX = /usr/bin/g++
CXXFLAGS = -MMD -I$(UMD_HOME)/. --std=c++17
ifeq ($(CONFIG), release)
CXXFLAGS += -O3
else ifeq ($(CONFIG), ci)
CXXFLAGS += -O3  # significantly smaller artifacts
else ifeq ($(CONFIG), assert)
CXXFLAGS += -O3 -g
else ifeq ($(CONFIG), asan)
CXXFLAGS += -O3 -g -fsanitize=address
else ifeq ($(CONFIG), ubsan)
CXXFLAGS += -O3 -g -fsanitize=undefined
else ifeq ($(CONFIG), debug)
CXXFLAGS += -O0 -g -DDEBUG
else
$(error Unknown value for CONFIG "$(CONFIG)")
endif
UMDHEADERS := device/device_api.h


include device/module.mk
include tests/module.mk

init:
	echo $(LIBDIR)
	mkdir -p $(OUT)
	mkdir -p $(INCDIR)
	mkdir -p $(LIBDIR)
	cp $(UMDHEADERS) $(INCDIR)
build: init umd_device

test: build $(TEST_TARGETS)
run: test
	LD_LIBRARY_PATH=$(LIBDIR) ./$(OUT)/tests/device_unit_tests --gtest_catch_exceptions
run-galaxy: test
	LD_LIBRARY_PATH=$(LIBDIR) ./$(OUT)/tests/galaxy_unit_tests

clean:
	rm -rf $(OUT)
