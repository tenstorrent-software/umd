# Every variable in subdir must be prefixed with subdir (emulating a namespace)

TEST_APP_CFLAGS = $(CFLAGS) -Werror
TEST_APP_INCLUDES = -I$(UMD_HOME) -I$(UMD_HOME)/device $(DEVICE_INCLUDES)

TEST_APP_FILES = $(basename $(wildcard $(UMD_HOME)/test_app/*.c*))

TEST_APP_SRCS = $(addsuffix .cpp, $(TEST_APP_FILES))

TEST_APP_LDFLAGS = -L$(LIBDIR) -lyaml-cpp -lhwloc -lgtest -lgtest_main -lpthread -lstdc++fs

# #build emulation tests separately
# ifeq ($(EMULATION_DEVICE_EN),1)
#   EMULATION_UNIT_TESTS += $(basename $(wildcard $(UMD_HOME)/tests/emulation/*.c*))
#   EMULATION_UNIT_TESTS_SRCS = $(addsuffix .cpp, $(EMULATION_UNIT_TESTS))

#   EMULATION_LDFLAGS += -L$(ZEBU_IP_ROOT)/lib -L$(ZEBU_ROOT)/lib -LDFLAGS "-g"
#   EMULATION_LIBS += -lxtor_amba_master_svs -lZebuXtor -lZebu -lZebuZEMI3 -lZebuVpi \
#                     -Wl,-rpath,$(ZEBU_IP_ROOT)/lib\
#                     $(OUT)/zcui.work/zebu.work/xtor_amba_master_axi4_svs.so\
#                     $(OUT)/zcui.work/zebu.work/tt_emu_cmd_xtor.so\
#                     $(TENSIX_EMULATION_ZEBU)/lib/libtt_zebu_wrapper.so

#   TEST_APP_LDFLAGS += $(EMULATION_LDFLAGS) $(EMULATION_LIBS)
  
# endif

DEVICE_CXX ?= /usr/bin/clang++-6.0

# Each module has a top level target as the entrypoint which must match the subdir name
test_app: $(OUT)/test_app

.PHONY: $(OUT)/test_app
$(OUT)/test_app: $(UMD_DEVICE_LIB)
	@mkdir -p $(@D)
	$(DEVICE_CXX) $(TEST_APP_CFLAGS) $(CXXFLAGS) $(TEST_APP_INCLUDES) $(TEST_APP_SRCS) -o $@ $^ $(LDFLAGS) $(TEST_APP_LDFLAGS)
