################################################################################
# Real-Time Locationing library component                                      #
################################################################################

ifeq (, $(filter $(MAKECMDGOALS), clean export help))
  ifeq (, $(filter $(UNAME), darwin linux))
    ifneq ($(CC), x86_64-w64-mingw32-gcc)
      $(error Toolchain not supported by RTL lib.)
    endif
  endif
endif

RTL_DIR = $(SDK_DIR)/util/silicon_labs/aox

DEVICE := x64
ifneq ($(filter arm%, $(shell uname -m)),)
  DEVICE := cortexa
endif

DEVICE := cortexa

override INCLUDEPATHS += $(RTL_DIR)/inc

override CFLAGS += -DRTL_LIB

ifeq ($(OS),posix)
override LDFLAGS += \
-L$(RTL_DIR)/lib/$(UNAME)_$(DEVICE)/gcc/release \
-laox_static \
-lm \
-lGeographic  -L/home/c606/jieryyyyy/geo/GeographicLib/GeographicLib/lib\
-lstdc++ \
-lpthread
else
override LDFLAGS += \
"$(RTL_DIR)/lib/windows_x64/gcc/release/libaox_static.a" \
-lstdc++ \
-lpthread
endif

PROJ_FILES += $(wildcard $(RTL_DIR)/lib/*/gcc/release/libaox_static.a)
