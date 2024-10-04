################################################################################
# General project settings                                                     #
################################################################################

# PROJECTNAME = lablognlos3
PROJECTNAME = lablognlos
SDK_DIR = ../../../..

APP_MODE ?= conn_less
CONFIG ?= default
ANGLE ?= 1


################################################################################
# Components                                                                   #
#                                                                              #
# After setting up the toolchain, components can contribute to the project by  #
# appending items to the project variables like INCLUDEPATHS or C_SRC.         #
################################################################################

include $(SDK_DIR)/app/bluetooth/component_host/toolchain.mk
include $(SDK_DIR)/app/bluetooth/component_host/app_log.mk
include $(SDK_DIR)/app/bluetooth/component_host/app_assert.mk
include $(SDK_DIR)/app/bluetooth/component_host/cjson.mk
include $(SDK_DIR)/app/bluetooth/component_host/ncp_host_nothread.mk
ifneq ($(ANGLE), 0)
include $(SDK_DIR)/app/bluetooth/component_host/aoa_angle.mk
include $(SDK_DIR)/app/bluetooth/component_host/rtl_lib.mk
endif




################################################################################
# Include paths        
#  /usr/src/linux-hwe-5.4-headers-5.4.0-84/include\
# /usr/src/linux-hwe-5.4-headers-5.4.0-152/arch/arm/include\
# /usr/src/linux-hwe-5.4-headers-5.4.0-152/arch/                                                       #
################################################################################

override INCLUDEPATHS += . \
$(SDK_DIR)/app/bluetooth/common/ncp_evt_filter \
$(SDK_DIR)/app/bluetooth/common_host/aoa_config/$(CONFIG) \
$(SDK_DIR)/app/bluetooth/common_host/aoa_util \
$(SDK_DIR)/app/bluetooth/common_host/app_signal \
$(SDK_DIR)/app/bluetooth/common_host/system \
$(SDK_DIR)/platform/common/inc \
$(SDK_DIR)/protocol/bluetooth/inc\
$(SDK_DIR)/app/bluetooth/example_host/aoa_locator/mpu9250/inc\
/home/c606/jieryyyyy/lib_imx/linux-imx-rel_imx_4.1.15_2.1.0_ga_alientek/arch/arm/include\
/home/c606/jieryyyyy/lib_imx/linux-imx-rel_imx_4.1.15_2.1.0_ga_alientek/arch\
/home/c606/jieryyyyy/lib_imx/linux-imx-rel_imx_4.1.15_2.1.0_ga_alientek/arch/arm/include/generated


################################################################################
# Input files                                                                  #
################################################################################
# 修改 定义变量，存储 C++ 源文件的路径
# override CXX_SRC += main.cpp



override C_SRC += \
$(SDK_DIR)/app/bluetooth/common_host/aoa_config/$(CONFIG)/aoa_config.c \
$(SDK_DIR)/app/bluetooth/common_host/aoa_util/aoa_parse.c \
$(SDK_DIR)/app/bluetooth/common_host/aoa_util/aoa_serdes.c \
$(SDK_DIR)/app/bluetooth/common_host/aoa_util/aoa_util.c \
$(SDK_DIR)/app/bluetooth/common_host/app_signal/app_signal_$(OS).c \
$(SDK_DIR)/app/bluetooth/common_host/system/system.c \
app.c \
app_$(APP_MODE).c \
conn.c \
my_udp.c \
nedToGeograph.c \
nlos.c\
main.c\
$(SDK_DIR)/app/bluetooth/example_host/aoa_locator/mpu9250/src/driver_mpu9250_basic.c\
$(SDK_DIR)/app/bluetooth/example_host/aoa_locator/mpu9250/src/driver_mpu9250_interface.c\
$(SDK_DIR)/app/bluetooth/example_host/aoa_locator/mpu9250/src/driver_mpu9250_dmp.c\
$(SDK_DIR)/app/bluetooth/example_host/aoa_locator/mpu9250/src/driver_mpu9250.c\
$(SDK_DIR)/app/bluetooth/example_host/aoa_locator/mpu9250/src/iic.c\
$(SDK_DIR)/app/bluetooth/example_host/aoa_locator/mpu9250/src/mpu_init.c\
$(SDK_DIR)/app/bluetooth/example_host/aoa_locator/mpu9250/src/mutex.c\
$(SDK_DIR)/app/bluetooth/example_host/aoa_locator/mpu9250/src/spi.c\
$(SDK_DIR)/app/bluetooth/example_host/aoa_locator/mpu9250/src/gpio.c\
kalman.c\
kalman2D.c\

ifeq ($(OS), win)
# List of .dll files required by mosquitto.
override LIBS += \
$(EXE_DIR)/mosquitto.dll \
$(EXE_DIR)/libcrypto-1_1-x64.dll \
$(EXE_DIR)/libssl-1_1-x64.dll
endif


################################################################################
# Target rules                                                                 #
################################################################################

include $(SDK_DIR)/app/bluetooth/component_host/targets.mk

# Copy .dll files next to the executable to avoid missing DLL errors.
$(EXE_DIR)/%.dll:
	$(shell cp "${MOSQUITTO_DIR}/$*.dll" $(EXE_DIR))
