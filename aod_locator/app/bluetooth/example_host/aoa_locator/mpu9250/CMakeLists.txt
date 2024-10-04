cmake_minimum_required (VERSION 2.8)
project (onlympu)

set(CMAKE_C_COMPILER /home/c606/jieryyyyy/gcc/gcc-linaro-7.5.0-2019.12-i686_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-gcc)
set(CMAKE_SYSTEM_PROCESSOR arm)
set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
aux_source_directory (src SRC_LIST)
include_directories (inc)
#include_directories (/usr/src/linux-hwe-5.4-headers-5.4.0-84/include)
#include_directories (/usr/src/linux-hwe-5.4-headers-5.4.0-152/arch/arm/include)
#include_directories (/usr/src/linux-hwe-5.4-headers-5.4.0-152/arch/)

# find the pkgconfig and use this tool to find the third party packages

# link_directories(/usr/local/lib)
# link_directories(/home/c606/jieryyyyy/onlyMPU/mpu9250/lib/usr/local/lib)

# 打印 ${GPIOD_INCLUDE_DIRS} 和 ${GPIOD_LIBRARIES} 的值
# message(STATUS "GPIOD_INCLUDE_DIRS: ${GPIOD_INCLUDE_DIRS}")
# message(STATUS "GPIOD_LIBRARIES: ${GPIOD_LIBRARIES}")
include_directories (/home/c606/jieryyyyy/lib_imx/linux-imx-rel_imx_4.1.15_2.1.0_ga_alientek/arch/)
include_directories (/home/c606/jieryyyyy/lib_imx/linux-imx-rel_imx_4.1.15_2.1.0_ga_alientek/arch/arm/include)
include_directories (/home/c606/jieryyyyy/lib_imx/linux-imx-rel_imx_4.1.15_2.1.0_ga_alientek/arch/arm/include/generated)


#绝对路径/home/c606/jieryyyyy/onlyMPU/mpu9250/lib/libgpiod.a
add_executable (${PROJECT_NAME} ${SRC_LIST})
target_link_libraries(${PROJECT_NAME}  
                        m
                        pthread)
