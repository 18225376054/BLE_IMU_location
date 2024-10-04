# toolchain.cmake

# 指定交叉编译工具链
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

# 指定编译器路径
set(CMAKE_C_COMPILER /home/c606/jieryyyyy/gcc/gcc-linaro-7.5.0-2019.12-i686_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-gcc)
set(CMAKE_CXX_COMPILER /home/c606/jieryyyyy/gcc/gcc-linaro-7.5.0-2019.12-i686_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-g++)
set(CMAKE_LINKER /home/c606/jieryyyyy/gcc/gcc-linaro-7.5.0-2019.12-i686_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-ld)
set(CMAKE_ASM_COMPILER /home/c606/jieryyyyy/gcc/gcc-linaro-7.5.0-2019.12-i686_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-as)

# 指定 sysroot (如果有的话)
set(CMAKE_SYSROOT /home/c606/jieryyyyy/gcc/gcc-linaro-7.5.0-2019.12-i686_arm-linux-gnueabihf/arm-linux-gnueabihf/libc)

# 指定查找程序和库的路径
set(CMAKE_FIND_ROOT_PATH /home/c606/jieryyyyy/gcc/gcc-linaro-7.5.0-2019.12-i686_arm-linux-gnueabihf)

# Adjust the search paths for programs, libraries, and headers
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
