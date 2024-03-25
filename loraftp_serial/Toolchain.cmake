SET(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR mips)
SET(LINUX_ARM TRUE)

set(CMAKE_CROSSCOMPILING TRUE)
#set(CMAKE_SYSROOT /home/vlad/x-tools/mipsel-unknown-linux-musl/mipsel-unknown-linux-musl/sysroot)
#set(CROSS_COMPILER_ROOT /home/vlad/x-tools/mipsel-unknown-linux-musl/bin/)
#set(CROSS_COMPILER_PREFIX "mipsel-unknown-linux-musl-")

#SET (CMAKE_C_COMPILER_WORKS 1)
#SET (CMAKE_CXX_COMPILER_WORKS 1)

set(CMAKE_SYSROOT /home/vlad/openwrt/staging_dir/target-mipsel_24kc_musl/root-ramips/)
set(CROSS_COMPILER_ROOT /home/vlad/openwrt/staging_dir/toolchain-mipsel_24kc_gcc-7.5.0_musl/bin/)
set(CROSS_COMPILER_PREFIX "mipsel-openwrt-linux-musl-")
set(STAGING_DIR "/home/vlad/openwrt/staging_dir/target-mipsel_24kc_musl")
#set(CMAKE_SYSROOT /home/vlad/Ingenic-SDK-T31-1.1.1-20200508/toolchain/gcc_472/mips-gcc472-glibc216-64bit/mips-linux-gnu/libc/)
#set(CROSS_COMPILER_ROOT /home/vlad/Ingenic-SDK-T31-1.1.1-20200508/toolchain/gcc_472/mips-gcc472-glibc216-64bit/bin/)
#set(CROSS_COMPILER_PREFIX "mips-linux-uclibc-gnu-")

# specify the cross compiler
SET(CMAKE_C_COMPILER ${CROSS_COMPILER_ROOT}${CROSS_COMPILER_PREFIX}gcc)
SET(CMAKE_CXX_COMPILER ${CROSS_COMPILER_ROOT}${CROSS_COMPILER_PREFIX}g++)

#set(CMAKE_VERBOSE_MAKEFILE TRUE)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static-libstdc++")

SET(CMAKE_LIBRARY_PATH=${CMAKE_SYSROOT}/usr/lib)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
