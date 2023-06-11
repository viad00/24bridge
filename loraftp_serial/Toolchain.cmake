SET(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR mipsel)
SET(LINUX_ARM TRUE)

set(CMAKE_CROSSCOMPILING TRUE)
#set(CMAKE_SYSROOT /home/vlad/x-tools/mipsel-unknown-linux-musl/mipsel-unknown-linux-musl/sysroot)
#set(CROSS_COMPILER_ROOT /home/vlad/x-tools/mipsel-unknown-linux-musl/bin/)
#set(CROSS_COMPILER_PREFIX "mipsel-unknown-linux-musl-")

set(CMAKE_SYSROOT /home/vlad/openwrt/staging_dir/target-mipsel_24kc_musl/root-ramips/)
set(CROSS_COMPILER_ROOT /home/vlad/openwrt/staging_dir/toolchain-mipsel_24kc_gcc-7.3.0_musl/bin/)
set(CROSS_COMPILER_PREFIX "mipsel-openwrt-linux-musl-")
#set(STAGING_DIR "/home/vlad/openwrt/staging_dir/target-mipsel_24kc_musl")

# specify the cross compiler
SET(CMAKE_C_COMPILER ${CROSS_COMPILER_ROOT}${CROSS_COMPILER_PREFIX}gcc)
SET(CMAKE_CXX_COMPILER ${CROSS_COMPILER_ROOT}${CROSS_COMPILER_PREFIX}g++)

SET(CMAKE_LIBRARY_PATH=${CMAKE_SYSROOT}/usr/lib)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
