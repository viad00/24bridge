#/bin/sh
export PATH=$PATH:/home/vlad/openwrt/staging_dir/toolchain-mipsel_24kc_gcc-7.5.0_musl/bin
export STAGING_DIR=/home/vlad/openwrt/staging_dir/

rm -rf build 
cmake -B build -DCMAKE_TOOLCHAIN_FILE=Toolchain.cmake
cd build
make -j12
