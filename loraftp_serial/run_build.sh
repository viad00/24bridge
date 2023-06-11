#/bin/sh

rm -rf build 
cmake -B build #-DCMAKE_TOOLCHAIN_FILE=Toolchain.cmake
cd build
make -j12
