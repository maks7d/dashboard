#!/bin/bash
#TODO : optimiser la compilation pour compiler avec ninja plutot que make

# Vérifie si le 1er argument est "debug"
if [ "$1" = "debug" ]; then
    echo "Debug build"
    BUILD_TYPE="Debug"
else
    echo "Release build"
    BUILD_TYPE="Release"
fi

# Nettoyage et configuration commune
rm -rf build
mkdir build
cd build

cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/gcc-arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE ..
cmake --build . --target all -- -j$(sysctl -n hw.ncpu 2>/dev/null || nproc 2>/dev/null || echo 4)