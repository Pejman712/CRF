#!/bin/bash

# Enable Google Test
sed -i -E -e 's/(crf_option\(BUILD_UnitTests.*).(OFF)\)/\1 ON)/g' CMakeLists.txt

# Build process
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Debug ../
make -j$((`nproc`))
