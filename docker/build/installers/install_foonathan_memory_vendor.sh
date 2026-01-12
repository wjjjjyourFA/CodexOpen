#!/usr/bin/env bash

set -e

CURRENT_PATH=$(cd $(dirname $0) && pwd)

INSTALL_PREFIX="$CURRENT_PATH/../third_party/install/foonathan_memory_vendor"

cd $CURRENT_PATH
cd ..
cd third_party

# git clone --depth 1 --branch v1.3.1 https://github.com/eProsima/foonathan_memory_vendor.git

cd foonathan_memory_vendor
mkdir -p build && cd build

cmake .. \
  -DBUILD_SHARED_LIBS=ON \
  -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_STANDARD=17 \
  -DCMAKE_INSTALL_RPATH="\$ORIGIN"

make -j12
sudo make install
cd .. && rm -rf build