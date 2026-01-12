#!/usr/bin/env bash

set -e

CURRENT_PATH=$(cd $(dirname $0) && pwd)

INSTALL_PREFIX="$CURRENT_PATH/../third_party/install/Fast-DDS"

cd $CURRENT_PATH
cd ..
cd third_party

# git clone --depth 1 --branch v2.14.3 https://github.com/eProsima/Fast-DDS.git

cd Fast-DDS
mkdir -p build && cd build

cmake .. \
  -DBUILD_SHARED_LIBS=ON \
  -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_RPATH="\$ORIGIN" \
  -DCMAKE_CXX_STANDARD=11 \
  -DCMAKE_PREFIX_PATH="$CURRENT_PATH/../third_party/install/Fast-CDR;$CURRENT_PATH/../third_party/install/foonathan_memory/lib/foonathan_memory/cmake" \

make -j12
sudo make install
cd .. && rm -rf build