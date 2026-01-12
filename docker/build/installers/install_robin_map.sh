#!/usr/bin/env bash

set -e

CURRENT_PATH=$(cd $(dirname $0) && pwd)

# INSTALL_PREFIX="$CURRENT_PATH/../third_party/install/robin_map"
INSTALL_PREFIX="/usr/local"

cd $CURRENT_PATH
cd ..
cd third_party

# git clone --depth 1 --branch v1.4.1 https://github.com/Tessil/robin-map.git
cd robin-map-1.4.1

mkdir -p build && cd build

cmake .. \
  -DBUILD_SHARED_LIBS=ON \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_RPATH="\$ORIGIN"

make -j12
sudo make install
cd .. && rm -rf build

sudo ldconfig