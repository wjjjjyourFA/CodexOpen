#!/usr/bin/env bash

set -e

CURRENT_PATH=$(cd $(dirname $0) && pwd)

# INSTALL_PREFIX="$CURRENT_PATH/../third_party/install/pangolin"
INSTALL_PREFIX="/usr/local"

cd $CURRENT_PATH
cd ..
cd third_party

cd Pangolin-0.8

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