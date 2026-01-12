#!/usr/bin/env bash

set -e

CURRENT_PATH=$(cd $(dirname $0) && pwd)

INSTALL_PREFIX="$CURRENT_PATH/../third_party/install/Fast-CDR"

cd $CURRENT_PATH
cd ..
cd third_party

# git clone --depth 1 --branch v2.2.2 https://github.com/eProsima/Fast-CDR.git

cd Fast-CDR
mkdir -p build && cd build

cmake .. \
  -DBUILD_SHARED_LIBS=ON \
  -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_RPATH="\$ORIGIN"

make -j12
sudo make install
cd .. && rm -rf build