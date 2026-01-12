#!/usr/bin/env bash

# sudo apt-get install -y libgtest-dev libgmock-dev

set -e

CURRENT_PATH=$(cd $(dirname $0) && pwd)

# INSTALL_PREFIX="$CURRENT_PATH/../third_party/install/googletest"
INSTALL_PREFIX="/usr/local"

cd $CURRENT_PATH
cd ..
cd third_party

# git clone --depth 1 --branch release-1.10.0 https://github.com/google/googletest.git
cd googletest-release-1.10.0

mkdir -p build && cd build

cmake .. \
  -DCMAKE_CXX_FLAGS="-fPIC" \
  -DCMAKE_CXX_FLAGS="-w" \
  -DBUILD_SHARED_LIBS=ON \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_RPATH="\$ORIGIN"

make -j$(nproc)
sudo make install
cd .. && rm -rf build

sudo ldconfig

