#!/usr/bin/env bash

# sudo apt-get install -y nlohmann-json3-dev

set -e

CURRENT_PATH=$(cd $(dirname $0) && pwd)

# INSTALL_PREFIX="$CURRENT_PATH/../third_party/install/json"
INSTALL_PREFIX="/usr/local"

cd $CURRENT_PATH
cd ..
cd third_party

# git clone --depth 1 --branch v3.11.3 https://github.com/nlohmann/json.git
cd json-3.11.3

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

