#!/usr/bin/env bash

set -e

CURRENT_PATH=$(cd $(dirname $0) && pwd)

# INSTALL_PREFIX="$CURRENT_PATH/../third_party/install/gtsam"
INSTALL_PREFIX="/usr/local"

cd $CURRENT_PATH
cd ..
cd third_party

# git clone --depth 1 --branch 4.3a0 https://github.com/borglab/gtsam.git
cd gtsam-4.3a0

mkdir -p build && cd build

# ubuntu20.04 ==> libeigen3-dev 3.3.7
cmake .. \
  -DBUILD_SHARED_LIBS=ON \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_RPATH="\$ORIGIN" \
  -DCMAKE_BUILD_RPATH_USE_ORIGIN=ON \
  -DCMAKE_SKIP_BUILD_RPATH=FALSE \
  -DGTSAM_BUILD_TESTS=OFF \
  -DGTSAM_INSTALL_CPPUNITLITE=OFF \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
  -DGTSAM_USE_SYSTEM_EIGEN=ON \
  -DEigen3_INCLUDE_DIR=/usr/include/eigen3
  # -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# check
# grep -i eigen CMakeCache.txt

make -j12
sudo make install
cd .. && rm -rf build

sudo ldconfig

