#!/usr/bin/env bash

# tf2-apollo

## Intro

# tf2-apollo was Apollo's clone of ROS tf2 for coordinate transforms.
# It seems that it was based on `ros/geometry2` tagged `0.5.16`

## How to build

set -e

CURRENT_PATH=$(cd $(dirname $0) && pwd)

INSTALL_PREFIX="$CURRENT_PATH/../third_party/install/tf2"

cd ${CURRENT_PATH}
cd ..
cd third_party

cd tf2
rm -rf build
mkdir build && cd build

cmake .. \
  -DCMAKE_INSTALL_PREFIX=../../install/tf2 \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_STANDARD=17 \
  -DBUILD_TESTS=OFF \
  -DCMAKE_INSTALL_RPATH="\$ORIGIN"

make -j6
sudo make install
cd .. && rm -rf build 


## gtest support for tf2-apollo

# In order not to build gtest into our docker image, gtest support for tf2 was
# implemented according to https://crascit.com/2015/07/25/cmake-gtest

## Reference
# - https://github.com/ros/geometry2
# - http://wiki.ros.org/geometry2

