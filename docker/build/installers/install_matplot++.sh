#!/usr/bin/env bash

# 二进制安装
# sudo apt-get install libcv-dev
# sudo apt-get install libopencv-dev


# another way:源码安装，默认的程序无法编译，需要修改一些值
set -e

CURRENT_PATH=$(cd $(dirname $0) && pwd)

INSTALL_PREFIX="$CURRENT_PATH/../third_party/install/matplot++"

# 这是一个c++ 17的库，暂时不要使用
sudo apt install gnuplot

cd $CURRENT_PATH
cd ..
cd third_party

git clone --depth=1 https://github.com/alandefreitas/matplotplusplus.git

cd matplotplusplus && mkdir build && cd build

cmake .. \
  -DCMAKE_INSTALL_PREFIX=~/easy_apollo/third_party/install/matplot++ \
  -DMATPLOTPP_BUILD_EXAMPLES=OFF \
  -DMATPLOTPP_BUILD_SHARED_LIBS=ON \
  -DMATPLOTPP_BUILD_TESTS=OFF \
  -DCMAKE_INTERPROCEDURAL_OPTIMIZATION=ON \
  -DCMAKE_INSTALL_RPATH="\$ORIGIN"

make -j$(nproc)
sudo make install
cd .. && rm -rf build
