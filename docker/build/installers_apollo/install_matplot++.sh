#!/usr/bin/env bash

set -e

# 二进制安装
# apt-get install libcv-dev
# apt-get install libopencv-dev

# another way:源码安装，默认的程序无法编译，需要修改一些值

SCRIPT_DIR=$(cd $(dirname $0) && pwd)

THIRD_PARTY_DIR="${SCRIPT_DIR}/../../third_party"
cd "${THIRD_PARTY_DIR}"

# DEST_DIR="$CURRENT_PATH/../../third_party/install/matplot++"
# DEST_DIR="/opt/CodexOpen/matplot++"
DEST_DIR=${SYSROOT_DIR}

# 这是一个c++ 17的库，暂时不要使用
apt-get install -y gnuplot

git clone --depth=1 https://github.com/alandefreitas/matplotplusplus.git

cd matplotplusplus
mkdir -p build && cd build

cmake .. \
  -DCMAKE_INSTALL_PREFIX="${DEST_DIR}" \
  -DMATPLOTPP_BUILD_EXAMPLES=OFF \
  -DMATPLOTPP_BUILD_SHARED_LIBS=ON \
  -DMATPLOTPP_BUILD_TESTS=OFF \
  -DCMAKE_INTERPROCEDURAL_OPTIMIZATION=ON \
  -DCMAKE_INSTALL_RPATH="\$ORIGIN"

make -j$(nproc)
make install

ldconfig

# cd .. && rm -rf build
