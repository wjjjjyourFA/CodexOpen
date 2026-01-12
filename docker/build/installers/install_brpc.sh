#!/usr/bin/env bash

set -e

CURRENT_PATH=$(cd $(dirname $0) && pwd)

INSTALL_PREFIX="$CURRENT_PATH/../third_party/install/brpc"

cd $CURRENT_PATH
cd ..
cd third_party

# git clone --depth 1 --branch 1.3.0 https://github.com/apache/brpc.git

cd brpc
mkdir -p build && cd build

cmake .. \
  -DBUILD_SHARED_LIBS=ON \
  -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_RPATH="\$ORIGIN" \
  -DCMAKE_PREFIX_PATH="$CURRENT_PATH/../third_party/install/protobuf" \
  -DProtobuf_DIR="$CURRENT_PATH/../third_party/install/protobuf/lib/cmake/protobuf" \
  -DProtobuf_INCLUDE_DIR="$CURRENT_PATH/../third_party/install/protobuf/include" \
  -DProtobuf_LIBRARIES="$CURRENT_PATH/../third_party/install/protobuf/lib/libprotobuf.so" \
  -DWITH_GLOG=ON -DWITH_GFLAGS=ON -DWITH_BUILTIN_GLOG=OFF \
  -DCMAKE_CXX_FLAGS="-D_GLIBCXX_USE_CXX11_ABI=1"

make -j12
sudo make install
cd .. && rm -rf build