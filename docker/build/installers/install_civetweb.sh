#!/usr/bin/env bash

set -e

CURRENT_PATH=$(cd $(dirname $0) && pwd)

INSTALL_PREFIX="$CURRENT_PATH/../third_party/install/civetweb"

# https://github.com/civetweb/civetweb/archive/v1.11.tar.gz

# https://cloud.tencent.com/developer/article/1913473

# need two minutes

cd /tmp
rm -rf civetweb

cd $CURRENT_PATH
cd ..
cd third_party

# git clone --depth 1 https://github.com/civetweb/civetweb.git

cd civetweb
mkdir buildx && cd buildx
# cd buildx

cmake .. \
  -DCIVETWEB_ENABLE_CXX=ON \
  -DBUILD_SHARED_LIBS=ON \
  -DCMAKE_INSTALL_PREFIX=./../../install/civetweb \
  -DCIVETWEB_ENABLE_WEBSOCKETS=ON \
  -DBUILD_TESTING=OFF \
  -DCMAKE_INSTALL_RPATH="\$ORIGIN"

make -j$(nproc)
sudo make install
cd .. && rm -rf build

sudo ldconfig

