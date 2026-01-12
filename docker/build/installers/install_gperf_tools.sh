#!/usr/bin/env bash

set -e

CURRENT_PATH=$(cd $(dirname $0) && pwd)

INSTALL_PREFIX="$CURRENT_PATH/../third_party/install/gperftools"

cd $CURRENT_PATH
cd ..
cd third_party

# git clone --depth 1 --branch  gperftools-2.8 https://github.com/gperftools/gperftools.git

cd gperftools

./autogen.sh || sleep 1 && ./autogen.sh
./configure --disable-libunwind --prefix="${INSTALL_PREFIX}" --libdir="${INSTALL_PREFIX}/lib"

make -j12
sudo make install