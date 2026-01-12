#!/usr/bin/env bash

set -e

# apt install:
# sudo apt-get install openscenegraph

CURRENT_PATH=$(cd $(dirname $0) && pwd)

INSTALL_PREFIX="$CURRENT_PATH/../third_party/install/OpenSceneGraph"

cd /tmp
rm -rf OpenSceneGraph

cd $CURRENT_PATH
cd ..
cd third_party

# git clone --depth 1 --branch OpenSceneGraph-3.6.5 https://github.com/openscenegraph/OpenSceneGraph.git

cd OpenSceneGraph && mkdir -p build && cd build

cmake .. \
  -DOSG_TEXT_USE_FONTCONFIG=OFF \
  -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX \
  -DCMAKE_INSTALL_RPATH="\$ORIGIN"

make -j$(nproc)
sudo make install
cd .. && rm -rf build

sudo ldconfig