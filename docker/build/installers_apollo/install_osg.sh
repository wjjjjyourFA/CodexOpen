#!/usr/bin/env bash

set -e

# apt install:
apt-get install -y openscenegraph

if false; then

SCRIPT_DIR=$(cd $(dirname $0) && pwd)

THIRD_PARTY_DIR="${SCRIPT_DIR}/../../third_party"
cd "${THIRD_PARTY_DIR}"

# DEST_DIR="$CURRENT_PATH/../../third_party/install/OpenSceneGraph"
# DEST_DIR="/opt/CodexOpen/OpenSceneGraph"
DEST_DIR=${SYSROOT_DIR}

VERSION="OpenSceneGraph-3.6.5"

if [ ! -d "OpenSceneGraph" ]; then
    git clone --depth 1 --branch ${VERSION} \
        https://github.com/openscenegraph/OpenSceneGraph.git \
        "${THIRD_PARTY_DIR}/OpenSceneGraph"
fi

cd OpenSceneGraph
mkdir -p build && cd build

cmake .. \
  -DOSG_TEXT_USE_FONTCONFIG=OFF \
  -DCMAKE_INSTALL_PREFIX=$DEST_DIR \
  -DCMAKE_INSTALL_RPATH="\$ORIGIN"

make -j$(nproc)
make install

ldconfig

# cd .. && rm -rf build

fi