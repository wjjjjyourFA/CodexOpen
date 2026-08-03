#!/usr/bin/env bash

set -e

SCRIPT_DIR=$(cd $(dirname $0) && pwd)

THIRD_PARTY_DIR="${SCRIPT_DIR}/../../third_party"
cd "${THIRD_PARTY_DIR}"

# DEST_DIR="$CURRENT_PATH/../../third_party/install/pangolin"
DEST_DIR="/usr/local"

VERSION="0.8"

pushd "Pangolin-${VERSION}"
    mkdir -p build && cd build
    cmake .. \
        -DBUILD_SHARED_LIBS=ON \
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
        -DCMAKE_INSTALL_PREFIX=${DEST_DIR} \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_RPATH="\$ORIGIN" \
        -DBUILD_PANGOLIN_VIDEO=OFF \
        -DBUILD_PANGOLIN_OPENNI=OFF \
        -DBUILD_PANGOLIN_OPENNI2=OFF \
        -DBUILD_PANGOLIN_PYTHON=OFF \
        # -DOpenNI2_LIBRARY=/usr/lib/x86_64-linux-gnu/libOpenNI2.so \
        # -DOpenNI2_INCLUDE_DIR=/usr/include/openni2
    make -j12
    make install
popd

ldconfig

# cd .. && rm -rf build