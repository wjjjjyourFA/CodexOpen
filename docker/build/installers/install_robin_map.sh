#!/usr/bin/env bash

set -e

SCRIPT_DIR=$(cd $(dirname $0) && pwd)

THIRD_PARTY_DIR="${SCRIPT_DIR}/../../third_party"
cd "${THIRD_PARTY_DIR}"

# DEST_DIR="$CURRENT_PATH/../../third_party/install/robin_map"
DEST_DIR="/usr/local"

VERSION="1.4.1"

if [ ! -d "robin-map-${VERSION}" ]; then
    git clone --depth 1 --branch v${VERSION} \
        https://github.com/Tessil/robin-map.git \
        "${THIRD_PARTY_DIR}/robin-map-${VERSION}"
fi

cd robin-map-${VERSION}

mkdir -p build && cd build

cmake .. \
  -DBUILD_SHARED_LIBS=ON \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DCMAKE_INSTALL_PREFIX=${DEST_DIR} \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_RPATH="\$ORIGIN"

make -j12
make install

ldconfig

# cd .. && rm -rf build