#!/usr/bin/env bash

set -e

SCRIPT_DIR=$(cd $(dirname $0) && pwd)

THIRD_PARTY_DIR="${SCRIPT_DIR}/../../third_party"
cd "${THIRD_PARTY_DIR}"

# DEST_DIR="$CURRENT_PATH/../../third_party/install/foonathan_memory"
# DEST_DIR="/opt/CodexOpen/foonathan_memory"
DEST_DIR=${SYSROOT_DIR}

VERSION="0.7-4"

if [ ! -d "foonathan_memory-${VERSION}" ]; then
    git clone --depth 1 --branch ${VERSION} \
        https://github.com/eProsima/foonathan_memory.git \
        "${THIRD_PARTY_DIR}/foonathan_memory-${VERSION}"
fi

cd foonathan_memory-${VERSION}
mkdir -p build && cd build

cmake .. \
  -DBUILD_SHARED_LIBS=ON \
  -DCMAKE_INSTALL_PREFIX=$DEST_DIR \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_RPATH="\$ORIGIN"

make -j12
make install

# cd .. && rm -rf build