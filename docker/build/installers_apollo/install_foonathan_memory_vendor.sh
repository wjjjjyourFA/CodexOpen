#!/usr/bin/env bash

set -e

SCRIPT_DIR=$(cd $(dirname $0) && pwd)

THIRD_PARTY_DIR="${SCRIPT_DIR}/../../third_party"
cd "${THIRD_PARTY_DIR}"

# DEST_DIR="$CURRENT_PATH/../../third_party/install/foonathan_memory_vendor"
# DEST_DIR="/opt/CodexOpen/foonathan_memory_vendor"
DEST_DIR=${SYSROOT_DIR}

VERSION="1.3.1"

if [ ! -d "foonathan_memory_vendor-${VERSION}" ]; then
    git clone --depth 1 --branch v${VERSION} \
        https://github.com/eProsima/foonathan_memory_vendor.git \
        "${THIRD_PARTY_DIR}/foonathan_memory_vendor-${VERSION}"
fi

cd foonathan_memory_vendor-${VERSION}
mkdir -p build && cd build

cmake .. \
  -DBUILD_SHARED_LIBS=ON \
  -DCMAKE_INSTALL_PREFIX=$DEST_DIR \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_STANDARD=17 \
  -DCMAKE_INSTALL_RPATH="\$ORIGIN"

make -j12
make install

# cd .. && rm -rf build