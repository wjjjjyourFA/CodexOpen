#!/usr/bin/env bash

set -e

SCRIPT_DIR=$(cd $(dirname $0) && pwd)

THIRD_PARTY_DIR="${SCRIPT_DIR}/../../third_party"
cd "${THIRD_PARTY_DIR}"

# DEST_DIR="$CURRENT_PATH/../../third_party/install/Fast-DDS"
# DEST_DIR="/opt/CodexOpen/Fast-DDS"
# DEST_DIR="/usr/local/fast-dds"
DEST_DIR=${SYSROOT_DIR}

VERSION="2.6.6"

if [ ! -d "fast-dds-${VERSION}" ]; then
    git clone --depth 1 --branch v${VERSION} \
        https://github.com/eProsima/Fast-DDS.git \
        "${THIRD_PARTY_DIR}/fast-dds-${VERSION}"
fi

pushd "fast-dds-${VERSION}"
  mkdir -p build && cd build
  cmake .. \
    -DBUILD_SHARED_LIBS=ON \
    -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_RPATH="\$ORIGIN" \
    -DCMAKE_CXX_STANDARD=11 \
    -DCMAKE_PREFIX_PATH="${SYSROOT_DIR}/fast-cdr;${SYSROOT_DIR}/foonathan_memory/lib/foonathan_memory/cmake" \
  make -j12
  make install
popd

ldconfig

# cd .. && rm -rf build