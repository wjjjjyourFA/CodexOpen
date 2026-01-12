#!/usr/bin/env bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"
. ./installer_base.sh

THIRD_PARTY_DIR="${SCRIPT_DIR}/../../third_party"
cd "${THIRD_PARTY_DIR}"

VERSION="11.0.0"

if [ ! -d "tinyxml2-${VERSION}" ]; then
    git clone --depth 1 --branch v${VERSION} \
        https://github.com/leethomason/tinyxml2.git \
        "${THIRD_PARTY_DIR}/tinyxml2-${VERSION}"
fi

DEST_DIR="/usr"

pushd "tinyxml2-${VERSION}"
    mkdir -p build && cd build
    cmake .. \
      -DBUILD_SHARED_LIBS=ON \
      -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
      -DCMAKE_INSTALL_PREFIX=${DEST_DIR} \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_RPATH="\$ORIGIN"
    make -j$(nproc)
    make install
popd

ldconfig

# rm -fr "${PKG_NAME}" "tinyxml2-${VERSION}"