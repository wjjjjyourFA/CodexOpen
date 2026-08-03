#!/usr/bin/env bash

# apt-get install -y nlohmann-json3-dev

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"
. ./../installers_apollo/installer_base.sh

THIRD_PARTY_DIR="${SCRIPT_DIR}/../../third_party"
cd "${THIRD_PARTY_DIR}"

VERSION="3.8.0"

if [ ! -d "json-${VERSION}" ]; then
    git clone --depth 1 --branch v${VERSION} \
        https://github.com/nlohmann/json.git \
        "${THIRD_PARTY_DIR}/json-${VERSION}"
fi

# DEST_DIR="/usr/local"
DEST_DIR=${SYSROOT_DIR}

pushd "json-${VERSION}"
    mkdir -p build && cd build
    cmake .. \
        -DBUILD_SHARED_LIBS=ON \
        -DCMAKE_CXX_STANDARD=14 \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
        -DCMAKE_INSTALL_PREFIX=${DEST_DIR} \
        -DCMAKE_INSTALL_RPATH="\$ORIGIN"
    cmake --build . --target install --parallel "$(nproc)"
popd

ldconfig

# Clean up
# rm -rf "json-${VERSION}" "${PKG_NAME}"