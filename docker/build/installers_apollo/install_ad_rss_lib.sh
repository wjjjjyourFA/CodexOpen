#!/usr/bin/env bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"
. ./installer_base.sh

THIRD_PARTY_DIR="${SCRIPT_DIR}/../../third_party"
cd "${THIRD_PARTY_DIR}"

VERSION="1.1.0"

if [ ! -d "ad-rss-lib-${VERSION}" ]; then
    git clone --depth 1 --branch v${VERSION} \
        https://github.com/intel/ad-rss-lib.git \
        "${THIRD_PARTY_DIR}/ad-rss-lib-${VERSION}"
fi

# DEST_DIR="/opt/CodexOpen/ad-rss-lib"
DEST_DIR=${SYSROOT_DIR}

pushd "ad-rss-lib-${VERSION}"
    mkdir -p build && cd build
    cmake .. \
      -DBUILD_SHARED_LIBS=ON \
      -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
      -DCMAKE_INSTALL_PREFIX=${DEST_DIR} \
      -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_TESTING=OFF \
      -DCMAKE_INSTALL_RPATH="\$ORIGIN"
    make -j$(nproc)
    make install
    cd ..
    if [ ! -d "${DEST_DIR}/include/ad_rss/situation" ]; then
        cp -r ./src/situation "${DEST_DIR}/include/ad_rss/"
    fi
popd

ldconfig