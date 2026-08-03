#!/usr/bin/env bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"
. ./installer_base.sh

THIRD_PARTY_DIR="${SCRIPT_DIR}/../../third_party"
cd "${THIRD_PARTY_DIR}"

# https://github.com/civetweb/civetweb/archive/v1.11.tar.gz

# https://cloud.tencent.com/developer/article/1913473

# need two minutes

VERSION="1.16"

if [ ! -d "civetweb-${VERSION}" ]; then
    git clone --depth 1 --branch v${VERSION} \
        https://github.com/civetweb/civetweb.git \
        "${THIRD_PARTY_DIR}/civetweb-${VERSION}"
fi

# DEST_DIR="/opt/CodexOpen/civetweb"
DEST_DIR=${SYSROOT_DIR}

pushd "civetweb-${VERSION}"
    mkdir -p buildx && cd buildx
    cmake .. \
        -DCIVETWEB_ENABLE_CXX=ON \
        -DBUILD_SHARED_LIBS=ON \
        -DCMAKE_CXX_STANDARD=14 \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=${DEST_DIR} \
        -DCMAKE_INSTALL_RPATH="\$ORIGIN" \
        -DCIVETWEB_ENABLE_WEBSOCKETS=ON \
        -DBUILD_TESTING=OFF
    cmake --build . --target install --parallel "$(nproc)"
popd

ldconfig

