#!/usr/bin/env bash

set -e

apt-get install -y libeigen3-dev

if false; then

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"
. ./../installers_apollo/installer_base.sh

THIRD_PARTY_DIR="${SCRIPT_DIR}/../../third_party"
cd "${THIRD_PARTY_DIR}"

VERSION="3.3.7"

if [ ! -d "eigen-${VERSION}" ]; then
    git clone --depth 1 --branch v${VERSION} \
        git@github.com:eigenteam/eigen-git-mirror.git \
        "${THIRD_PARTY_DIR}/eigen-${VERSION}"
fi

DEST_DIR=${SYSROOT_DIR}

pushd "eigen-${VERSION}"
    mkdir -p build && cd build
    cmake .. \
      -DBUILD_SHARED_LIBS=ON \
      -DCMAKE_INSTALL_PREFIX=${DEST_DIR} \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_RPATH="\$ORIGIN"
    make -j$(nproc)
    make install
popd

ldconfig

fi