#!/usr/bin/env bash

# apt-get install -y libgtest-dev libgmock-dev

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"
. ./installer_base.sh

THIRD_PARTY_DIR="${SCRIPT_DIR}/../../third_party"
cd "${THIRD_PARTY_DIR}"

VERSION="1.10.0"

if [ ! -d "googletest-release-${VERSION}" ]; then
    git clone --depth 1 --branch ${VERSION} \
        https://github.com/google/googletest.git \
        "${THIRD_PARTY_DIR}/googletest-release-${VERSION}"
fi

DEST_DIR=${SYSROOT_DIR}

pushd "googletest-release-${VERSION}"
    mkdir -p build && cd build
    cmake .. \
        -DCMAKE_CXX_FLAGS="-fPIC" \
        -DCMAKE_CXX_FLAGS="-w" \
        -DBUILD_SHARED_LIBS=ON \
        -DCMAKE_CXX_STANDARD=14 \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=${DEST_DIR} \
        -DCMAKE_INSTALL_RPATH="\$ORIGIN"
    cmake --build . --target install --parallel "$(nproc)"
popd

# echo "${DEST_DIR}/lib" >> "${CODEXOPEN_LD_FILE}"
echo "${DEST_DIR}/lib" | tee -a "${CODEXOPEN_LD_FILE}" > /dev/null

ldconfig

# rm -rf "googletest-release-${VERSION}" "${PKG_NAME}"