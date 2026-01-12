#!/usr/bin/env bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"
. ./installer_base.sh

THIRD_PARTY_DIR="${SCRIPT_DIR}/../../third_party"
cd "${THIRD_PARTY_DIR}"

VERSION="1.5.1"
# PKG_NAME="benchmark-${VERSION}.tar.gz"
# CHECKSUM="23082937d1663a53b90cb5b61df4bcc312f6dee7018da78ba00dd6bd669dfef2"
# DOWNLOAD_LINK="https://github.com/google/benchmark/archive/v${VERSION}.tar.gz"
# download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"
# tar xzf "${PKG_NAME}"

if [ ! -d "benchmark-${VERSION}" ]; then
    git clone --depth 1 --branch v${VERSION} \
        https://github.com/google/benchmark.git \
        "${THIRD_PARTY_DIR}/benchmark-${VERSION}"
fi

# 为什么安装到 /opt/CodexOpen/sysroot 路径？
DEST_DIR=${SYSROOT_DIR}

pushd "benchmark-${VERSION}"
    cp -a ./../googletest ./
    mkdir -p build && cd build
    cmake .. \
        -DBUILD_SHARED_LIBS=ON \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="${DEST_DIR}" \
        -DCMAKE_INSTALL_RPATH="\$ORIGIN" \
        -DBENCHMARK_ENABLE_LTO=true \
        -DBENCHMARK_ENABLE_GTEST_TESTS=OFF
    make -j$(nproc)
    make install
popd

ldconfig

ok "Successfully installed benchmark ${VERSION}."

# Clean up
# rm -fr "${PKG_NAME}" "benchmark-${VERSION}"