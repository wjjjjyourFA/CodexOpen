#!/usr/bin/env bash

set -e

apt-get install -y \
    libtbb-dev

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"
. ./../installers_apollo/installer_base.sh

THIRD_PARTY_DIR="${SCRIPT_DIR}/../../third_party"
cd "${THIRD_PARTY_DIR}"

VERSION="4.3a0"

if [ ! -d "gtsam-${VERSION}" ]; then
    git clone --depth 1 --branch ${VERSION} \
        https://github.com/borglab/gtsam.git \
        "${THIRD_PARTY_DIR}/gtsam-${VERSION}"
fi

DEST_DIR="/usr/local"

# ubuntu20.04 ==> libeigen3-dev 3.3.7
pushd "gtsam-${VERSION}"
    mkdir -p build && cd build
    cmake .. \
        -DBUILD_SHARED_LIBS=ON \
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="${DEST_DIR}" \
        -DCMAKE_INSTALL_RPATH="\$ORIGIN" \
        -DCMAKE_BUILD_RPATH_USE_ORIGIN=ON \
        -DCMAKE_SKIP_BUILD_RPATH=FALSE \
        -DGTSAM_BUILD_EXAMPLES=OFF \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_INSTALL_CPPUNITLITE=OFF \
        -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
        -DGTSAM_USE_SYSTEM_EIGEN=ON \
        -DGTSAM_USE_TBB=ON \
        -DCMAKE_CXX_FLAGS="-Wno-error=unused-but-set-variable"
        # -DEigen3_INCLUDE_DIR=/usr/include/eigen3
        # -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
    make -j$(nproc)
    make install
popd

# check
# grep -i eigen CMakeCache.txt

ldconfig