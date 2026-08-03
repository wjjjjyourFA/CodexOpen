#!/usr/bin/env bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"
. ./installer_base.sh

THIRD_PARTY_DIR="${SCRIPT_DIR}/../../third_party"
cd "${THIRD_PARTY_DIR}"

apt install -y \
  libleveldb-dev \
  libssl-dev

# DEST_DIR="$CURRENT_PATH/../../third_party/install/brpc"
# DEST_DIR="/opt/CodexOpen/brpc"
DEST_DIR=${SYSROOT_DIR}

VERSION="1.16.0"

if [ ! -d "brpc-${VERSION}" ]; then
    git clone --depth 1 --branch ${VERSION} \
        https://github.com/apache/brpc.git \
        "${THIRD_PARTY_DIR}/brpc-${VERSION}"
fi

DEST_DIR=${SYSROOT_DIR}

pushd "brpc-${VERSION}"
  mkdir -p build && cd build
  cmake .. \
    -DBUILD_SHARED_LIBS=ON \
    -DCMAKE_INSTALL_PREFIX=$DEST_DIR \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_RPATH="\$ORIGIN" \
    -DWITH_GLOG=ON \
    -DWITH_GFLAGS=ON \
    -DWITH_BUILTIN_GLOG=OFF \
    -DCMAKE_CXX_FLAGS="-D_GLIBCXX_USE_CXX11_ABI=1" \
    # -DCMAKE_PREFIX_PATH="$THIRD_PARTY_DIR/protobuf" \
    # -DProtobuf_DIR="$THIRD_PARTY_DIR/protobuf/lib/cmake/protobuf" \
    # -DProtobuf_INCLUDE_DIR="$THIRD_PARTY_DIR/protobuf/include" \
    # -DProtobuf_LIBRARIES="$THIRD_PARTY_DIR/protobuf/lib/libprotobuf.so" \
  make -j12
  make install
popd

# echo "${DEST_DIR}/lib" >> "${CODEXOPEN_LD_FILE}"
echo "${DEST_DIR}/lib" | tee -a "${CODEXOPEN_LD_FILE}" > /dev/null

ldconfig

# cd .. && rm -rf build