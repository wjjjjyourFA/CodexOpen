#!/usr/bin/env bash

set -e

apt-get install -y python-numpy

SCRIPT_DIR=$(cd $(dirname $0) && pwd)

THIRD_PARTY_DIR="${SCRIPT_DIR}/../../third_party"
cd "${THIRD_PARTY_DIR}"

# DEST_DIR="/opt/CodexOpen/matplotlib-cpp"
DEST_DIR=${SYSROOT_DIR}

git clone --depth 1 git@github.com:lava/matplotlib-cpp.git

cd matplotlib-cpp
mkdir -p build && cd build

cmake .. \
  -DCMAKE_INSTALL_PREFIX="${DEST_DIR}" \
  -DCMAKE_INSTALL_RPATH="\$ORIGIN"

make -j$(nproc)
make install

# cd .. && rm -rf build