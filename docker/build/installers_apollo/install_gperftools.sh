#!/usr/bin/env bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"
. ./installer_base.sh

apt_get_update_and_install \
    libunwind8 \
    libunwind-dev \
    graphviz

THIRD_PARTY_DIR="${SCRIPT_DIR}/../../third_party"
cd "${THIRD_PARTY_DIR}"

VERSION="2.8"
PKG_NAME="gperftools-${VERSION}.tar.gz"
CHECKSUM="b09193adedcc679df2387042324d0d54b93d35d062ea9bff0340f342a709e860"
DOWNLOAD_LINK="https://github.com/gperftools/gperftools/archive/${PKG_NAME}"
download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"
tar xzf ${PKG_NAME}

if [ ! -d "gperftools-${VERSION}" ]; then
    git clone --depth 1 --branch gperftools-${VERSION} \
        https://github.com/gperftools/gperftools.git \
        "${THIRD_PARTY_DIR}/gperftools-${VERSION}"
fi

pushd "gperftools-${VERSION}" >/dev/null
    ./autogen.sh || sleep 1 && ./autogen.sh
    ./configure --prefix=/usr
    # --disable-libunwind
    # shared lib only options: --enable-static=no --with-pic=yes
    make -j$(nproc)
    make install
popd >/dev/null

ldconfig

ok "Successfully installed gperftools-${VERSION}."

# Clean up
sudo apt_get_remove \
    libunwind-dev

# rm -rf ${PKG_NAME} "gperftools-gperftools-${VERSION}"
