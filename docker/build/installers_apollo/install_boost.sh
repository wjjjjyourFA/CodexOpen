#!/usr/bin/env bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"
. ./installer_base.sh

if ldconfig -p | grep -q "libboost_system.so" ; then
    info "Found existing Boost installation. Reinstallation skipped."
    exit 0
fi

# ubuntu 18.04 默认版本是 1.65
# ubuntu 20.04 默认版本是 1.71
# ubuntu 22.04 默认版本是 1.74
sudo apt-get install -y \
    libboost-all-dev

#### 2,源码安装 ####
if false; then

# PreReq for Unicode support for Boost.Regex
#    icu-devtools \
#    libicu-dev
apt_get_update_and_install \
    liblzma-dev \
    libbz2-dev \
    libzstd-dev

THIRD_PARTY_DIR="${SCRIPT_DIR}/../../third_party"
cd "${THIRD_PARTY_DIR}"

# Ref: https://www.boost.org/
VERSION="1_74_0"
PKG_NAME="boost_${VERSION}.tar.bz2"
DOWNLOAD_LINK="https://boostorg.jfrog.io/artifactory/main/release/${VERSION//_/.}/source/boost_${VERSION}.tar.bz2"
CHECKSUM="83bfc1507731a0906e387fc28b7ef5417d591429e51e788417fe9ff025e116b1"
download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"
tar xjf "${PKG_NAME}"

DEST_DIR=${SYSROOT_DIR}

py3_ver="$(py3_version)"

# Ref: https://www.boost.org/doc/libs/1_73_0/doc/html/mpi/getting_started.html
pushd "boost_${VERSION}"
    # A) For mpi built from source
    #  echo "using mpi : ${SYSROOT_DIR}/bin/mpicc ;" > user-config.jam
    # B) For mpi installed via apt
    # echo "using mpi ;" > user-config.jam
    ./bootstrap.sh \
        --with-python-version=${py3_ver} \
        --prefix="${DEST_DIR}" \
        --without-icu

    ./b2 -d+2 -q -j$(nproc) \
        --without-graph_parallel \
        --without-mpi \
        variant=release \
        link=shared \
        threading=multi \
        install
        #--user-config=user-config.jam
popd

ldconfig

# Clean up
# rm -rf "boost_${VERSION}" "${PKG_NAME}"

if [[ -n "${CLEAN_DEPS}" ]]; then
    apt_get_remove  \
        liblzma-dev \
        libbz2-dev \
        libzstd-dev
fi

fi