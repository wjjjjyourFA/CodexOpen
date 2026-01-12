#!/usr/bin/env bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"
. ./installer_base.sh

apt_get_update_and_install \
    libsqlite3-dev \
    sqlite3 \
    libtiff-dev \
    libcurl4-openssl-dev

# Note(storypku)
# apt_get_update_and_install libproj-dev
# proj installed via apt was 4.9.3, incompatible with pyproj which
# requres proj >= 6.2.0

if ldconfig -p | grep -q "libproj.so"; then
    warning "Proj was already installed. Reinstallation skipped."
    exit 0
fi

VERSION="7.1.0"
# PKG_NAME="proj-${VERSION}.tar.gz"
# CHECKSUM="876151e2279346f6bdbc63bd59790b48733496a957bccd5e51b640fdd26eaa8d"
# DOWNLOAD_LINK="https://github.com/OSGeo/PROJ/releases/download/${VERSION}/${PKG_NAME}"
# download_if_not_cached "$PKG_NAME" "$CHECKSUM" "$DOWNLOAD_LINK"
# tar xzf "${PKG_NAME}"

# 可以安装到thirdparty 中，不要安装到根目录中
THIRD_PARTY_DIR="${SCRIPT_DIR}/../../third_party"
cd "${THIRD_PARTY_DIR}"

if [ ! -d "proj-${VERSION}" ]; then
    git clone --depth 1 --recursive --branch ${VERSION} \
        https://github.com/OSGeo/PROJ.git \
        "${THIRD_PARTY_DIR}/proj-${VERSION}"
fi

DEST_DIR=${SYSROOT_DIR}

pushd proj-${VERSION} >/dev/null
    mkdir -p build && cd build
    cmake .. \
        -DBUILD_SHARED_LIBS=ON \
        -DBUILD_TESTING=OFF \
        -DCMAKE_INSTALL_PREFIX="${DEST_DIR}" \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_RPATH="\$ORIGIN"
    make -j$(nproc)
    make install
popd >/dev/null

ldconfig

ok "Successfully built proj = ${VERSION}"

# rm -fr "${PKG_NAME}" "proj-${VERSION}"

if [[ -n "${CLEAN_DEPS}" ]]; then
    # Remove build-deps for proj
    apt_get_remove \
        libsqlite3-dev \
        sqlite3 \
        libtiff-dev \
        libcurl4-openssl-dev
fi

# Clean up cache to reduce layer size.
sudo apt-get clean &&
    sudo rm -rf /var/lib/apt/lists/*