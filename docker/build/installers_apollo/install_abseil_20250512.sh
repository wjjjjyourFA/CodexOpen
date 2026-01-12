#!/usr/bin/env bash

# Fail on first error.
set -e

# 入口路径获取是“行业标准写法”：获取脚本文件所在的目录
# cd "$(dirname "${BASH_SOURCE[0]}")"
# 第一行只是“计算路径并赋值给变量”，不会改变当前 shell 的工作目录。
# 需要第二行再进入脚本所在路径
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"
. ./installer_base.sh

# 安装前检查:
# 可重复执行，不会重复安装
# CI / 离线环境友好
# todo(zero): if check "libabsl_base.so" is enough
if ldconfig -p | grep -q "libabsl_base.so" ; then
    info "Found existing Abseil installation. Reinstallation skipped."
    exit 0
fi

THIRD_PARTY_DIR="${SCRIPT_DIR}/../../third_party"
cd "${THIRD_PARTY_DIR}"

# Install abseil.
VERSION="20200225.2"
# PKG_NAME="abseil-cpp-${VERSION}.tar.gz"
# DOWNLOAD_LINK="https://apollo-system.cdn.bcebos.com/archive/6.0/${VERSION}.tar.gz"
# CHECKSUM="f41868f7a938605c92936230081175d1eae87f6ea2c248f41077c8f88316f111"
# download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"
# tar xzf "${PKG_NAME}"

# 不使用 apollo 版本，使用官方版本
if [ ! -d "abseil-cpp-${VERSION}" ]; then
    git clone --depth 1 --branch ${VERSION} \
        https://github.com/abseil/abseil-cpp.git \
        "${THIRD_PARTY_DIR}/abseil-cpp-${VERSION}"
fi

# cmake 会自动创建 install 目录
DEST_DIR="/opt/CodexOpen/absl"

pushd "abseil-cpp-${VERSION}"
    mkdir -p build && cd build
    cmake .. \
        -DBUILD_SHARED_LIBS=ON \
        -DCMAKE_CXX_STANDARD=14 \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=${DEST_DIR} \
        -DCMAKE_INSTALL_RPATH="\$ORIGIN"
    cmake --build . --target install --parallel "$(nproc)"
popd

echo "${DEST_DIR}/lib" >> "${APOLLO_LD_FILE}"

ldconfig

# Clean up
# rm -rf "abseil-cpp-${VERSION}" "${PKG_NAME}"