#!/usr/bin/env bash

set -e

WORKHORSE="$1"
if [ -z "${WORKHORSE}" ]; then
    WORKHORSE="cpu"
fi

CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
. ${CURR_DIR}/installer_base.sh

# Install system-provided pcl need 5 minute
sudo apt-get -y update && \
  sudo apt-get -y install \
  libpcl-dev
# exit 0
if ldconfig -p | grep -q libpcl_common ; then
    info "Found existing PCL installation. Skipp re-installation."
    exit 0
fi

# 启用 PCL 中关于 CUDA 的部分
GPU_OPTIONS="-DCUDA_ARCH_BIN=\"${SUPPORTED_NVIDIA_SMS}\""
if [ "${WORKHORSE}" = "cpu" ]; then
    GPU_OPTIONS="-DWITH_CUDA=OFF"
fi

info "GPU Options for PCL:\"${GPU_OPTIONS}\""

TARGET_ARCH="$(uname -m)"
ARCH_OPTIONS=""
if [ "${TARGET_ARCH}" = "x86_64" ]; then
    ARCH_OPTIONS="-DPCL_ENABLE_SSE=ON"
else
    ARCH_OPTIONS="-DPCL_ENABLE_SSE=OFF"
fi

# libpcap-dev
# libopenmpi-dev
# libboost-all-dev

apt_get_update_and_install \
    libeigen3-dev \
    libflann-dev \
    libglew-dev \
    libglfw3-dev \
    freeglut3-dev \
    libusb-1.0-0-dev \
    libdouble-conversion-dev \
    libopenni-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    liblz4-dev \
    libfreetype6-dev \
    libpcap-dev \
    libqhull-dev

# NOTE(storypku)
# libglfw3-dev depends on libglfw3,
# and libglew-dev have a dependency over libglew2.0

THREAD_NUM=$(nproc)

ldconfig

ok "Successfully installed PCL ${VERSION}"

# Clean up
rm -fr ${PKG_NAME} pcl-pcl-${VERSION}

if [[ -n "${CLEAN_DEPS}" ]]; then
    # Remove build-deps for PCL
    # Note(storypku):
    # Please keep libflann-dev as it was required by local_config_pcl
    apt_get_remove \
        libeigen3-dev \
        libglew-dev \
        libglfw3-dev \
        freeglut3-dev \
        libusb-1.0-0-dev \
        libdouble-conversion-dev \
        libopenni-dev \
        libjpeg-dev \
        libpng-dev \
        libtiff-dev \
        liblz4-dev \
        libfreetype6-dev \
        libpcap-dev \
        libqhull-dev

    # Add runtime-deps for pcl
    apt_get_update_and_install \
        libusb-1.0-0 \
        libopenni0 \
        libfreetype6 \
        libtiff5 \
        libdouble-conversion1 \
        libpcap0.8 \
        libqhull7
fi

# Clean up cache to reduce layer size.
apt-get clean && \
    rm -rf /var/lib/apt/lists/*
