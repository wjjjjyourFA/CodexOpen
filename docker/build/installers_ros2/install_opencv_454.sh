#!/usr/bin/env bash

set -e

# 二进制安装
apt-get install -y \
    libopencv-dev

#### 2,源码安装 ####
# 默认的程序无法编译，需要修改一些值
if false; then

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"
. ./../installers_apollo/installer_base.sh

# if ldconfig -p | grep -q libopencv_core; then
#     info "OpenCV was already installed"
#     exit 0
# fi

WORKHORSE="$1"
if [ -z "${WORKHORSE}" ]; then
    WORKHORSE="cpu"
fi

# Note(all): opencv_contrib is not required in cpu mode
BUILD_CONTRIB="yes"

apt_get_update_and_install \
  libjpeg-dev \
  libpng-dev \
  libtiff-dev \
  libgtk2.0-dev \
  libv4l-dev \
  libxvidcore-dev \
  libx264-dev \
  libopenni-dev \
  libwebp-dev \
  # libeigen3-dev \
  # libopenblas-dev \
  # libatlas-base-dev

apt-get install -y \
  libavcodec-dev libavformat-dev \
  libswscale-dev 
apt-get install -y \
  libgtk-3-dev libcanberra-gtk3-module \
  gfortran python3-dev python3-numpy

# 没有这些库，OpenCV 会退回到 默认 CPU 实现，性能可能略低，但功能正常
apt install -y \
  libopenblas-dev \
  libatlas-base-dev \
  liblapacke-dev 

# OCR 字符检测
apt install -y tesseract-ocr libtesseract-dev

# SFM / CERES（3D 重建）
apt install -y libsuitesparse-dev

# ovis
# apt install -y libogre-1.9-dev libogre-1.9.0v5

THIRD_PARTY_DIR="${SCRIPT_DIR}/../../third_party"
cd "${THIRD_PARTY_DIR}"

# DEST_DIR="/usr/local"
DEST_DIR=${SYSROOT_DIR}

# ==============================
# 检测 CUDA（如果存在则启用）
# ==============================
WITH_CUDA=OFF
# if command -v nvcc >/dev/null 2>&1; then
#   CUDA_VERSION=$(nvcc --version | grep release | awk '{print $6}' | cut -c2-)
#   echo "[CUDA] Detected! Enabling CUDA support."
#   WITH_CUDA=ON
# else
#   echo "[CUDA] Not found. Building CPU version."
# fi

# 4.2.0 | 4.5.4 | 4.6.0
# VERSION=4.2.0
VERSION=4.5.4

# git clone --depth 1 --branch ${VERSION} https://github.com/opencv/opencv.git
# git clone --depth 1 --branch ${VERSION} https://github.com/opencv/opencv_contrib.git

# mv opencv_contrib-${VERSION} opencv-${VERSION}/opencv_contrib-${VERSION}

# ==============================
# 自动检测 Python 路径 
# （兼容 Python 3.8 ~ 3.12）
# 兼容虚拟环境（如 conda）
# ==============================
PYTHON_EXEC=$(which python3)
PYTHON_VERSION=$($PYTHON_EXEC -c "import sys; print(f'{sys.version_info[0]}.{sys.version_info[1]}')")
# PYTHON_PACKAGES=$($PYTHON_EXEC -c "import site; print(site.getsitepackages()[0])")
PYTHON_PACKAGES=$($PYTHON_EXEC -c "from sysconfig import get_paths; print(get_paths()['purelib'])")
# PYTHON_INCLUDE="/usr/include/python${PYTHON_VERSION}"
PYTHON_INCLUDE_DIR=$($PYTHON_EXEC -c "from sysconfig import get_paths; print(get_paths()['include'])")
PYTHON_NUMPY=$($PYTHON_EXEC -c "import numpy; print(numpy.get_include())")

echo "[Python] Executable : $PYTHON_EXEC"
echo "[Python] Version    : $PYTHON_VERSION"
echo "[Python] Packages   : $PYTHON_PACKAGES"

####
# -DCMAKE_CXX_STANDARD=14
# -DCMAKE_CXX_STANDARD=17

EXTRA_OPTIONS=
if [ "${TARGET_ARCH}" = "x86_64" ]; then
    EXTRA_OPTIONS="${EXTRA_OPTIONS} -DCPU_BASELINE=SSE4"
fi

if [ "${BUILD_CONTRIB}" = "yes" ]; then
    EXTRA_OPTIONS="${EXTRA_OPTIONS} -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-${VERSION}/modules"
else
    EXTRA_OPTIONS="${EXTRA_OPTIONS} -DBUILD_opencv_world=OFF"
fi

EXTRA_PYTHON_OPTIONS=""
EXTRA_PYTHON_OPTIONS+=" -DPYTHON_DEFAULT_EXECUTABLE=$(which python3)"
EXTRA_PYTHON_OPTIONS+=" -DOPENCV_PYTHON3_INSTALL_PATH=${SYSROOT_DIR}/lib/python$(py3_version)/dist-packages"
EXTRA_PYTHON_OPTIONS+=" -DPYTHON3_EXECUTABLE=${PYTHON_EXEC}"
EXTRA_PYTHON_OPTIONS+=" -DPYTHON3_PACKAGES_PATH=${PYTHON_PACKAGES}"
EXTRA_PYTHON_OPTIONS+=" -DPYTHON3_INCLUDE_DIR=${PYTHON_INCLUDE}"
EXTRA_PYTHON_OPTIONS+=" -DPYTHON3_NUMPY_INCLUDE_DIRS=${PYTHON_NUMPY}"

cd opencv-${VERSION}

mkdir -p build && cd build

cmake .. \
  -DCMAKE_INSTALL_PREFIX="${SYSROOT_DIR}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_SHARED_LIBS=ON \
  -DENABLE_PRECOMPILED_HEADERS=OFF \
  -DOPENCV_GENERATE_PKGCONFIG=ON \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_DOCS=OFF \
  -DBUILD_TESTS=OFF \
  -DBUILD_PERF_TESTS=OFF \
  -DBUILD_JAVA=OFF \
  -DBUILD_PROTOBUF=OFF \
  -DPROTOBUF_UPDATE_FILES=ON \
  -DINSTALL_C_EXAMPLES=OFF \
  -DWITH_QT=OFF \
  -DWITH_GTK=ON \
  -DWITH_GTK_2_X=OFF \
  -DWITH_IPP=OFF \
  -DWITH_ITT=OFF \
  -DWITH_TBB=OFF \
  -DWITH_EIGEN=ON \
  -DWITH_FFMPEG=ON \
  -DWITH_LIBV4L=ON \
  -DWITH_OPENMP=ON \
  -DWITH_OPENNI=ON \
  -DWITH_OPENCL=ON \
  -DWITH_WEBP=ON \
  -DWITH_VTK=OFF \
  -DOpenGL_GL_PREFERENCE=GLVND \
  -DBUILD_opencv_python2=OFF \
  -DBUILD_opencv_python3=ON \
  -DBUILD_NEW_PYTHON_SUPPORT=ON \
  -DOPENCV_ENABLE_NONFREE=ON \
  -DCV_TRACE=OFF \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DCMAKE_CXX_STANDARD=14 \
  -DCMAKE_INSTALL_RPATH="\$ORIGIN" \
  -DBUILD_opencv_dnn=OFF \
  -DWITH_PROTOBUF=OFF \
  -DProtobuf_INCLUDE_DIR=/usr/include \
  -DProtobuf_LIBRARIES=/usr/lib/x86_64-linux-gnu/libprotobuf.so \
  -DProtobuf_PROTOC_EXECUTABLE=/usr/bin/protoc \
  -DOpenBLAS_INCLUDE_DIR=/usr/include/x86_64-linux-gnu \
  -DOpenBLAS_LIB=/usr/lib/x86_64-linux-gnu/libopenblas.so \
  -DWITH_GSTREAMER=ON \
  -DOPENCV_DOWNLOAD_PATH=../.cache \
  ${EXTRA_OPTIONS} \
  ${EXTRA_PYTHON_OPTIONS} \
  ${GPU_OPTIONS} \
  # -DWITH_CUDNN=ON \
  # -DOPENCV_DNN_CUDA=ON

# 自动检测CPU核心数
make -j$(nproc)  
make install

ok "Successfully installed OpenCV ${VERSION}."

ldconfig

# cd .. && rm -rf build

fi