#!/usr/bin/env bash

# 二进制安装
# sudo apt-get install libopencv-dev

# another way:源码安装，默认的程序无法编译，需要修改一些值
set -e

sudo apt-get install -y libgtk2.0-dev libtiff-dev pkg-config
sudo apt-get install -y build-essential libavcodec-dev libavformat-dev libjpeg-dev libpng-dev libswscale-dev 
sudo apt-get install -y \
  libv4l-dev libxvidcore-dev libx264-dev \
  libgtk-3-dev libcanberra-gtk3-module \
  gfortran python3-dev python3-numpy

# 没有这些库，OpenCV 会退回到 默认 CPU 实现，性能可能略低，但功能正常
sudo apt install -y libopenblas-dev libatlas-base-dev
sudo apt install -y liblapacke-dev

# OCR 字符检测
sudo apt install -y tesseract-ocr libtesseract-dev

# SFM / CERES（3D 重建）
sudo apt install -y libsuitesparse-dev

# ovis
# sudo apt install -y libogre-1.9-dev libogre-1.9.0v5

CURRENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

INSTALL_PREFIX="/usr/local"

cd ${CURRENT_DIR}
cd ..
cd third_party

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

# mv opencv_contrib-${VERSION} opencv-${VERSION}/opencv_contrib

cd opencv-${VERSION}

rm -rf build
mkdir build && cd build

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
# -DCMAKE_CXX_STANDARD=14 \
# -DCMAKE_CXX_STANDARD=17 \

cmake .. \
  -DBUILD_SHARED_LIBS=ON \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_STANDARD=17 \
  -DCMAKE_INSTALL_RPATH="\$ORIGIN" \
  -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib/modules \
  -DOPENCV_DOWNLOAD_PATH=../.cache \
  -DOPENCV_GENERATE_PKGCONFIG=ON \
  -DBUILD_TESTS=OFF \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_PERF_TESTS=OFF \
  -DBUILD_DOCS=OFF \
  -DWITH_TBB=ON \
  -DWITH_OPENMP=ON \
  -DWITH_IPP=ON \
  -DPYTHON3_EXECUTABLE=${PYTHON_EXEC} \
  -DPYTHON3_PACKAGES_PATH=${PYTHON_PACKAGES} \
  -DPYTHON3_INCLUDE_DIR=${PYTHON_INCLUDE} \
  -DPYTHON3_NUMPY_INCLUDE_DIRS=${PYTHON_NUMPY} \
  -DProtobuf_INCLUDE_DIR=/usr/include \
  -DProtobuf_LIBRARIES=/usr/lib/x86_64-linux-gnu/libprotobuf.so \
  -DProtobuf_PROTOC_EXECUTABLE=/usr/bin/protoc \
  -DPROTOBUF_UPDATE_FILES=OFF \
  -DOpenBLAS_INCLUDE_DIR=/usr/include/x86_64-linux-gnu \
  -DOpenBLAS_LIB=/usr/lib/x86_64-linux-gnu/libopenblas.so \
  -DWITH_QT=ON \
  -DWITH_GTK=ON \
  -DWITH_LAPACK=OFF \
  -DWITH_GSTREAMER=ON \
  -DWITH_CUDA=${WITH_CUDA} \
  # -DWITH_CUDNN=ON \
  # -DOPENCV_DNN_CUDA=ON

# 自动检测CPU核心数
make -j$(nproc)  
sudo make install
# cd .. && rm -rf build

sudo ldconfig