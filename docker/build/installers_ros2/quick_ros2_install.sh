#!/usr/bin/env bash

## ros2 全用C17

set -e

mkdir -p ./../log

install_with_log() {
  local name=$1
  local script=$2
  local log="./../log/${name}.log"

  echo "[START] $name"
  if $script > $log 2>&1; then
      echo "[SUCCESS] $name"
  else
      echo "[FAIL] $name, see $log"
  fi
}

sudo apt install -y \
  curl wget ninja-build \
  gnupg \
  lsb-release \
  protobuf-compiler libprotobuf-dev \
  libprotoc-dev \
  libyaml-cpp-dev \
  libpcap-dev \
  libboost-all-dev \
  libepoxy-dev \
  mesa-utils libgl1-mesa-dev libepoxy-dev \
  google-perftools libgoogle-perftools-dev \
  build-essential \
  libglu1-mesa-dev \
  libglew-dev \
  libpoco-dev \
  libgtkmm-3.0-dev \
  libfmt-dev

sudo apt-get install -y \
  qtchooser qt5-qmake

# 串行安装依赖库（顺序敏感）
install_with_log "abseil" ./install_abseil_20250512.sh
install_with_log "ccache" ./install_ccache.sh
install_with_log "ceres" ./install_ceres.sh
install_with_log "gstreamer" ./install_gstreamer.sh
install_with_log "geographic" ./install_geographic.sh

install_with_log "opengl" ./install_opengl.sh
install_with_log "openjp" ./install_openjp.sh

# 改系统安装的库
# install_with_log "gflags" ./install_gflags.sh
# install_with_log "glog" ./install_glog.sh
# install_with_log "gtest" ./install_gtest.sh
# install_with_log "json" ./install_json_3113.sh
# install_with_log "tinyxml2" ./install_tinyxml2.sh
sudo apt-get install -y \
  libgflags-dev \
  libgoogle-glog-dev \
  libgtest-dev libgmock-dev \
  nlohmann-json3-dev \
  libtinyxml2-dev

# 可独立并行安装的库
install_with_log "osqp" ./install_osqp.sh &
install_with_log "gtsam" ./install_gtsam.sh &
install_with_log "pangolin" ./install_pangolin.sh &
install_with_log "nanoflann" ./install_nanoflann.sh &
install_with_log "qtcreator" ./install_qtcreator.sh &
# install_with_log "opencv" ./install_opencv_454.sh &

# 等待所有后台任务完成
wait

echo "所有库安装完成，查看 ./logs/ 目录确认安装详情"