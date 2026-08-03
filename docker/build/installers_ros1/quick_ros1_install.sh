#!/usr/bin/env bash

## ros1 全用C14

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"
. ./../installers_apollo/installer_base.sh

mkdir -p ./../logs

apt update 

## 安装基础依赖库
## in Dockerfile
# apt install -y \
#   ninja-build \
#   libboost-all-dev \
#   protobuf-compiler libprotobuf-dev \
#   libprotoc-dev \
#   libgoogle-perftools-dev google-perftools \
#   libyaml-cpp-dev uuid-dev \
#   libpcap-dev \
#   libepoxy-dev \
#   libfmt-dev

# 改系统安装的库
# install_with_log "gflags" ./install_gflags.sh
# install_with_log "glog" ./install_glog.sh
# install_with_log "gtest" ./install_gtest.sh
# install_with_log "json" ./install_json_380.sh
# install_with_log "tinyxml2" ./install_tinyxml2.sh
# in Dockerfile
# apt-get install -y \
#   libgflags-dev \
#   libgoogle-glog-dev \
#   libgtest-dev libgmock-dev \
#   nlohmann-json3-dev \
#   libtinyxml2-dev

apt-get install -y \
  libopencv-dev \
  libpcl-dev

# 安装编译开发工具
apt-get install -y \
  qt5-default

# 串行安装依赖库（顺序敏感）
install_with_log "abseil" ./../installers_apollo/install_abseil_20200225.sh
install_with_log "ccache" ./../installers/install_ccache.sh
install_with_log "ceres" ./../installers/install_ceres.sh
install_with_log "gstreamer" ./../installers/install_gstreamer.sh
install_with_log "geographic" ./../installers/install_geographic.sh

install_with_log "opengl" ./../installers/install_opengl.sh
install_with_log "openjp" ./../installers/install_openjp.sh
install_with_log "gtsam" ./../installers/install_gtsam.sh
install_with_log "ffmpeg" ./../installers_apollo/install_ffmpeg.sh

# 可独立并行安装的库
install_with_log "osqp" ./../installers_apollo/install_osqp.sh &
install_with_log "pangolin" ./../installers/install_pangolin.sh &
install_with_log "nanoflann" ./../installers/install_nanoflann.sh &
install_with_log "nanoflann" ./../installers/install_robin_map.sh &
install_with_log "qtcreator" ./../installers/install_qtcreator.sh &
# install_with_log "opencv" ./install_opencv_420.sh &

# 等待所有后台任务完成
wait

echo "所有库安装完成，查看 ./../logs/ 目录确认安装详情"