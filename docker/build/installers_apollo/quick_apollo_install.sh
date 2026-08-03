#!/usr/bin/env bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"
. ./installer_base.sh

mkdir -p ./../logs

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[-1]}" )" && pwd )"

# apt 编译库
# bash ${SCRIPT_DIR}/install_cmake.sh
# bash ${SCRIPT_DIR}/install_g++.sh
# bash ${SCRIPT_DIR}/../installers_autoware/install_git.sh
bash ${SCRIPT_DIR}/install_poco.sh

# bash ${SCRIPT_DIR}/install_boost.sh
# bash ${SCRIPT_DIR}/install_eigen.sh
# bash ${SCRIPT_DIR}/install_pcl.sh
# bash ${SCRIPT_DIR}/install_opencv.sh

bash ${SCRIPT_DIR}/install_adolc.sh
bash ${SCRIPT_DIR}/install_cyber_deps.sh
# bash ${SCRIPT_DIR}/install_abseil_20200225.sh
# bash ${SCRIPT_DIR}/install_uuid.sh
# bash ${SCRIPT_DIR}/install_ncurses.sh
# bash ${SCRIPT_DIR}/install_yaml_cpp.sh

# bash ${SCRIPT_DIR}/install_catkin.sh
bash ${SCRIPT_DIR}/install_gdal.sh
# bash ${SCRIPT_DIR}/../installers/install_geographic.sh
# bash ${SCRIPT_DIR}/../installers/install_opengl.sh

bash ${SCRIPT_DIR}/install_ipopt.sh

# 便捷安装库
# bash ${SCRIPT_DIR}/install_gflags_glog.sh
# bash ${SCRIPT_DIR}/install_glog.sh
# bash ${SCRIPT_DIR}/install_gtest.sh
bash ${SCRIPT_DIR}/install_benchmark.sh

# 拒绝 Intel 库
# bash ${SCRIPT_DIR}/install_ad_rss_lib.sh

# bash ${SCRIPT_DIR}/install_tinyxml2.sh
# bash ${SCRIPT_DIR}/install_json.sh
# bash ${SCRIPT_DIR}/install_osqp.sh

# 工程级 3D 场景图引擎；被 ROS2 替代；
# bash ${SCRIPT_DIR}/install_osg.sh

# 轻量级嵌入式 Web 服务器库
# bash ${SCRIPT_DIR}/install_civetweb.sh

# equals to libgoogle-perftools-dev
# bash ${SCRIPT_DIR}/install_gperftools.sh

# 负责“经纬度 ↔ 平面坐标 ↔ 各种地图坐标系”的数学库
# bash ${SCRIPT_DIR}/install_proj.sh

# apollo
# bash ${SCRIPT_DIR}/install_tf2.sh

# C17
# bash ${SCRIPT_DIR}/install_matplot++.sh
# bash ${SCRIPT_DIR}/install_matplotlib-cpp.sh

# 手动编译库
# CyberRT 已经死了，弃用
# bash ${SCRIPT_DIR}/install_fast-cdr.sh
# bash ${SCRIPT_DIR}/install_fast-dds.sh
# bash ${SCRIPT_DIR}/install_fast-rtps.sh
# bash ${SCRIPT_DIR}/install_protobuf.sh
# bash ${SCRIPT_DIR}/install_brpc.sh

# Python
# bash ${SCRIPT_DIR}/install_python3.10.sh
# bash ${SCRIPT_DIR}/install_python3_dependency.sh

# 按需使用
# bash ${SCRIPT_DIR}/install_linuxcan.sh
# bash ${SCRIPT_DIR}/install_libtorch.sh

# 不再使用
# bash ${SCRIPT_DIR}/install_openmp.sh


