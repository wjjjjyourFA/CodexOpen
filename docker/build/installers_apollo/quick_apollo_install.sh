#!/usr/bin/env bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"
. ./installer_base.sh

mkdir -p ./../log

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[-1]}" )" && pwd )"

# apt 编译库
bash ${SCRIPT_DIR}/install_cmake.sh
bash ${SCRIPT_DIR}/install_g++.sh
bash ${SCRIPT_DIR}/install_git.sh
bash ${SCRIPT_DIR}/install_poco.sh

bash ${SCRIPT_DIR}/install_boost.sh
bash ${SCRIPT_DIR}/install_eigen.sh
bash ${SCRIPT_DIR}/install_pcl.sh
# bash ${SCRIPT_DIR}/install_opencv.sh

bash ${SCRIPT_DIR}/install_adolc.sh
bash ${SCRIPT_DIR}/install_uuid.sh
bash ${SCRIPT_DIR}/install_yaml_cpp.sh

# bash ${SCRIPT_DIR}/install_catkin.sh
bash ${SCRIPT_DIR}/install_libgdal.sh
bash ${SCRIPT_DIR}/install_libgeographic.sh
bash ${SCRIPT_DIR}/install_ncurses.sh
bash ${SCRIPT_DIR}/install_opengl.sh

# bash ${SCRIPT_DIR}/install_qt.sh
# bash ${SCRIPT_DIR}/install_ros2.sh

bash ${SCRIPT_DIR}/install_ipopt.sh

# 便捷安装库
bash ${SCRIPT_DIR}/install_gtest.sh
bash ${SCRIPT_DIR}/install_glog.sh
bash ${SCRIPT_DIR}/install_gflags.sh
bash ${SCRIPT_DIR}/install_benchmark.sh

bash ${SCRIPT_DIR}/install_abseil.sh
bash ${SCRIPT_DIR}/install_ad_rss_lib.sh
bash ${SCRIPT_DIR}/install_tinyxml2.sh
bash ${SCRIPT_DIR}/install_json.sh
bash ${SCRIPT_DIR}/install_osqp.sh

bash ${SCRIPT_DIR}/install_osg.sh
bash ${SCRIPT_DIR}/install_civetweb.sh
bash ${SCRIPT_DIR}/install_gperf_tools.sh

bash ${SCRIPT_DIR}/install_proj.sh

# apollo
# bash ${SCRIPT_DIR}/install_tf2.sh

# C17
# bash ${SCRIPT_DIR}/install_matplot++.sh
# bash ${SCRIPT_DIR}/install_matplotlib-cpp.sh

# 手动编译库
bash ${SCRIPT_DIR}/install_fastcdr.sh
bash ${SCRIPT_DIR}/install_fastrtps.sh
bash ${SCRIPT_DIR}/install_protobuf.sh
bash ${SCRIPT_DIR}/install_brpc.sh

# Python
# bash ${SCRIPT_DIR}/install_python3.10.sh
# bash ${SCRIPT_DIR}/install_python3_dependency.sh

# 按需使用
# bash ${SCRIPT_DIR}/install_can.sh
# bash ${SCRIPT_DIR}/install_libtorch.sh

# 不再使用
# bash ${SCRIPT_DIR}/install_openmp.sh


