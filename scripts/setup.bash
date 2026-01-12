#!/usr/bin/bash

# 获取当前脚本路径
TOP_DIR="$(cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd -P)"
echo "[DEBUG] TOP_DIR = $TOP_DIR"
source ${TOP_DIR}/scripts/apollo.bashrc

# ==> CodexOpen/cyber
export APOLLO_BAZEL_DIST_DIR="${APOLLO_CACHE_DIR}/distdir"
# export CYBER_PATH="${APOLLO_ROOT_DIR}/cyber"
export CYBER_PATH="${APOLLO_ROOT_DIR}/install/bin/cyber"
echo "[DEBUG] CYBER_PATH = $CYBER_PATH"

# ==> CodexOpen/install
export APOLLO_RUNTIME_PATH="${APOLLO_ROOT_DIR}/install"

#### APOLLO_ROOT_DIR == CodexOpenV9/install ####
# bazel_bin_path="${APOLLO_ROOT_DIR}/bazel-bin"
bazel_bin_path="${APOLLO_RUNTIME_PATH}/bin"
mainboard_path="${bazel_bin_path}/cyber/mainboard"
cyber_tool_path="${bazel_bin_path}/cyber/tools"
recorder_path="${cyber_tool_path}/cyber_recorder"
launch_path="${cyber_tool_path}/cyber_launch"
channel_path="${cyber_tool_path}/cyber_channel"
node_path="${cyber_tool_path}/cyber_node"
service_path="${cyber_tool_path}/cyber_service"
monitor_path="${cyber_tool_path}/cyber_monitor"
visualizer_path="${bazel_bin_path}/modules/tools/visualizer"

# TODO(all): place all these in one place and pathprepend
for entry in "${mainboard_path}" \
  "${recorder_path}" "${monitor_path}"  \
  "${channel_path}" "${node_path}" \
  "${service_path}" \
  "${launch_path}" \
  "${visualizer_path}" ; do
  export PATH="${entry}":$PATH
done

#### 设置 python 相关路径 ####
# 手动实际 Python 版本  ==> PYTHON_VERSION
PYTHON_VERSION=3.8
# 自动检测 Python 版本
# PYTHON_VERSION=$(python3 -c "import sys; print(f'python{sys.version_info.major}.{sys.version_info.minor}')")
export PYTHON_INSTALL_PATH="${APOLLO_ROOT_DIR}/install"
pathprepend ${bazel_bin_path}/cyber/python/internal PYTHONPATH
pathprepend "${PYTHON_INSTALL_PATH}/lib/python${PYTHON_VERSION}/site-packages" PYTHONPATH
pathprepend "${PYTHON_INSTALL_PATH}/bin/" PATH
# echo "[DEBUG] PYTHONPATH = $PYTHONPATH"

export CYBER_DOMAIN_ID=80
export CYBER_IP=127.0.0.1

# export GLOG_log_dir="${APOLLO_ROOT_DIR}/data/log"
cd ${APOLLO_RUNTIME_PATH}
mkdir -p "log"
cd ${APOLLO_RUNTIME_PATH}/log
CYBER_TIME=`date +"%Y%m%d_%H"`
mkdir -p "$CYBER_TIME"
export GLOG_log_dir=${APOLLO_RUNTIME_PATH}/log/${CYBER_TIME}
export GLOG_alsologtostderr=0
export GLOG_colorlogtostderr=1
export GLOG_minloglevel=0

export sysmo_start=0

export CYBER_SHM_HOME=/dev/shm/cyber

# for DEBUG log
#export GLOG_v=4

cd ${APOLLO_RUNTIME_PATH}
# source ${CYBER_PATH}/tools/cyber_tools_auto_complete.bash
source ${bazel_bin_path}/cyber/tools/cyber_tools_auto_complete.bash
source ${APOLLO_RUNTIME_PATH}/env.bash