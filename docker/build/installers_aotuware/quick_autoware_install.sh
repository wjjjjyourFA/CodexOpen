#!/usr/bin/env bash

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
  python3-colcon-mixin \
  python3-colcon-common-extensions \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  ros-dev-tools \
  python3-flake8 \
  python3-flake8-blind-except \
  python3-flake8-builtins \
  python3-flake8-class-newline \
  python3-flake8-comprehensions \
  python3-flake8-deprecated \
  python3-flake8-docstrings \
  python3-flake8-import-order \
  python3-flake8-quotes \
  python3-pytest-repeat \
  python3-pytest-rerunfailures

# python3 -m pip install -U \
#   flake8-blind-except \
#   flake8-builtins \
#   flake8-class-newline \
#   flake8-comprehensions \
#   flake8-deprecated \
#   flake8-docstrings \
#   flake8-import-order \
#   flake8-quotes \
#   pytest-repeat \
#   pytest-rerunfailures \
#   pytest \
#   setuptools

sudo apt install -y python3-testresources

# 需要手动执行
# install_with_log "golang" ./install_golang.sh
# install_with_log "pacmod" ./install_pacmod.sh

# 串行安装依赖库（顺序敏感）
install_with_log "ansible" ./install_ansible.sh
install_with_log "ccache" ./install_ccache.sh
install_with_log "ceres" ./install_ceres.sh
install_with_log "gstreamer" ./install_gstreamer.sh
install_with_log "git" ./install_git.sh
install_with_log "freetype" ./install_freetype.sh
install_with_log "geographic" ./install_geographic.sh

install_with_log "opengl" ./install_opengl.sh
install_with_log "openjp" ./install_openjp.sh

# 可独立并行安装的库
install_with_log "rmw" ./install_rmw.sh &

# install_with_log "spconv" ./install_spconv.sh &
# install_with_log "cuda" ./install_cuda.sh &

# 等待所有后台任务完成
wait

echo "所有库安装完成，查看 ./logs/ 目录确认安装详情"

sudo apt-get install -y \
  ros-humble-diagnostic-updater \
  ros-humble-geographic-msgs \
  ros-humble-pcl-ros \
  ros-humble-lanelet2-core \
  ros-humble-lanelet2-io \
  ros-humble-lanelet2-validation \
  ros-humble-grid-map-core \
  ros-humble-grid-map-ros \
  ros-humble-grid-map-rviz-plugin \
  ros-humble-xacro \
  ros-humble-topic-tools \
  ros-humble-velodyne-msgs \
  ros-humble-radar-msgs \
  ros-humble-usb-cam \
  ros-humble-aruco \
  ros-humble-ublox-msgs

### 更新下载源码
# cd ../autoware
# mkdir -p src
# vcs import src < autoware_fszn.repos

### 中国加速
sudo mkdir -p /etc/ros/rosdep/sources.list.d/
sudo curl -o /etc/ros/rosdep/sources.list.d/20-default.list https://mirrors.tuna.tsinghua.edu.cn/github-raw/ros/rosdistro/master/rosdep/sources.list.d/20-default.list
export ROSDISTRO_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml
# 更新 rosdep（使用国内源）  --include-eol-distros 让你即使用旧发行版也能更新（例如 ROS Melodic）
rosdep update --include-eol-distros
# 把镜像配置放入 .bashrc，使之永久生效
echo 'export ROSDISTRO_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml' >> ~/.bashrc
source /opt/ros/humble/setup.bash

### 补全依赖包
rosdep update
sudo apt update
rosdep install -y --from-paths src --ignore-src --rosdistro humble

# 源码编译 autoware
# 全核心编译会卡死
# 单个包最多 4 线程
export MAKEFLAGS=-j4
# 同时构建 8 个包
colcon build --symlink-install \
  --parallel-workers 8 \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=OFF
    # -DOpenCV_DIR=/usr/local/lib/cmake/opencv4

### 20251130 jojo update
# autoware_behavior_velocity_detection_area_module