#!/usr/bin/env bash

# 对于 Universe， rosdistro 和 rmw_implementation 变量也可以在该位置找到：../../playbooks/universe.yaml
# wget -O /tmp/amd64.env https://raw.githubusercontent.com/autowarefoundation/autoware/main/amd64.env \
#   && source /tmp/amd64.env

AMD64_ENV=/tmp/amd64.env

# 如果不存在才下载
if [ ! -f "$AMD64_ENV" ]; then
    echo "Downloading amd64.env..."
    wget -O "$AMD64_ENV" https://raw.githubusercontent.com/autowarefoundation/autoware/main/amd64.env
else
    echo "amd64.env already exists, skip downloading."
fi

# 加载环境
source "$AMD64_ENV"

# For details: https://docs.ros.org/en/humble/How-To-Guides/Working-with-multiple-RMW-implementations.html
sudo apt update

rmw_implementation_dashed=$(eval sed -e "s/_/-/g" <<< "${rmw_implementation}")
sudo apt-get install -y ros-${rosdistro}-${rmw_implementation_dashed}
 
# (Optional) You set the default RMW implementation in the ~/.bashrc file.
echo '' >> ~/.bashrc && echo "export RMW_IMPLEMENTATION=${rmw_implementation}" >> ~/.bashrc