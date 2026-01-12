#!/usr/bin/env bash

#对于 Universe， rosdistro 变量还可以在该位置找到：../../playbooks/universe.yaml
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

# Taken from https://github.com/astuff/pacmod3#installation
sudo apt-get install -y apt-transport-https
sudo sh -c 'echo "deb [trusted=yes] https://s3.amazonaws.com/autonomoustuff-repo/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/autonomoustuff-public.list'

sudo apt update

sudo apt-get install -y ros-${rosdistro}-pacmod3
sudo apt-get install -y ros-${rosdistro}-plotjuggler-ros