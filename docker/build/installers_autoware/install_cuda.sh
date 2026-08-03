#!/usr/bin/env bash

set -e

# AMD64_ENV=/tmp/amd64.env
AMD64_ENV=/workspaces/CodexOpen/docker/build/installers_autoware/amd64.env

# 如果不存在才下载
if [ ! -f "$AMD64_ENV" ]; then
    echo "Downloading amd64.env..."
    wget -O "$AMD64_ENV" https://raw.githubusercontent.com/autowarefoundation/autoware/main/amd64.env
else
    echo "amd64.env already exists, skip downloading."
fi

# 加载环境
source "$AMD64_ENV"

# Modified from:
# https://developer.nvidia.com/cuda-11-4-4-download-archive?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=20.04&target_type=deb_network

# CUDA repo for Ubuntu 22.04 (Jammy)
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-ubuntu2204.pin
mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600

apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/3bf863cc.pub
add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/ /"

apt-get update

cuda_version_dashed=$(eval sed -e "s/[.]/-/g" <<< "${cuda_version}")
apt install -y cuda-${cuda_version_dashed} --no-install-recommends

apt-get install -y libcudnn8=${cudnn_version} libcudnn8-dev=${cudnn_version}

apt-mark hold libcudnn8 libcudnn8-dev