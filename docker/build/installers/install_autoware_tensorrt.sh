#!/usr/bin/env bash

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

# Taken from: https://docs.nvidia.com/deeplearning/tensorrt/install-guide/index.html#installing

# TensorRT repo for Ubuntu 22.04 (Jammy)
sudo apt-get install -y \
    libnvinfer8=${tensorrt_version} \
    libnvonnxparsers8=${tensorrt_version} \
    libnvparsers8=${tensorrt_version} \
    libnvinfer-plugin8=${tensorrt_version} \
    libnvinfer-dev=${tensorrt_version} \
    libnvonnxparsers-dev=${tensorrt_version} \
    libnvparsers-dev=${tensorrt_version} \
    libnvinfer-plugin-dev=${tensorrt_version}

sudo apt-mark hold \
    libnvinfer8 \
    libnvonnxparsers8 \
    libnvparsers8 \
    libnvinfer-plugin8 \
    libnvinfer-dev \
    libnvonnxparsers-dev \
    libnvparsers-dev \
    libnvinfer-plugin-dev