#!/usr/bin/env bash

set -e

sudo ln -sfn /workspaces/CodexOpenExtra/third_party/cuda/cuda-11.7 /usr/local/cuda-11.7
sudo ln -sfn /usr/local/cuda-11.7 /usr/local/cuda

sudo ln -sfn /workspaces/CodexOpenExtra/third_party/TRT/cuda-11.7/TensorRT-8.5.3.1 /opt/TensorRT-8.5.3.1
sudo ln -sfn /opt/TensorRT-8.5.3.1 /opt/TensorRT