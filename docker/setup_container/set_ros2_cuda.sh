#!/usr/bin/env bash

set -e

sudo ln -sfn /workspaces/CodexOpenExtra/third_party/cuda/cuda-12.6 /usr/local/cuda-12.6
sudo ln -sfn /usr/local/cuda-12.6 /usr/local/cuda

sudo ln -sfn /workspaces/CodexOpenExtra/third_party/TRT/cuda-12.6/TensorRT-10.13.3.9 /opt/TensorRT-10.13.3.9
sudo ln -sfn /opt/TensorRT-10.13.3.9 /opt/TensorRT