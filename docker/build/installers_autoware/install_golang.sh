#!/usr/bin/env bash

set -e  # 遇到错误退出

# Install Golang (Add Go PPA for shfmt)
sudo add-apt-repository ppa:longsleep/golang-backports

SCRIPT_DIR=$(cd $(dirname $0) && pwd)

THIRD_PARTY_DIR="${SCRIPT_DIR}/../../third_party"
cd "${THIRD_PARTY_DIR}"

cd golang-1.25

## 
# 只下载包，不安装
# apt download golang-go golang-src
# 如果有特定版本
# apt download golang-1.25-go golang-1.25-src

## 
dpkg -i *.deb
sudo apt-get install -y golang golang-go

pip3 install pre-commit \
    -i https://pypi.tuna.tsinghua.edu.cn/simple \
    --trusted-host pypi.tuna.tsinghua.edu.cn

pre_commit_clang_format_version=14.0.6
pip3 install clang-format==${pre_commit_clang_format_version} \
    -i https://pypi.tuna.tsinghua.edu.cn/simple \
    --trusted-host pypi.tuna.tsinghua.edu.cn