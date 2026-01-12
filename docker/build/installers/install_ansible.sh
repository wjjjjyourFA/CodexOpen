#!/usr/bin/env bash

# 卸载通过 apt 安装的 ansible
# sudo apt-get install -y ansible
sudo apt-get purge -y ansible

# apt安装 pipx
sudo apt-get -y update
sudo apt-get -y install pipx
 
# 添加 pipx 到 系统 PATH
python3 -m pipx ensurepath
 
# pipx安装 ansible
pipx install --include-deps --force "ansible==6.*" \
  --pip-args="--index-url https://pypi.tuna.tsinghua.edu.cn/simple \
              --trusted-host pypi.tuna.tsinghua.edu.cn"
 
# 安装完成记得生效
source ~/.bashrc