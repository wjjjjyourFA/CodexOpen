#!/usr/bin/env bash

set -e

apt-get install -y \
  libqt5multimedia5 \
  libqt5multimediawidgets5 \
  libqt5multimedia5-plugins \
  qtmultimedia5-dev \
  qtbase5-dev 

# 非 GUI 开发，可不安装
apt-get install -y \
  qtbase5-dev-tools \
  qttools5-dev qttools5-dev-tools \
  qt5-assistant

apt-get install -y \
  qtcreator