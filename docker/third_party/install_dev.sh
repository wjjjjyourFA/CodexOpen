#!/bin/bash

set -e

sudo apt install -y \
  ninja-build \
  libeigen3-dev \
  libgflags-dev \
  libgoogle-glog-dev \
  libgtest-dev \
  libgoogle-perftools-dev \
  nlohmann-json3-dev \
  linuxptp \
  ptpd 

# qtcreator chinese fonts
sudo apt install -y \
  fonts-noto-cjk \
  qtmultimedia5-dev \
  libqt5multimedia5-plugins

sudo apt install -y \
  gstreamer1.0-libav \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-tools \
  gstreamer1.0-plugins-base-apps

sudo apt install -y \
  libusb-1.0-0-dev \
  libuvc-dev