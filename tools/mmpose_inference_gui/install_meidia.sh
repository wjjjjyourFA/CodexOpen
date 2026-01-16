#!/usr/bin/env bash

set -e

sudo apt install -y \
  libqt5multimedia5 \
  libqt5multimedia5-plugins \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav
  
sudo apt install -y \
  ffmpeg \
  qt5-default \
  
