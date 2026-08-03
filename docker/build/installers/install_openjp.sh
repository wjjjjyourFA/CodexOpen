#!/usr/bin/env bash

set -e

apt-get install -y \
    libopenjp2-7-dev

# Ubuntu 22/24 没有打包
# apt-get install -y \
#     libopenjpip-dev \
#     libopenjpip2.4 \
#     openjpeg-tools