#!/usr/bin/env bash

set -e

# sudo apt-get install -y git-all
sudo apt-get install -y \
    git \
    python3-pip

# Setup Git LFS
sudo apt-get install -y git-lfs

git lfs install