#!/usr/bin/env bash

set -e

CURRENT_PATH=$(cd $(dirname $0) && pwd)

INSTALL_PREFIX="/usr/local"

cd $CURRENT_PATH
cd ..
cd third_party

cd nanoflann-1.7.1

sudo cp ./include/nanoflann.hpp $INSTALL_PREFIX/include