#!/usr/bin/env bash

set -e

GEOLOC="${1:-us}"

CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
. ${CURR_DIR}/installer_base.sh

apt_get_update_and_install \
    libtinyxml2-dev \
    libpng-dev \
    nasm

# NodeJS
info "Installing nodejs ..."
bash ${CURR_DIR}/install_node.sh "${GEOLOC}"

info "Installing yarn ..."
bash ${CURR_DIR}/install_yarn.sh

# Clean up cache to reduce layer size.
sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/*
