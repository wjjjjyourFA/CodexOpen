#!/usr/bin/env bash

set -e

cd "$(dirname "${BASH_SOURCE[0]}")"

# Related projects
# https://github.com/CSCsw/ColPack.git
# https://github.com/coin-or/ADOL-C

sudo apt-get install -y \
    libcolpack-dev \
    libadolc-dev