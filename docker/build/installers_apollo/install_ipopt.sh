#!/usr/bin/env bash

# Fail on first error.
set -e

#### 1, 直接使用库安装 ####
# https://gitee.com/YaoDecheng/ipopt_install?_from=gitee_search#%E4%B8%89%E5%91%BD%E4%BB%A4%E8%A1%8C%E5%AE%89%E8%A3%85

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"
. ./installer_base.sh

sudo apt-get install -y \
    coinor-libipopt-dev

# FIXME(all): dirty hack here.
sudo sed -i '/#define __IPSMARTPTR_HPP__/a\#define HAVE_CSTDDEF' \
    /usr/include/coin/IpSmartPtr.hpp

# Source Code Package Link: https://github.com/coin-or/Ipopt/releases

#### 2,源码安装 ####
# 据博客作者反映，源码安装后期使用容易出错

if false; then

sudo apt-get install -y \
    gcc g++ gfortran git patch wget pkg-config \
    liblapack-dev libmetis-dev

THIRD_PARTY_DIR="${SCRIPT_DIR}/../../third_party"
cd "${THIRD_PARTY_DIR}"

git clone git@gitee.com:lebment/Ipopt.git

cd Ipopt
# 一定要看到配置成功的提示
./configure
 
# 编译
make -j$(nproc)
# make test
make install

fi
