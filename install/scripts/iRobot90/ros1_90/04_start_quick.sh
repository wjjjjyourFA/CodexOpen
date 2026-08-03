#!/bin/bash

source $(dirname "$0")/source_env.sh

# 需要 ping 的 ip 地址
export master_ip=192.168.1.90
# 调用函数检查目标 IP 的网络状态
check_host_net "$master_ip"

cd ${MY_PROJECT_PATH}/${MY_SHELL_NAME} || exit 1
# ./09_end_clear.sh
./01_launch_main.sh
