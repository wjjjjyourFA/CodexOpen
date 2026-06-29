#!/bin/bash

# 需要 ping 的 ip 地址
export master_ip=192.168.1.90
while ((1)); do
  if ping -w 1 -c 1 $master_ip  > /dev/null; then
    echo "$master_ip ping is success"
    break
  else
    echo "$master_ip ping is failure"
  fi
  sleep 1
done

source $(dirname "$0")/source_env.sh

cd ${MY_PROJECT_PATH}/${MY_SHELL_NAME}
# ./09_end_clear.sh
./01_launch_main.sh
