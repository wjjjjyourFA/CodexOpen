#!/bin/bash

program_name="camera_init"

printf "\e]2;%s\a" "$program_name"

source $(dirname "$0")/source_env.sh

# cd ${MY_PROJECT_PATH}/third_tools/tztek_cam_platform/demo

# echo ${MY_SYSTEM_KEY} | sudo -S ./camera_demo >${MY_LOG_PATH}/$program_name.log 2>&1 &
# echo ${MY_SYSTEM_KEY} | sudo -S ./camera_demo >${MY_LOG_PATH}/$program_name.log
echo ${MY_SYSTEM_KEY} | sudo -S tztek-jetson-tool-cpld-test -d /dev/ttyTHS1 -t 4 -c 8 -f 30 -w 1000 -o 0 >${MY_LOG_PATH}/$program_name.log