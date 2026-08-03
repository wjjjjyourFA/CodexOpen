#!/bin/bash

program_name="lidar"

printf "\e]2;%s\a" "$program_name"

source $(dirname "$0")/../../source_env.sh

cd ${MY_PROJECT_PATH}/ros1

# roslaunch rslidar_sdk start.launch >${MY_LOG_PATH}/$program_name.log 2>&1 &
roslaunch rslidar_sdk start.launch >${MY_LOG_PATH}/$program_name.log
# roslaunch rslidar_sdk start.launch