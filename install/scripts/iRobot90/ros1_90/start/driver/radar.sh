#!/bin/bash

program_name="radar"

printf "\e]2;%s\a" "$program_name"

source $(dirname "$0")/../../source_env.sh

cd ${MY_PROJECT_PATH}/ros1

sleep 1
roslaunch ars548_process start.launch >${MY_LOG_PATH}/$program_name.log
# roslaunch ars548_process start.launch
