#!/bin/bash

program_name="ars548"

printf "\e]2;%s\a" "$program_name"

source $(dirname "$0")/source_env.sh

cd ${MY_PROJECT_PATH}/ros1_old

sleep 1
roslaunch ars548_process start.launch
