#!/bin/bash

source $(dirname "$0")/source_env.sh

export master_ip=192.168.1.90
check_host_net "$master_ip"

cd ${MY_PROJECT_PATH}/${MY_SHELL_NAME} || exit 1
# ./09_end_clear.sh
./01_launch_main.sh