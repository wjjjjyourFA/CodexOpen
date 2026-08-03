#!/bin/bash

program_name="camera_image_to_ros1"

printf "\e]2;%s\a" "$program_name"

source $(dirname "$0")/source_env.sh

cd ${MY_PROJECT_PATH}/install/bin/modules/drivers/camera

start_process "${program_name}" "${MY_LOG_PATH}"