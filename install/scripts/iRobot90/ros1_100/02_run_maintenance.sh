#!/bin/bash

program_name="maintain"

printf "\e]2;%s\a" "$program_name"

source $(dirname "$0")/source_env.sh

check_program_name_1="camera_image_to_ros1"
check_1_path=${MY_PROJECT_PATH}/install/bin/modules/drivers/camera

sleep 15
while :
do
ps -ef | grep $check_program_name_1 | grep -v "grep"
if [ $? -ne 0 ]
then
	./camera.sh
  sleep 7
  echo "$check_program_name_1 RESTART success by shell!"
else
  sleep 10
fi
done
