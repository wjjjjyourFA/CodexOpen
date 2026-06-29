#!/bin/bash

source $(dirname "$0")/source_env.sh

##########################################################
echo "${MY_SYSTEM_KEY}" | sudo -S /etc/init.d/ntp stop
sleep 1

##########################################################
# kill_process object_tracker
sleep 1

##########################################################
kill_process camera_image_to_ros1
# kill_process camera_demo
sleep 1

# kill_process rslidar_sdk_node