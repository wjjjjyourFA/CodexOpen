#!/bin/bash

source ~/.bashrc
source /opt/ros/noetic/setup.bash

export MY_PROJECT_PATH=/home/alv/CodexOpen
export OTHER_PROJECT_PATH=/home/alv/CodexOpen/program
export MY_SHELL_NAME='install/scripts'
export MY_SYSTEM_KEY='alv123456'

# source ${MY_PROJECT_PATH}/ros1/devel/setup.bash

export ROS_MASTER_URI=http://192.168.1.90:11311
export ROS_IP=192.168.1.100

mkdir -p ${MY_PROJECT_PATH}/install/log
export MY_LOG_PATH=${MY_PROJECT_PATH}/install/log

source ${MY_PROJECT_PATH}/${MY_SHELL_NAME}/utils/function.sh