#!/bin/bash

source $(dirname "$0")/source_env.sh

######################### base #########################
gnome-terminal --tab -e 'bash -c "cd ${MY_PROJECT_PATH}/${MY_SHELL_NAME}; ./03_sync_time.sh; exec bash"'
sleep 5

######################### driver #########################
# gnome-terminal --tab -e 'bash -c "cd ${MY_PROJECT_PATH}/${MY_SHELL_NAME}; ./camera_init.sh; exec bash"'
sleep 5
# gnome-terminal --tab -e 'bash -c "cd ${MY_PROJECT_PATH}/${MY_SHELL_NAME}; ./radar.sh; exec bash"' \
#                --tab -e 'bash -c "cd /home/nvidia/qsz; ./start_all.sh; exec bash"'
sleep 2

######################### perception #########################
sleep 2

######################### final #########################
# gnome-terminal --tab -e 'bash -c "cd ${MY_PROJECT_PATH}/${MY_SHELL_NAME}; ./02_run_maintenance.sh; exec bash"'
sleep 2
