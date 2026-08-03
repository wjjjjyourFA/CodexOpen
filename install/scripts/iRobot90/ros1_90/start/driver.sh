#!/bin/bash

source $(dirname "$0")/../source_env.sh

gnome-terminal \
  --tab -e 'bash -c "cd ${MY_PROJECT_PATH}/${MY_SHELL_NAME}/start/diver; ./lidar.sh; exec bash"' \
  --tab -e 'bash -c "cd ${MY_PROJECT_PATH}/${MY_SHELL_NAME}/start/diver; ./radar.sh; exec bash"' \
