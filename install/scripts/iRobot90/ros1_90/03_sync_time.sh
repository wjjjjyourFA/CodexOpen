#!/bin/bash

source $(dirname "$0")/source_env.sh

export PTP_NET=enp86s0

# echo ${MY_SYSTEM_KEY} | sudo -S sudo timedatectl set-ntp false
echo ${MY_SYSTEM_KEY} | sudo -S timedatectl set-timezone Asia/Shanghai

#### ptpd master ####
# gnome-terminal --tab -e 'bash -c "echo ${MY_SYSTEM_KEY} | sudo -S ptp4l -E -4 -S -i ${PTP_NET} -m; exec bash"'
gnome-terminal --tab -e 'bash -c "echo ${MY_SYSTEM_KEY} | sudo -S ptpd -E -M -i ${PTP_NET} -C -V; exec bash"'
sleep 2
