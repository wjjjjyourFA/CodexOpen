#!/bin/sh -e

export MY_SYSTEM_KEY=nvidia
echo ${MY_SYSTEM_KEY} | sudo -S /etc/init.d/ntp stop

sudo mount /dev/nvme0n1p1 /home/nvidia/workspace/
exit 0
