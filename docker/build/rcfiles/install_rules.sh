#!/bin/bash

set -e

sudo cp 99-rockmong_usb.rules /etc/udev/rules.d/

sudo chmod 777 /etc/udev/rules.d/99-rockmong_usb.rules

# reload rules
sudo udevadm control --reload-rules
sudo udevadm trigger