#!/bin/bash

# 引入外部函数文件
source $(dirname "$0")/vlan_configure.sh

#enp0s31f6  enp5s0  enp7s0
#parent_1=enp0s31f6
#parent_2=enp5s0
parent_3=enp7s0
vlan_id=19

#delete_vlan_interface "$parent_1" "$vlan_id"
#delete_vlan_interface "$parent_2" "$vlan_id"
delete_vlan_interface "$parent_3" "$vlan_id"

# 调用函数配置 VLAN
#create_vlan_interface "$parent_1" "$vlan_id" 10.13.1.166/24 10.13.1.1 yes
#create_vlan_interface "$parent_2" "$vlan_id" 10.13.1.166/24 10.13.1.1 no
create_vlan_interface "$parent_3" "$vlan_id" 10.13.1.166/24 10.13.1.1 no
