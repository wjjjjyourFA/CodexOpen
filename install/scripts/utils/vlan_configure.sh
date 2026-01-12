#!/bin/bash

# 删除 VLAN 接口
function delete_vlan_interface(){
  local parent="$1"
  local vlan_id="$2"
  local vlan_if="${parent}.${vlan_id}"

  # 判断接口是否存在
  if ip link show "$vlan_if" &> /dev/null; then
    echo "Deleting existing interface: $vlan_if"
    sudo ip link set "$vlan_if" down
    sudo ip link delete "$vlan_if"
  fi
}

#This function calls the nm-connection-editor so the user can create a virtual connection.
function create_vlan_interface(){
  local parent="$1"
  local vlan_id="$2"
  local ip_addr="$3"
  local gateway="$4"

  if [ -z "$parent" ] || [ -z "$vlan_id" ] || [ -z "$ip_addr" ] || [ -z "$gateway" ]; then
    echo "用法: create_vlan_interface <父接口> <VLAN ID> <IP地址/掩码> <网关> [yes/no]"
    return 1
  fi

  local vlan_if="${parent}.${vlan_id}"

  echo "creating connection interface : $vlan_if"

  # 删除已有接口
  delete_vlan_interface "$parent" "$vlan_id"
  
  # 创建 VLAN 接口
  sudo ip link add link "$parent" name "$vlan_if" type vlan id "$vlan_id"
  
  # 计算广播地址
  local ip_only="${ip_addr%/*}"
  local cidr="${ip_addr#*/}"
  local i1 i2 i3 i4 mask bcast b1 b2 b3 b4 broadcast_addr
  IFS=. read -r i1 i2 i3 i4 <<< "$ip_only"
  local mask=$((0xFFFFFFFF << (32 - cidr) & 0xFFFFFFFF))
  local bcast=$(( ( (i1<<24)|(i2<<16)|(i3<<8)|i4 ) | (~mask & 0xFFFFFFFF) ))
  local b1=$(( (bcast >> 24) & 0xFF ))
  local b2=$(( (bcast >> 16) & 0xFF ))
  local b3=$(( (bcast >> 8) & 0xFF ))
  local b4=$(( bcast & 0xFF ))
  local broadcast_addr="$b1.$b2.$b3.$b4"
  
  # 配置 IP
  sudo ip addr add "$ip_addr" broadcast "$broadcast_addr" dev "$vlan_if"

  # 启动接口
  sudo ip link set "$vlan_if" up

  # 设置默认路由（只在需要时设置）
  if [ "$set_default" = "yes" ]; then
    sudo ip route replace default via "$gateway" dev "$vlan_if"
  fi

  echo "connection $vlan_if created ... "
  echo "VLAN interface $vlan_if created with IP $ip_addr and broadcast $broadcast_addr."
}

# 调用示例
# configure_vlan enp5s0 19 10.13.1.166/24 10.13.1.1 yes
