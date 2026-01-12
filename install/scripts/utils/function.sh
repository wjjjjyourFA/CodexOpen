#!/bin/bash

function start_process() {
  local program_name="$1"
  local log_path="$2"

  if [ -z "$program_name" ]; then
    echo "Usage: start_process <program_name> [log_path]"
    return 1
  fi

  # 默认日志路径
  if [ -z "$log_path" ]; then
    log_path="./logs"
  fi

  # 创建日志目录
  mkdir -p "$log_path"

  # 检查进程是否已运行
  if pgrep -x "$program_name" >/dev/null 2>&1; then
    echo "$program_name is already running!"
    return 0
  fi

  # 启动程序（后台运行并重定向日志）
  ./"$program_name" >"$log_path/$program_name.log" 2>&1 | tee "$log_path/$program_name.log"

  sleep 2

  if pgrep -x "$program_name" >/dev/null 2>&1; then
    echo "$program_name started successfully by shell!"
  else
    echo "$program_name start failed!"
    return 1
  fi
}

function kill_process(){
  program="$1"

  # check program is running
  if ! pgrep -x "$program" > /dev/null; then
    echo "$program is not running!!"
  else
    kill -9 $(pgrep -x "$program") 2>/dev/null
    echo "$program is killed!"
  fi
}