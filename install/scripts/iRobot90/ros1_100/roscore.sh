#!/bin/bash

program_name="roscore"

printf "\e]2;%s\a" "$program_name"

source $(dirname "$0")/source_env.sh

# ps -ef|grep $program_name |grep -v "grep"
# if [ $? -ne 0 ]
# then
# 	$program_name >${MY_LOG_PATH}/$program_name.log 2>&1 &
# 	sleep 2
# 	ps -ef|grep $program_name |grep -v "grep"
# 	if [ $? -eq 0 ]
# 	then
# 	echo "$program_name start sucess by shell!"
# 	else
# 	echo "$program_name start failed"
# 	fi
# else
# 	echo "$program_name is already runing!"
# fi

start_process "${program_name}" "${MY_LOG_PATH}"