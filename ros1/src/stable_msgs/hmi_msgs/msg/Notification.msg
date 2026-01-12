std_msgs/Header header
uint32 message_num

uint8 level              # 0: 信息, 1: 成功, 2: 提示, 3: 警告
string title             # 提示标题，如“任务完成”
string message           # 提示内容
uint8 duration_sec       # 建议显示时间（秒）