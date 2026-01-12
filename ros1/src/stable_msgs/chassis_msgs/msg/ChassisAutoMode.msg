std_msgs/Header header
uint32 message_num

# Chassis auto state
bool parking_brake   # 是否拉起手刹
bool drive_enabled   # 驱动是否使能
bool emergency_stop  # 是否处于紧急停车状态
uint8 mode           # 是否处于自动驾驶模式 
# 0: COMPLETE_MANUAL, 1: COMPLETE_AUTO_DRIVE, 
# 2: AUTO_STEER_ONLY, 3: AUTO_SPEED_ONLY, 
# 4: EMERGENCY_MODE