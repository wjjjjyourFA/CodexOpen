std_msgs/Header header
uint32 message_num

# 对应数值使用时 x1e-3 以获得实际值
# x1e-3
int32 f_whl_angle     # 前轮转角，0.001rad
int32 f_whl_angle_spd	# 前轮转速，0.001rad/s

# x1e-2
int32 fl_whl_spd      # 左前轮速度，cm/s
int32 fr_whl_spd	  	# 右前轮速度，cm/s
int32 rl_whl_spd      # 左后轮速度，cm/s
int32 rr_whl_spd      # 右后轮速度，cm/s
