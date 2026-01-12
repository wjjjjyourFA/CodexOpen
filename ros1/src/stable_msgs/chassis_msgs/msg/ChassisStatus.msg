std_msgs/Header header
uint32 message_num

# 对应数值使用时 x1e-2 以获得实际值
# x1e-2
int32 cur_spd          # 实际速度，cm/s
int32 acc_longitudinal # 纵向加速度，cm/s2
int32 acc_lateral      # 侧向加速度，cm/s2

# x1e-3
int32 yaw_rate       # 航向角速度，0.001rad/s
# 使用 符号 表示方向：> 0 表示左转，< 0 表示右转，inf 表示直行，
# 0 表示原地转向 cur_spd = 0, yaw_rate  != 0

# x1e-2
int32 tur_radius     # 转向半径，cm

uint8 cur_fuel       # 实际油门值，0~100 %
uint8 cur_brake      # 实际刹车值，0~100
uint8 cur_gear       # 实际档位,1～N 2～D 3～R 4～P