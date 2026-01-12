# Header for timestamp and frame ID
std_msgs/Header header
uint32 message_num

# Desired control values
# 对应数值使用时 x1e-2 以获得实际值
int32 exp_spd      # cm/s >0: foward, cm/s, <0: backward
int32 yaw_rate     # 期望前轮转角,unit:0.01° 
# 使用 符号 表示方向：> 0 表示左转，< 0 表示右转，inf 表示直行，
# 0 表示原地转向 cur_spd = 0, yaw_rate  != 0
int32 tur_radius   # 转向半径，cm

uint8 exp_fuel	   # 期望油门值，0~100 %
uint8 exp_brake	   # 期望刹车值，0~100
uint8 exp_gear     # 期望档位,1～N 2～D 3～R 4～P