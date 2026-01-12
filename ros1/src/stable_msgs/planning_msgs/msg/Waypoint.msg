std_msgs/Header header
uint32 message_num

# 应当基于 局部坐标系 给出 规划 路点
basic_msgs/Vector3Int position
# 使用时 x1e-2 以获得实际值
# int32 pos_x           # 推算 X 轴位置，单位：厘米（cm）
# int32 pos_y           # 推算 Y 轴位置，单位：厘米（cm）
# int32 pos_z           # 推算 Z 轴位置，单位：厘米（cm）

basic_msgs/OrientationAngles orientation
# x1e-3
# int32 azimuth         # 偏航角（0 至 359.99) 0.001rad
# int32 pitch           # 俯仰角（-90 至 90) 
# int32 roll            # 横滚角（-180 至 180）

basic_msgs/Vector3Int velocity
# 使用时 x1e-2
# int32 vx              # 推算X速度，单位：厘米每秒（cm/s）
# int32 vy              # 推算Y速度，单位：厘米每秒（cm/s）
# int32 vz              # 推算Z速度，单位：厘米每秒（cm/s）

float32 curvature       # 曲率（可选） 1/cm