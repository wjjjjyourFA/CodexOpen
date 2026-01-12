std_msgs/Header header
uint32 message_num

# DR (Dead Reckoning) 推算定位数据
# 推算位置数据
basic_msgs/Vector3Int position
# 使用时 x1e-2 以获得实际值
# int32 pos_x           # 推算 X 轴位置，单位：厘米（cm）
# int32 pos_y           # 推算 Y 轴位置，单位：厘米（cm）
# int32 pos_z           # 推算 Z 轴位置，单位：厘米（cm）

# 为了联通CAN总线，推算采用 0.001rad 为单位
# 姿态角（单位：0.001rad）
# 逆时针为正
basic_msgs/OrientationAngles orientation
# 使用时 x1e-3 以获得实际值
# int32 azimuth         # 偏航角，单位：0.001rad
# int32 pitch           # 俯仰角，单位：0.001rad
# int32 roll            # 横滚角，单位：0.001rad

# 推算速度数据 pos 坐标系
basic_msgs/Vector3Int velocity
# 使用时 x1e-2
# int32 vx              # 推算X速度，单位：厘米每秒（cm/s）
# int32 vy              # 推算Y速度，单位：厘米每秒（cm/s）
# int32 vz              # 推算Z速度，单位：厘米每秒（cm/s）
int32 speed
