std_msgs/Header header
uint32 message_num

# 融合解算定位信息 GPS IMU DR
uint8 source_type       # 0=gnss, 1=ins--imu+gps, 2=odom--encoder, 
                        # 3=lidar
int16 motion_flag       # motion info is updated

# local_pose
# 使用时 x1e-2 以获得实际值
basic_msgs/Vector3Int position
# int32 pos_x           # 推算 X 轴位置，单位：厘米（cm）
# int32 pos_y           # 推算 Y 轴位置，单位：厘米（cm）
# int32 pos_z           # 推算 Z 轴位置，单位：厘米（cm）
basic_msgs/Vector3Int dev_position

# global_pose
basic_msgs/Vector3Int gauss_point
# Warning 默认高斯坐标 WGS84 坐标系
# x1e-2
# int32 gauss_x         # in cm
# int32 gauss_y         # in cm

# 用于地图对齐、SLAM等用途
# x1e-2
# int32 map_z           # in cm

# global_pose
basic_msgs/GeoPoint geo_point
# x1e-7
# int32 lat             # 纬度（-90 至 90） x1e7 degree
# int32 lon             # 经度（-180 至 180）

# x1e-2
# int32 height          # 海拔高度，单位（cm）

# 局部坐标系，速度要解算到 xyz
# x1e-2
basic_msgs/Vector3Int velocity
# int32 vx              # x，单位（cm/s）
# int32 vy              # y，单位（cm/s）
# int32 vz              # z，单位（cm/s）
int32 speed
basic_msgs/Vector3Int dev_velocity
# int32 dev_vx          # in cm/s
# int32 dev_vy          # in cm/s
# int32 dev_vz          # in cm/s
int32 dev_speed

# 局部坐标系，姿态要解算到 xyz
# x1e-3
basic_msgs/OrientationAngles orientation
# int32 azimuth         # 偏航角（0 至 359.99) 0.001rad
# int32 pitch           # 俯仰角（-90 至 90) 
# int32 roll            # 横滚角（-180 至 180）
basic_msgs/OrientationAngles dev_orientation
# int32 dev_azimuth	    # in 0.001rad
# int32 dev_pitch       # in 0.001rad
# int32 dev_roll	      # in 0.001rad

int8 status             # 系统状态