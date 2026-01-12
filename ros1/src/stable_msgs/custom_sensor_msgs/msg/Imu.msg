std_msgs/Header header
# for frame synchronization
uint32 message_num

# IMU 初级解算
# 原始数据追求高精度，陀螺仪的单位是 0.001rad/s 0.001rad 0.001m/s^2
basic_msgs/EulerAngles attitude
# 使用时 x1e-3 以获得实际值
# int32 rot_x     # 0.001rad
# int32 rot_y
# int32 rot_z
float64[9] orientation_covariance # Row major about x, y, z axes

ImuRaw raw