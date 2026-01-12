std_msgs/Header header
uint32 message_num

# 原始数据硬解码
# 原始陀螺仪数据是姿态角的基础，但不能直接把陀螺仪值当作 pitch/roll/yaw 的值
# 原始输出数据一般默认是 以 m/s²
# 原始数据追求高精度，陀螺仪的单位是 0.001rad/s  0.001m/s^2
basic_msgs/Vector3Int omega
# 使用时 x1e-3 以获得实际值
# int32 omega_x   # 0.001rad/s
# int32 omega_y
# int32 omega_z
float64[9] angular_velocity_covariance # Row major about x, y, z axes

basic_msgs/Vector3Int acc
# x1e-2
# int32 acc_x     # cm/s^2
# int32 acc_y
# int32 acc_z
float64[9] linear_acceleration_covariance # Row major x, y z