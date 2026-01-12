std_msgs/Header header
uint32 message_num

# GNSS 压缩数据
int16 week                # 自 1980-1-6 至当前的星期数
float32 time              # 自本周日 0:00:00 至当前的秒数

basic_msgs/GeoPoint position
# x1e-7
# int32 lat               # 纬度（-90 至 90） x1e7 degree
# int32 lon               # 经度（-180 至 180）
# x1e-2
# int32 alti              # 海拔高度，单位（cm）

# 0.001rad
# 对应数值使用时 x1e-3 以获得实际值
basic_msgs/OrientationAngles orientation
# x1e-3
# int32 azimuth           # 偏航角（-180 至 180) 实际是 0.001rad
# int32 pitch             # 俯仰角（-90 至 90) 
# int32 roll              # 横滚角（-180 至 180）

basic_msgs/Vector3Int gyro
# x1e-3
# int32 gyro_x            # 陀螺 X 轴 0.001rad/s
# int32 gyro_y            # 陀螺 Y 轴
# int32 gyro_z            # 陀螺 Z 轴

basic_msgs/Vector3Int acc
# x1e-2
# int32 acc_x             # 加表 X 轴 cm/s^2
# int32 acc_y             # 加表 Y 轴
# int32 acc_z             # 加表 Z 轴

basic_msgs/GeoVelocity velocity
# 东北天坐标系 X轴：北向（North）Y轴：东向（East）Z轴：天向（Up）
# x1e-2
# int32 ve                # 东向速度，单位（cm/s）
# int32 vn                # 北向速度，单位（cm/s）
# int32 vu                # 天向速度，单位（cm/s）
# int32 speed             # 车辆速度，单位（cm/s）

int8 main_satellite_num # 主天线 1 卫星数
int8 vice_satellite_num # 副天线 2 卫星数
int8 status             # 系统状态
int8 age                # 差分延时