std_msgs/Header header
uint32 message_num

# 原始数据硬解码
# GPCHC degree m s
int16 week              # 自 1980-1-6 至当前的星期数
float32 time            # 自本周日 0:00:00 至当前的秒数
float32 azimuth         # 偏航角（0 至 359.99) degree
float32 pitch           # 俯仰角（-90 至 90)
float32 roll            # 横滚角（-180 至 180）
float32 gyro_x          # 陀螺 X 轴 rad/s
float32 gyro_y          # 陀螺 Y 轴
float32 gyro_z          # 陀螺 Z 轴
float32 acc_x           # 加表 X 轴 m/s^2
float32 acc_y           # 加表 Y 轴
float32 acc_z           # 加表 Z 轴
float64 lat             # 纬度（-90 至 90）
float64 lon             # 经度（-180 至 180）
float32 alti            # 海拔高度，单位（米）
float32 ve              # 东向速度，单位（米/秒）
float32 vn              # 北向速度，单位（米/秒）
float32 vu              # 天向速度，单位（米/秒）
float32 v               # 车辆速度，单位（米/秒）
int8 main_satellite_num # 主天线 1 卫星数
int8 vice_satellite_num # 副天线 2 卫星数
int8 status             # 系统状态
int8 age                # 差分延时