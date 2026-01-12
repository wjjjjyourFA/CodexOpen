std_msgs/Header header
uint32 message_num

uint8 task_id   # 任务编号 具体定义见文档
bool is_success # 任务是否成功

# 只是远控下的期望最高，实际速度和方向由规划决定
int8 exp_speed_kmph         # 期望速度（km/h），适用于遥控控制
float32 exp_steer_curvature # 期望转向曲率（cm）

uint8 gear          # 档位控制，例如： 1：前进  2：倒车
                    # 3：空档  4：停车  5：其他档位
uint16 speed_level  # 速度档位（限速控制，用于遥控）

bool emergency_stop # 紧急停止，true表示立即停止，false表示不执行

basic_msgs/GeoPoint geo_point # 任务点
# 对应数值使用时 x1e-3 以获得实际值
# x1e-7
# int32 lon     # Longitude 经度（-180 至 180）
# int32 lat     # Latitude 纬度（-90 至 90） x1e7 degree

# x1e-2
# int32 alti    # Altitude 海拔高度，单位（cm）

basic_msgs/OrientationAngles orient_angles
# x1e-3
# int32 azimuth         # 偏航角（0 至 359.99) 0.001rad
# int32 pitch           # 俯仰角（-90 至 90) 
# int32 roll            # 横滚角（-180 至 180）