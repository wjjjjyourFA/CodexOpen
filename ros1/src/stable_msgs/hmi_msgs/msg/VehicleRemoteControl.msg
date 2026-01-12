std_msgs/Header header
uint32 message_num

uint8 station_id    # 任务发起者 指控地面站ID 0x01 ~ 0xff
                    # 遥控器也是地面站的一种
uint8 vehicle_id    # 任务执行者 相应命令的车辆ID 0x01：1号车
                    # 0x02：2号车   0x03：3号车   0x04：4号车
                    # 0xFF ：所有车

uint8 control_id    # 控制命令的唯一标识符，用于区分不同的遥控指令

uint8 throttle      # 油门控制，0-100 范围，0表示完全刹车，100表示最大加速
uint8 brake         # 刹车控制，0-100 范围，0表示不刹车，100表示最大刹车
int8 steer          # 转向控制，-100到100 范围，
                    # -100表示最大左转，100表示最大右转，0表示直行
uint8 gear          # 档位控制，例如： 1：前进  2：倒车
                    # 3：空档  4：停车  5：其他档位
uint8 speed_level   # 速度档位（限速控制，用于遥控）

bool emergency_stop # 紧急停止，true表示立即停止，false表示不执行
