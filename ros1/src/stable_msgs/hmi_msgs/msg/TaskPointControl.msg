std_msgs/Header header
uint32 message_num

uint8 station_id    # 任务发起者 指控地面站ID 0x01 ~ 0xff  
uint8 vehicle_id    # 任务执行者 相应命令的车辆ID 0x01：1号车
                    # 0x02：2号车   0x03：3号车   0x04：4号车
                    # 0xFF ：所有车

uint8 control_id    # 控制模式：0x01=遥控，0x02=自主（可拓展）

TaskPoint[] points  # 任务点