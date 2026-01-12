std_msgs/Header header
uint32 message_num

uint8 station_id    # 任务发起者 指控地面站ID 0x01 ~ 0xff  
uint8 vehicle_id    # 任务执行者 相应命令的车辆ID 0x01：1号车
                    # 0x02：2号车   0x03：3号车   0x04：4号车
                    # 0xFF ：所有车
uint8 task_id       # 任务编号 具体定义见文档
uint8 task_type     # 任务类型 0x00：单车任务 0x01：编队任务
uint8 task_mode     # 任务模式 0x00：手动模式 0x01：自动模式
                    # 该模式主要面向任务对象设置，例如，底盘自主，但上装手控；
uint8 task_priority	# 任务优先级 0x00：低优先级 0x01：中优先级 0x02：高优先级

uint32[] para       # 额外控制参数
