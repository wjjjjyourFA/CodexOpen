std_msgs/Header header
uint32 message_num

uint8 station_id    # 任务发起者 指控地面站ID 0x01 ~ 0xff  
uint8 vehicle_id    # 任务执行者 相应命令的车辆ID 0x01：1号车
uint8 task_id       # 任务编号 具体定义见文档
uint8 task_status   # 任务执行状态 0：默认 1：接到消息但未执行 2：正在执行 
                    # 3：暂停 4：执行成功 5：执行失败
uint8 task_progress # task_status = 3：任务进度（0 ~ 100）
