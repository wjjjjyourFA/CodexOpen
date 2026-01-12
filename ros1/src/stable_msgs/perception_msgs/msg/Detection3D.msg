std_msgs/Header header
uint32 message_num

uint16 device_id

uint32 id
uint8 type            # 0:未知，1:人，2:车，3:自行车，4:动物 ...
uint8 conf            # confidence：0-100

# 检测框坐标，3D坐标
uint16 xmin
uint16 ymin
uint16 zmin
uint16 xmax
uint16 ymax
uint16 zmax