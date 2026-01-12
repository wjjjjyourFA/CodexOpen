std_msgs/Header header
uint32 message_num

# 车机触摸类交互事件

# 触摸事件类型（0: 点击, 1: 滑动, 2: 长按等）
uint8 event_type

# 触摸坐标（屏幕分辨率坐标）
uint16 x
uint16 y

# 滑动方向（仅滑动事件使用，0: 无, 1: 上, 2: 下, 3: 左, 4: 右）
uint8 swipe_direction
uint8 swipe_distance