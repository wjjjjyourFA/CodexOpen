std_msgs/Header header
uint32 message_num

# 这里填写关于从地图获取参考点相对位姿的信息
uint8 map_id     # 如 "GNSS", "SLAM", "Manual"
uint8 point_type # 0: unknown, 1: start, 2: goal, 3: obstacle, 4: landmark

basic_msgs/Vector3Int position
# x1e-2
# z 可以是0，部分地图的高度是0

# x1e-3
int32 heading    # 0.001rad
# 坐标朝向角，单位 车头方向，逆时针为正（车体坐标系）

# x1e-2
int32 speed      # cm/s