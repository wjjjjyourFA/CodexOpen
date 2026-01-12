std_msgs/Header header
uint32 message_num

basic_msgs/Pose6DInt local_pose_stamped

uint32 id
uint8 type            # 0:未知，1:人，2:车，3:自行车，4:动物 ...
uint8 conf            # confidence：0-100

# 感知的物体的3D位置 均以高斯坐标系为基准
basic_msgs/Pose6DInt center_point

# x1e-2
basic_msgs/Vector3Int velocity
# int32 vx              # x，单位（cm/s）
# int32 vy              # y，单位（cm/s）
# int32 vz              # z，单位（cm/s）
int32 speed

# x1e-2
int32 distance

geometry_msgs/Polygon convex_hull  # 目标的边界轮廓

uint16[] para         # 额外控制参数
# 例如 para[0] 代表威胁程度