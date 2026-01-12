std_msgs/Header header
uint32 message_num

basic_msgs/Pose6DInt local_pose_stamped

uint32 id
uint8 type            # 0:未知，1:人，2:车，3:自行车，4:动物 ...
uint8 conf            # confidence：0-100

# current object 3D position
basic_msgs/Pose6DInt center_point

# x1e-2
basic_msgs/Vector3Int velocity
# int32 vx              # x，单位（cm/s）
# int32 vy              # y，单位（cm/s）
# int32 vz              # z，单位（cm/s）
int32 speed

basic_msgs/Pose7DInt[] trajectory # 轨迹点列表，不包含当前