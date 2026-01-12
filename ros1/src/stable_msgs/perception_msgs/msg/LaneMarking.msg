std_msgs/Header header
uint32 message_num

basic_msgs/Pose6DInt local_pose_stamped

uint32 id
uint8 type             # 实线、虚线、黄色、白色、双黄等
uint8 conf             # confidence：0-100

int8 relative_position # 车道线在自车坐标系中的相对位置，
                       # -2 左2 | -1 左1 | 0 中 | 1 右1 | 2 右2

# 车道线拟合出来的点
# x1e-2
basic_msgs/Vector3Int[] points