std_msgs/Header header
uint32 message_num

basic_msgs/Pose6DInt local_pose_stamped

GridMapFlt occupancy_grid     # 静态环境占据图
PerceptionObject[] objects    # 感知目标列表
LaneMarking[] lanes           # 车道线