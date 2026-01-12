std_msgs/Header header
uint32 message_num

basic_msgs/Pose6DInt local_pose_stamped

# 类别属性
GridMapInt label   	  # 0-unknow  1-terrain  2-gray  3-obstacle   
                      # 4-hanging 5-negative 6-semantic_edge
# 高度图
GridMapFlt elevation  # elevation_map

# lidar_h_offset = lidar_h_cur - lidar_h_first
float32 lidar_h_offset
float32 residual_x
float32 residual_y