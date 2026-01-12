std_msgs/Header header
uint32 message_num

basic_msgs/Pose6DInt local_pose_stamped

# 类别属性
GridMapInt label   	  # 0-unkown 1-dirty 2-grass 
                      # 3-construct 4-sand

# lidar_h_offset = lidar_h_cur - lidar_h_first
float32 lidar_h_offset
float64 residual_x		# in m
float64 residual_y		# in m