std_msgs/Header header
uint32 message_num

basic_msgs/Pose6DInt local_pose_stamped

GridMapFlt elevation_mu      # 地形的均值，代表一个栅格点的平均高程信息
GridMapFlt elevation_sigma   # 地形的标准差（sigma），用于表示地形变化的复杂性或不确定性
GridMapFlt elevation_maxmin  # 地形的最大最小值（maxmin），用于存储每个栅格点的最大和最小高程
GridMapFlt elevation_predict # 预测的地形高程
GridMapFlt elevation_slope   # 地形的坡度

basic_msgs/Pose7DInt[] trajectory # 轨迹点列表，不包含当前

# lidar_h_offset = lidar_h_cur - lidar_h_first
float32 lidar_h_offset
float32 residual_x
float32 residual_y