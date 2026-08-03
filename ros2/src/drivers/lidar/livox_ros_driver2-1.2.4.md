## ATTENTION

非原版 livox_ros_driver2，相较于原版，修改了消息发布机制

`ros2 launch msg_MID360_launch.py` 会同时发布 `CustomMsg` 和 `PointCloud2` 两种类型消息。

|     **Topic name**      |            **Type**             |       **Note**        |
| :---------------------: | :-----------------------------: | :-------------------: |
|      /livox/lidar       | livox_ros_driver2/msg/CustomMsg | mid360 自定义消息类型 |
| /livox/lidar_pointcloud |   sensor_msgs/msg/PointCloud2   |   ROS2 点云消息格式   |
|       /livox/imu        |       sensor_msgs/msg/Imu       |    mid360 机内 imu    |

TODO：内置预编译的 Livox SDK2，无需再次克隆编译安装。 
