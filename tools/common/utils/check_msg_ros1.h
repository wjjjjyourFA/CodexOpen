#ifndef CHECK_MSG_ROS1_H
#define CHECK_MSG_ROS1_H

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud) {
  std::cout << "PointCloud fields:" << std::endl;
  for (size_t i = 0; i < cloud->fields.size(); ++i)
    std::cout << cloud->fields[i].name << std::endl;
}

#endif