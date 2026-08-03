// ros1_convert.h
#ifndef GROUND_REMOVE_ROS1_CONVERT_H
#define GROUND_REMOVE_ROS1_CONVERT_H

#include <memory>
#include <string>

#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>

#include "modules/perception/common/config/sensor_extrinsics.h"
#include "modules/perception/ground_remove/config/interface_config.h"
#include "modules/perception/ground_remove/config/runtime_config.h"
#include "modules/perception/ground_remove/ground_remove.h"

namespace jojo {
namespace perception {

class Ros1Convert {
 public:
  Ros1Convert(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~Ros1Convert();

  // 0. 初始化
  bool Init(std::shared_ptr<jojo::perception::RuntimeConfig> param,
            std::shared_ptr<jojo::perception::InterfaceConfig> interface);

  void Run();

 protected:
  // 1. 回调函数
  void LidarCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
  void PoseCallback(const nav_msgs::OdometryConstPtr& odom_msg);

  std::shared_ptr<GroundRemove> ground_remove_;

  void UpdateGridMap();

 private:
  std::shared_ptr<jojo::perception::RuntimeConfig> rparam_;
  std::shared_ptr<jojo::perception::InterfaceConfig> iparam_;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber pose_sub_, cloud_sub_;
  ros::Publisher map_pub_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
  Eigen::Matrix4f pose;
  bool cloud_recv_ = false, pose_recv_ = false;

  int map_rows_, map_cols_;
  float map_resolution_;
  int half_rows_, half_cols_;
  std::string map_frame_;

  // 全局消息，时间戳在 LidarCallback 中更新，数据在 UpdateGridMap 中更新
  nav_msgs::OccupancyGrid map_msg;
};

}  // namespace perception
}  // namespace jojo

#endif  // GROUND_REMOVE_ROS1_CONVERT_H