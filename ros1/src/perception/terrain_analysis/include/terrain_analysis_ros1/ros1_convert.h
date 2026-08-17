#pragma once

#include <memory>
#include <string>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>

#include "modules/perception/terrain_analysis/terrain_analysis.h"

namespace jojo {
namespace perception {
namespace ros1 {

class TerrainAnalysisRos1Convert {
 public:
  TerrainAnalysisRos1Convert(::ros::NodeHandle& node,
                             ::ros::NodeHandle& private_node);

  bool Init();
  void Run();

 private:
  void OdometryCallback(const nav_msgs::OdometryConstPtr& message);
  void ScanCallback(const sensor_msgs::PointCloud2ConstPtr& message);
  void ClearingCallback(const std_msgs::Float32ConstPtr& message);
  static common_struct::Pose FromRosPose(const geometry_msgs::Pose& pose);

  ::ros::NodeHandle node_;
  ::ros::NodeHandle private_node_;
  ::ros::Subscriber odometry_subscriber_;
  ::ros::Subscriber scan_subscriber_;
  ::ros::Subscriber clearing_subscriber_;
  ::ros::Publisher terrain_publisher_;
  std::unique_ptr<TerrainAnalysis> core_;

  pcl::PointCloud<pcl::PointXYZI> pending_scan_;
  ::ros::Time pending_scan_time_;
  bool has_pending_scan_{false};

  std::string input_topic_{"/registered_scan"};
  std::string odometry_topic_{"/state_estimation"};
  std::string clearing_topic_{"/map_clearing"};
  std::string terrain_map_topic_{"/terrain_map"};
  std::string world_frame_{"map"};
  double processing_frequency_{100.0};
  int odometry_queue_size_{5};
  int scan_queue_size_{5};
  int clearing_queue_size_{5};
  int output_queue_size_{2};
};

}  // namespace ros1
}  // namespace perception
}  // namespace jojo
