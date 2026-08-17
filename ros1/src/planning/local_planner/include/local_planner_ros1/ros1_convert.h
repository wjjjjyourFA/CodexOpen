#pragma once

#include <memory>
#include <string>

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>

#include "modules/planning/local_planner/local_planner.h"

namespace jojo {
namespace planning {
namespace ros1 {

class LocalPlannerRos1Convert {
 public:
  LocalPlannerRos1Convert(::ros::NodeHandle& node,
                          ::ros::NodeHandle& private_node);

  bool Init();
  void Run();

 private:
  void OdometryCallback(const nav_msgs::OdometryConstPtr& message);
  void RegisteredScanCallback(
      const sensor_msgs::PointCloud2ConstPtr& message);
  void TerrainCallback(const sensor_msgs::PointCloud2ConstPtr& message);
  void GoalCallback(const geometry_msgs::PoseStampedConstPtr& message);
  void GlobalPathCallback(const nav_msgs::PathConstPtr& message);
  void SpeedCallback(const std_msgs::Float32ConstPtr& message);
  void BoundaryCallback(const geometry_msgs::PolygonStampedConstPtr& message);
  void AddedObstaclesCallback(
      const sensor_msgs::PointCloud2ConstPtr& message);
  void StopCallback(const std_msgs::Int8ConstPtr& message);
  void GoalValidCallback(const std_msgs::BoolConstPtr& message);

  static common_struct::Pose FromRosPose(const geometry_msgs::Pose& pose);
  static geometry_msgs::Pose ToRosPose(const common_struct::Pose& pose);
  static common_struct::Path FromRosPath(const nav_msgs::Path& path);
  nav_msgs::Path ToRosPath(const common_struct::Path& path) const;
  static geometry_msgs::Twist ToRosTwist(
      const common_struct::Twist& twist);

  ::ros::NodeHandle node_;
  ::ros::NodeHandle private_node_;
  ::ros::Subscriber odometry_subscriber_;
  ::ros::Subscriber registered_scan_subscriber_;
  ::ros::Subscriber terrain_subscriber_;
  ::ros::Subscriber goal_subscriber_;
  ::ros::Subscriber global_path_subscriber_;
  ::ros::Subscriber speed_subscriber_;
  ::ros::Subscriber boundary_subscriber_;
  ::ros::Subscriber added_obstacles_subscriber_;
  ::ros::Subscriber stop_subscriber_;
  ::ros::Subscriber goal_valid_subscriber_;
  ::ros::Publisher path_publisher_;
  ::ros::Publisher command_publisher_;
  ::ros::Publisher free_paths_publisher_;
  std::unique_ptr<LocalPlanner> core_;

  std::string odometry_topic_{"/state_estimation"};
  std::string registered_scan_topic_{"/registered_scan"};
  std::string terrain_topic_{"/terrain_map"};
  std::string goal_topic_{"/way_point"};
  std::string global_path_topic_{"/global_reference_path"};
  std::string speed_topic_{"/speed"};
  std::string boundary_topic_{"/navigation_boundary"};
  std::string added_obstacles_topic_{"/added_obstacles"};
  std::string stop_topic_{"/stop"};
  std::string goal_valid_topic_{"/isgoal_vaild"};
  std::string path_topic_{"/path"};
  std::string command_topic_{"/cmd_vel_corrected"};
  std::string free_paths_topic_{"/free_paths"};
  std::string vehicle_frame_{"vehicle"};
  int input_queue_size_{5};
  int path_queue_size_{5};
  int command_queue_size_{5};
  int free_paths_queue_size_{2};
};

}  // namespace ros1
}  // namespace planning
}  // namespace jojo
