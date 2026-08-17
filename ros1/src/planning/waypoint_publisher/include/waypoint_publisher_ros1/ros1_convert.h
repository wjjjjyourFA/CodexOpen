#pragma once

#include <memory>
#include <string>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>

#include "modules/planning/waypoint_publisher/waypoint_publisher.h"

namespace jojo {
namespace planning {
namespace ros1 {

class WaypointPublisherRos1Convert {
 public:
  WaypointPublisherRos1Convert(::ros::NodeHandle& node,
                               ::ros::NodeHandle& private_node);
  bool Init();
  void Run();

 private:
  void OdometryCallback(const nav_msgs::OdometryConstPtr& message);
  void NavigationGoalCallback(
      const geometry_msgs::PoseStampedConstPtr& message);
  static common_struct::Pose FromRosPose(const geometry_msgs::Pose& pose);
  static geometry_msgs::Pose ToRosPose(const common_struct::Pose& pose);
  static std_msgs::Header ToRosHeader(const common_struct::Header& header);

  ::ros::NodeHandle node_;
  ::ros::NodeHandle private_node_;
  ::ros::Subscriber odometry_subscriber_;
  ::ros::Subscriber navigation_goal_subscriber_;
  ::ros::Publisher waypoint_publisher_;
  ::ros::Publisher waypoint_show_publisher_;
  ::ros::Publisher speed_publisher_;
  ::ros::Publisher boundary_publisher_;
  ::ros::Publisher goal_valid_publisher_;
  std::unique_ptr<WaypointPublisher> core_;

  std::string odometry_topic_{"/state_estimation"};
  std::string waypoint_topic_{"/way_point"};
  std::string waypoint_show_topic_{"/way_point_show"};
  std::string navigation_goal_topic_{"/move_base_simple/goal"};
  std::string speed_topic_{"/speed"};
  std::string boundary_topic_{"/navigation_boundary"};
  std::string goal_valid_topic_{"/isgoal_vaild"};
  double processing_frequency_{100.0};
  int odometry_queue_size_{5};
  int navigation_goal_queue_size_{5};
  int output_queue_size_{5};
};

}  // namespace ros1
}  // namespace planning
}  // namespace jojo
