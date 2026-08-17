#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>

#include "terrain_waypoint_exploration/terrain_waypoint_explorer.h"

namespace terrain_waypoint_exploration {
namespace ros1 {

class TerrainWaypointExplorerRos1Convert {
 public:
  TerrainWaypointExplorerRos1Convert(::ros::NodeHandle& node,
                                     ::ros::NodeHandle& private_node);

  bool Init();
  void Run();

 private:
  void OdometryCallback(const nav_msgs::OdometryConstPtr& message);
  void ScanCallback(const sensor_msgs::PointCloud2ConstPtr& message);
  void TerrainCallback(const sensor_msgs::PointCloud2ConstPtr& message);
  void PlanningTimerCallback(const ::ros::TimerEvent& event);
  void PublishOutput(const TerrainWaypointExplorerOutput& output,
                     const ::ros::Time& timestamp);
  static jojo::common_struct::Pose FromRosPose(
      const geometry_msgs::Pose& pose);
  static geometry_msgs::Pose ToRosPose(
      const jojo::common_struct::Pose& pose);

  ::ros::NodeHandle node_;
  ::ros::NodeHandle private_node_;
  ::ros::Subscriber odometry_subscriber_;
  ::ros::Subscriber scan_subscriber_;
  ::ros::Subscriber terrain_subscriber_;
  ::ros::Publisher waypoint_publisher_;
  ::ros::Publisher goal_valid_publisher_;
  ::ros::Publisher map_publisher_;
  ::ros::Publisher frontier_publisher_;
  ::ros::Publisher candidate_publisher_;
  ::ros::Publisher goal_marker_publisher_;
  ::ros::Publisher finished_publisher_;
  ::ros::Timer planning_timer_;
  std::unique_ptr<TerrainWaypointExplorer> core_;
  std::mutex core_mutex_;

  std::string world_frame_{"map"};
  std::string odometry_topic_{"/state_estimation"};
  std::string registered_scan_topic_{"/registered_scan"};
  std::string terrain_topic_{"/terrain_map"};
  std::string waypoint_topic_{"/way_point"};
  std::string goal_valid_topic_{"/isgoal_vaild"};
  std::string finished_topic_{"/exploration_finished"};
  std::string debug_map_topic_{"debug_map"};
  std::string debug_frontiers_topic_{"debug_frontiers"};
  std::string debug_candidates_topic_{"debug_candidates"};
  std::string debug_goal_topic_{"debug_goal"};
  int odometry_queue_size_{20};
  int scan_queue_size_{2};
  int terrain_queue_size_{2};
  int waypoint_queue_size_{2};
  int goal_valid_queue_size_{5};
  int debug_queue_size_{1};
  int finished_queue_size_{1};
  bool output_latched_{true};
};

}  // namespace ros1
}  // namespace terrain_waypoint_exploration
