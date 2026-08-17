#pragma once

#include <memory>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "modules/planning/world_planner/world_planner.h"

namespace jojo {
namespace planning {
namespace ros1 {

class Ros1Convert {
 public:
  Ros1Convert(::ros::NodeHandle& node, ::ros::NodeHandle& private_node);

  bool Init();
  void Run();

 private:
  void TerrainCallback(const sensor_msgs::PointCloud2ConstPtr& message);
  void OdometryCallback(const nav_msgs::OdometryConstPtr& message);
  void GoalCallback(const geometry_msgs::PoseStampedConstPtr& message);
  static common_struct::Pose ConvertPose(const geometry_msgs::Pose& pose);
  static geometry_msgs::Pose ConvertPose(const common_struct::Pose& pose);
  static nav_msgs::Path ConvertPath(const common_struct::Path& path);
  static nav_msgs::OccupancyGrid ConvertOccupancyGrid(
      const common_struct::OccupancyGrid& grid);

  ::ros::NodeHandle node_;
  ::ros::NodeHandle private_node_;
  ::ros::Subscriber terrain_subscriber_;
  ::ros::Subscriber odometry_subscriber_;
  ::ros::Subscriber goal_subscriber_;
  ::ros::Publisher path_publisher_;
  ::ros::Publisher obstacle_grid_publisher_;

  std::unique_ptr<WorldPlanner> planner_;
  double planning_frequency_{5.0};
  int terrain_queue_size_{2};
  int odometry_queue_size_{5};
  int goal_queue_size_{5};
  int path_queue_size_{2};
  int obstacle_grid_queue_size_{1};
  bool path_latched_{true};
  bool obstacle_grid_latched_{true};
  std::string terrain_topic_{"/terrain_map"};
  std::string odometry_topic_{"/state_estimation"};
  std::string goal_topic_{"/way_point"};
  std::string path_topic_{"/global_reference_path"};
  std::string obstacle_grid_topic_{"/global_obstacle_grid"};
};

}  // namespace ros1
}  // namespace planning
}  // namespace jojo
