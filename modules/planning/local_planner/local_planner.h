#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "modules/common_struct/basic_msgs/Pose.h"
#include "modules/common_struct/planning_msgs/Path.h"
#include "modules/common_struct/planning_msgs/Planning.h"

namespace jojo {
namespace planning {

struct LocalPlannerConfig {
  std::string path_folder;
  double vehicle_length{0.6};
  double vehicle_width{0.6};
  double sensor_offset_x{0.0};
  double sensor_offset_y{0.0};
  bool two_way_drive{true};
  double laser_voxel_size{0.05};
  double terrain_voxel_size{0.2};
  bool use_terrain_analysis{false};
  bool check_obstacle{true};
  bool check_rotation_obstacle{false};
  double adjacent_range{3.5};
  double obstacle_height_threshold{0.2};
  double ground_height_threshold{0.1};
  double cost_height_threshold{0.1};
  double cost_score{0.02};
  double obstacle_inflation_radius{0.12};
  double inflated_obstacle_penalty{0.35};
  double center_path_bias{0.25};
  double path_continuity_weight{0.35};
  double group_continuity_weight{0.15};
  double side_switch_penalty{0.4};
  double large_switch_angle_degrees{50.0};
  double hold_path_score_ratio{0.92};
  bool use_cost{false};
  int point_per_path_threshold{2};
  double min_relative_z{-0.5};
  double max_relative_z{0.25};
  double max_speed{1.0};
  double direction_weight{0.02};
  double direction_threshold{90.0};
  bool direction_to_vehicle{false};
  double path_scale{1.0};
  double min_path_scale{0.75};
  double path_scale_step{0.25};
  bool path_scale_by_speed{true};
  double min_path_range{1.0};
  double path_range_step{0.5};
  bool path_range_by_speed{true};
  bool path_crop_by_goal{true};
  bool autonomy_mode{false};
  double autonomy_speed{1.0};
  double goal_clear_range{0.5};
  double goal_x{0.0};
  double goal_y{0.0};
  double global_path_look_ahead{2.0};
  double global_path_goal_switch_distance{1.5};
  double control_look_ahead_distance{0.5};
  double forward_align_angle_degrees{8.0};
  double yaw_rate_gain{1.8};
  double max_yaw_rate_degrees{20.0};
  double max_acceleration{0.3};
  double max_yaw_acceleration_degrees{40.0};
  double goal_stop_distance{0.2};
  double goal_slow_distance{1.0};
  double plan_timeout{0.5};
  double control_frequency{100.0};
  double near_goal_enter_distance{0.5};
  double near_goal_exit_distance{0.8};
  double near_goal_xy_tolerance{0.2};
  double near_goal_yaw_tolerance_degrees{5.0};
  double near_goal_min_speed{0.08};
  double near_goal_min_yaw_rate_degrees{3.0};
  double near_goal_position_gain{0.8};
  double near_goal_yaw_gain{1.2};
  double near_goal_obstacle_check_range{0.2};
  double near_goal_obstacle_check_margin{0.1};
};

struct LocalPlannerOutput {
  bool path_updated{false};
  bool free_paths_updated{false};
  common_struct::Path path;
  common_struct::Twist command;
  pcl::PointCloud<pcl::PointXYZI> free_paths;
};

// Middleware-independent local planning and control core. The caller owns all
// time, transport, frame naming, parameter loading, and publication policy.
class LocalPlanner {
 public:
  explicit LocalPlanner(const LocalPlannerConfig& config);
  ~LocalPlanner();

  LocalPlanner(LocalPlanner&&) noexcept;
  LocalPlanner& operator=(LocalPlanner&&) noexcept;
  LocalPlanner(const LocalPlanner&) = delete;
  LocalPlanner& operator=(const LocalPlanner&) = delete;

  void SetOdometry(double timestamp_seconds,
                   const common_struct::Pose& pose);
  void SetRegisteredScan(
      const pcl::PointCloud<pcl::PointXYZI>& registered_scan);
  void SetTerrain(const pcl::PointCloud<pcl::PointXYZI>& terrain);
  void SetGoal(const common_struct::Pose& goal);
  void SetGlobalPath(const common_struct::Path& path);
  void SetSpeed(double speed);
  void SetBoundary(const common_struct::PolygonStamped& boundary);
  void SetAddedObstacles(
      const pcl::PointCloud<pcl::PointXYZI>& obstacles);
  void SetObstacleChecking(bool enabled);
  void SetSafetyStop(std::int8_t stop_mask);
  void SetGoalValid(bool valid);
  LocalPlannerOutput Step(double now_seconds);

  double control_frequency() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace planning
}  // namespace jojo
