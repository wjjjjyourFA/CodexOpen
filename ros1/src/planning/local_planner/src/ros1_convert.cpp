#include "local_planner_ros1/ros1_convert.h"

#include <exception>

#include <pcl_conversions/pcl_conversions.h>

namespace jojo {
namespace planning {
namespace ros1 {

LocalPlannerRos1Convert::LocalPlannerRos1Convert(
    ::ros::NodeHandle& node, ::ros::NodeHandle& private_node)
    : node_(node), private_node_(private_node) {}

bool LocalPlannerRos1Convert::Init() {
  LocalPlannerConfig config;
  private_node_.param("pathFolder", config.path_folder, config.path_folder);
  private_node_.param("vehicleLength", config.vehicle_length,
                      config.vehicle_length);
  private_node_.param("vehicleWidth", config.vehicle_width,
                      config.vehicle_width);
  private_node_.param("sensorOffsetX", config.sensor_offset_x,
                      config.sensor_offset_x);
  private_node_.param("sensorOffsetY", config.sensor_offset_y,
                      config.sensor_offset_y);
  private_node_.param("twoWayDrive", config.two_way_drive,
                      config.two_way_drive);
  private_node_.param("laserVoxelSize", config.laser_voxel_size,
                      config.laser_voxel_size);
  private_node_.param("terrainVoxelSize", config.terrain_voxel_size,
                      config.terrain_voxel_size);
  private_node_.param("useTerrainAnalysis", config.use_terrain_analysis,
                      config.use_terrain_analysis);
  private_node_.param("checkObstacle", config.check_obstacle,
                      config.check_obstacle);
  private_node_.param("checkRotObstacle", config.check_rotation_obstacle,
                      config.check_rotation_obstacle);
  private_node_.param("adjacentRange", config.adjacent_range,
                      config.adjacent_range);
  private_node_.param("obstacleHeightThre",
                      config.obstacle_height_threshold,
                      config.obstacle_height_threshold);
  private_node_.param("groundHeightThre", config.ground_height_threshold,
                      config.ground_height_threshold);
  private_node_.param("costHeightThre", config.cost_height_threshold,
                      config.cost_height_threshold);
  private_node_.param("costScore", config.cost_score, config.cost_score);
  private_node_.param("obstacleInflationRadius",
                      config.obstacle_inflation_radius,
                      config.obstacle_inflation_radius);
  private_node_.param("inflatedObstaclePenalty",
                      config.inflated_obstacle_penalty,
                      config.inflated_obstacle_penalty);
  private_node_.param("centerPathBias", config.center_path_bias,
                      config.center_path_bias);
  private_node_.param("pathContinuityWeight",
                      config.path_continuity_weight,
                      config.path_continuity_weight);
  private_node_.param("groupContinuityWeight",
                      config.group_continuity_weight,
                      config.group_continuity_weight);
  private_node_.param("sideSwitchPenalty", config.side_switch_penalty,
                      config.side_switch_penalty);
  private_node_.param("largeSwitchAngleDeg",
                      config.large_switch_angle_degrees,
                      config.large_switch_angle_degrees);
  private_node_.param("holdPathScoreRatio", config.hold_path_score_ratio,
                      config.hold_path_score_ratio);
  private_node_.param("useCost", config.use_cost, config.use_cost);
  private_node_.param("pointPerPathThre",
                      config.point_per_path_threshold,
                      config.point_per_path_threshold);
  private_node_.param("minRelZ", config.min_relative_z,
                      config.min_relative_z);
  private_node_.param("maxRelZ", config.max_relative_z,
                      config.max_relative_z);
  private_node_.param("maxSpeed", config.max_speed, config.max_speed);
  private_node_.param("dirWeight", config.direction_weight,
                      config.direction_weight);
  private_node_.param("dirThre", config.direction_threshold,
                      config.direction_threshold);
  private_node_.param("dirToVehicle", config.direction_to_vehicle,
                      config.direction_to_vehicle);
  private_node_.param("pathScale", config.path_scale, config.path_scale);
  private_node_.param("minPathScale", config.min_path_scale,
                      config.min_path_scale);
  private_node_.param("pathScaleStep", config.path_scale_step,
                      config.path_scale_step);
  private_node_.param("pathScaleBySpeed", config.path_scale_by_speed,
                      config.path_scale_by_speed);
  private_node_.param("minPathRange", config.min_path_range,
                      config.min_path_range);
  private_node_.param("pathRangeStep", config.path_range_step,
                      config.path_range_step);
  private_node_.param("pathRangeBySpeed", config.path_range_by_speed,
                      config.path_range_by_speed);
  private_node_.param("pathCropByGoal", config.path_crop_by_goal,
                      config.path_crop_by_goal);
  private_node_.param("autonomyMode", config.autonomy_mode,
                      config.autonomy_mode);
  private_node_.param("autonomySpeed", config.autonomy_speed,
                      config.autonomy_speed);
  private_node_.param("goalClearRange", config.goal_clear_range,
                      config.goal_clear_range);
  private_node_.param("goalX", config.goal_x, config.goal_x);
  private_node_.param("goalY", config.goal_y, config.goal_y);
  private_node_.param("globalPathLookAhead",
                      config.global_path_look_ahead,
                      config.global_path_look_ahead);
  private_node_.param("globalPathGoalSwitchDis",
                      config.global_path_goal_switch_distance,
                      config.global_path_goal_switch_distance);
  private_node_.param("controlLookAheadDis",
                      config.control_look_ahead_distance,
                      config.control_look_ahead_distance);
  private_node_.param("forwardAlignAngle",
                      config.forward_align_angle_degrees,
                      config.forward_align_angle_degrees);
  private_node_.param("yawRateGain", config.yaw_rate_gain,
                      config.yaw_rate_gain);
  private_node_.param("maxYawRate", config.max_yaw_rate_degrees,
                      config.max_yaw_rate_degrees);
  private_node_.param("maxAccel", config.max_acceleration,
                      config.max_acceleration);
  private_node_.param("maxYawAccel",
                      config.max_yaw_acceleration_degrees,
                      config.max_yaw_acceleration_degrees);
  private_node_.param("goalStopDistance", config.goal_stop_distance,
                      config.goal_stop_distance);
  private_node_.param("goalSlowDistance", config.goal_slow_distance,
                      config.goal_slow_distance);
  private_node_.param("planTimeout", config.plan_timeout,
                      config.plan_timeout);
  private_node_.param("controlFrequency", config.control_frequency,
                      config.control_frequency);
  private_node_.param("nearGoalEnterDistance",
                      config.near_goal_enter_distance,
                      config.near_goal_enter_distance);
  private_node_.param("nearGoalExitDistance",
                      config.near_goal_exit_distance,
                      config.near_goal_exit_distance);
  private_node_.param("nearGoalXYTolerance", config.near_goal_xy_tolerance,
                      config.near_goal_xy_tolerance);
  private_node_.param("nearGoalYawToleranceDeg",
                      config.near_goal_yaw_tolerance_degrees,
                      config.near_goal_yaw_tolerance_degrees);
  private_node_.param("nearGoalMinSpeed", config.near_goal_min_speed,
                      config.near_goal_min_speed);
  private_node_.param("nearGoalMinYawRateDeg",
                      config.near_goal_min_yaw_rate_degrees,
                      config.near_goal_min_yaw_rate_degrees);
  private_node_.param("nearGoalPositionGain",
                      config.near_goal_position_gain,
                      config.near_goal_position_gain);
  private_node_.param("nearGoalYawGain", config.near_goal_yaw_gain,
                      config.near_goal_yaw_gain);
  private_node_.param("nearGoalObstacleCheckRange",
                      config.near_goal_obstacle_check_range,
                      config.near_goal_obstacle_check_range);
  private_node_.param("nearGoalObstacleCheckMargin",
                      config.near_goal_obstacle_check_margin,
                      config.near_goal_obstacle_check_margin);

  private_node_.param("odometry_topic", odometry_topic_, odometry_topic_);
  private_node_.param("registered_scan_topic", registered_scan_topic_,
                      registered_scan_topic_);
  private_node_.param("terrain_topic", terrain_topic_, terrain_topic_);
  private_node_.param("goal_topic", goal_topic_, goal_topic_);
  private_node_.param("global_path_topic", global_path_topic_,
                      global_path_topic_);
  private_node_.param("speed_topic", speed_topic_, speed_topic_);
  private_node_.param("boundary_topic", boundary_topic_, boundary_topic_);
  private_node_.param("added_obstacles_topic", added_obstacles_topic_,
                      added_obstacles_topic_);
  private_node_.param("stop_topic", stop_topic_, stop_topic_);
  private_node_.param("goal_valid_topic", goal_valid_topic_,
                      goal_valid_topic_);
  private_node_.param("path_topic", path_topic_, path_topic_);
  private_node_.param("command_topic", command_topic_, command_topic_);
  private_node_.param("free_paths_topic", free_paths_topic_,
                      free_paths_topic_);
  private_node_.param("vehicle_frame", vehicle_frame_, vehicle_frame_);
  private_node_.param("queues/input", input_queue_size_, input_queue_size_);
  private_node_.param("queues/path", path_queue_size_, path_queue_size_);
  private_node_.param("queues/command", command_queue_size_,
                      command_queue_size_);
  private_node_.param("queues/free_paths", free_paths_queue_size_,
                      free_paths_queue_size_);
  if (input_queue_size_ <= 0 || path_queue_size_ <= 0 ||
      command_queue_size_ <= 0 || free_paths_queue_size_ <= 0) {
    ROS_ERROR("local_planner interface queues must be positive");
    return false;
  }

  try {
    core_.reset(new LocalPlanner(config));
  } catch (const std::exception& exception) {
    ROS_ERROR_STREAM("Failed to initialize local planner core: "
                     << exception.what());
    return false;
  }

  odometry_subscriber_ = node_.subscribe<nav_msgs::Odometry>(
      odometry_topic_, input_queue_size_,
      &LocalPlannerRos1Convert::OdometryCallback, this);
  registered_scan_subscriber_ = node_.subscribe<sensor_msgs::PointCloud2>(
      registered_scan_topic_, input_queue_size_,
      &LocalPlannerRos1Convert::RegisteredScanCallback, this);
  terrain_subscriber_ = node_.subscribe<sensor_msgs::PointCloud2>(
      terrain_topic_, input_queue_size_,
      &LocalPlannerRos1Convert::TerrainCallback, this);
  goal_subscriber_ = node_.subscribe<geometry_msgs::PoseStamped>(
      goal_topic_, input_queue_size_,
      &LocalPlannerRos1Convert::GoalCallback, this);
  global_path_subscriber_ = node_.subscribe<nav_msgs::Path>(
      global_path_topic_, input_queue_size_,
      &LocalPlannerRos1Convert::GlobalPathCallback, this);
  speed_subscriber_ = node_.subscribe<std_msgs::Float32>(
      speed_topic_, input_queue_size_,
      &LocalPlannerRos1Convert::SpeedCallback, this);
  boundary_subscriber_ = node_.subscribe<geometry_msgs::PolygonStamped>(
      boundary_topic_, input_queue_size_,
      &LocalPlannerRos1Convert::BoundaryCallback, this);
  added_obstacles_subscriber_ = node_.subscribe<sensor_msgs::PointCloud2>(
      added_obstacles_topic_, input_queue_size_,
      &LocalPlannerRos1Convert::AddedObstaclesCallback, this);
  stop_subscriber_ = node_.subscribe<std_msgs::Int8>(
      stop_topic_, input_queue_size_,
      &LocalPlannerRos1Convert::StopCallback, this);
  goal_valid_subscriber_ = node_.subscribe<std_msgs::Bool>(
      goal_valid_topic_, input_queue_size_,
      &LocalPlannerRos1Convert::GoalValidCallback, this);
  path_publisher_ = node_.advertise<nav_msgs::Path>(path_topic_,
                                                    path_queue_size_);
  command_publisher_ =
      node_.advertise<geometry_msgs::Twist>(command_topic_,
                                            command_queue_size_);
  free_paths_publisher_ =
      node_.advertise<sensor_msgs::PointCloud2>(free_paths_topic_,
                                                free_paths_queue_size_);
  return true;
}

common_struct::Pose LocalPlannerRos1Convert::FromRosPose(
    const geometry_msgs::Pose& pose) {
  common_struct::Pose converted;
  converted.position.x = pose.position.x;
  converted.position.y = pose.position.y;
  converted.position.z = pose.position.z;
  converted.orientation.x = pose.orientation.x;
  converted.orientation.y = pose.orientation.y;
  converted.orientation.z = pose.orientation.z;
  converted.orientation.w = pose.orientation.w;
  return converted;
}

geometry_msgs::Pose LocalPlannerRos1Convert::ToRosPose(
    const common_struct::Pose& pose) {
  geometry_msgs::Pose converted;
  converted.position.x = pose.position.x;
  converted.position.y = pose.position.y;
  converted.position.z = pose.position.z;
  converted.orientation.x = pose.orientation.x;
  converted.orientation.y = pose.orientation.y;
  converted.orientation.z = pose.orientation.z;
  converted.orientation.w = pose.orientation.w;
  return converted;
}

common_struct::Path LocalPlannerRos1Convert::FromRosPath(
    const nav_msgs::Path& path) {
  common_struct::Path converted;
  converted.header.timestamp = path.header.stamp.toNSec();
  converted.header.seq = path.header.seq;
  converted.header.frame_id = path.header.frame_id;
  converted.poses.reserve(path.poses.size());
  for (const geometry_msgs::PoseStamped& pose : path.poses) {
    common_struct::PoseStamped converted_pose;
    converted_pose.header.timestamp = pose.header.stamp.toNSec();
    converted_pose.header.seq = pose.header.seq;
    converted_pose.header.frame_id = pose.header.frame_id;
    converted_pose.pose = FromRosPose(pose.pose);
    converted.poses.push_back(converted_pose);
  }
  return converted;
}

nav_msgs::Path LocalPlannerRos1Convert::ToRosPath(
    const common_struct::Path& path) const {
  nav_msgs::Path converted;
  converted.header.stamp.fromNSec(path.header.timestamp);
  converted.header.seq = path.header.seq;
  converted.header.frame_id = vehicle_frame_;
  converted.poses.reserve(path.poses.size());
  for (const common_struct::PoseStamped& pose : path.poses) {
    geometry_msgs::PoseStamped converted_pose;
    converted_pose.header = converted.header;
    converted_pose.pose = ToRosPose(pose.pose);
    converted.poses.push_back(converted_pose);
  }
  return converted;
}

geometry_msgs::Twist LocalPlannerRos1Convert::ToRosTwist(
    const common_struct::Twist& twist) {
  geometry_msgs::Twist converted;
  converted.linear.x = twist.linear.x;
  converted.linear.y = twist.linear.y;
  converted.linear.z = twist.linear.z;
  converted.angular.x = twist.angular.x;
  converted.angular.y = twist.angular.y;
  converted.angular.z = twist.angular.z;
  return converted;
}

void LocalPlannerRos1Convert::OdometryCallback(
    const nav_msgs::OdometryConstPtr& message) {
  core_->SetOdometry(message->header.stamp.toSec(),
                     FromRosPose(message->pose.pose));
}

void LocalPlannerRos1Convert::RegisteredScanCallback(
    const sensor_msgs::PointCloud2ConstPtr& message) {
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::fromROSMsg(*message, cloud);
  core_->SetRegisteredScan(cloud);
}

void LocalPlannerRos1Convert::TerrainCallback(
    const sensor_msgs::PointCloud2ConstPtr& message) {
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::fromROSMsg(*message, cloud);
  core_->SetTerrain(cloud);
}

void LocalPlannerRos1Convert::GoalCallback(
    const geometry_msgs::PoseStampedConstPtr& message) {
  core_->SetGoal(FromRosPose(message->pose));
}

void LocalPlannerRos1Convert::GlobalPathCallback(
    const nav_msgs::PathConstPtr& message) {
  core_->SetGlobalPath(FromRosPath(*message));
}

void LocalPlannerRos1Convert::SpeedCallback(
    const std_msgs::Float32ConstPtr& message) {
  core_->SetSpeed(message->data);
}

void LocalPlannerRos1Convert::BoundaryCallback(
    const geometry_msgs::PolygonStampedConstPtr& message) {
  common_struct::PolygonStamped boundary;
  boundary.header.timestamp = message->header.stamp.toNSec();
  boundary.header.seq = message->header.seq;
  boundary.header.frame_id = message->header.frame_id;
  boundary.points.reserve(message->polygon.points.size());
  for (const geometry_msgs::Point32& point : message->polygon.points) {
    boundary.points.emplace_back(point.x, point.y, point.z);
  }
  core_->SetBoundary(boundary);
}

void LocalPlannerRos1Convert::AddedObstaclesCallback(
    const sensor_msgs::PointCloud2ConstPtr& message) {
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::fromROSMsg(*message, cloud);
  core_->SetAddedObstacles(cloud);
}

void LocalPlannerRos1Convert::StopCallback(
    const std_msgs::Int8ConstPtr& message) {
  core_->SetSafetyStop(message->data);
}

void LocalPlannerRos1Convert::GoalValidCallback(
    const std_msgs::BoolConstPtr& message) {
  core_->SetGoalValid(message->data);
}

void LocalPlannerRos1Convert::Run() {
  ::ros::Rate rate(core_->control_frequency());
  while (::ros::ok()) {
    ::ros::spinOnce();
    const LocalPlannerOutput output = core_->Step(::ros::Time::now().toSec());
    if (output.path_updated) {
      path_publisher_.publish(ToRosPath(output.path));
    }
    if (output.free_paths_updated) {
      sensor_msgs::PointCloud2 free_paths_message;
      pcl::toROSMsg(output.free_paths, free_paths_message);
      free_paths_message.header.stamp.fromNSec(output.path.header.timestamp);
      free_paths_message.header.frame_id = vehicle_frame_;
      free_paths_publisher_.publish(free_paths_message);
    }
    command_publisher_.publish(ToRosTwist(output.command));
    rate.sleep();
  }
}

}  // namespace ros1
}  // namespace planning
}  // namespace jojo
