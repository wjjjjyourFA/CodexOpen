#include "world_planner_ros1/ros1_convert.h"

#include <algorithm>
#include <exception>

#include <pcl_conversions/pcl_conversions.h>

namespace jojo {
namespace planning {
namespace ros1 {

Ros1Convert::Ros1Convert(::ros::NodeHandle& node,
                         ::ros::NodeHandle& private_node)
    : node_(node), private_node_(private_node) {}

bool Ros1Convert::Init() {
  WorldPlannerConfig config;
  private_node_.param("grid_resolution", config.grid_resolution,
                      config.grid_resolution);
  private_node_.param("map_extra_margin", config.map_extra_margin,
                      config.map_extra_margin);
  private_node_.param("robot_width", config.robot_width, config.robot_width);
  private_node_.param("robot_length", config.robot_length,
                      config.robot_length);
  private_node_.param("inflation_margin", config.inflation_margin,
                      config.inflation_margin);
  private_node_.param("obstacle_height_thre",
                      config.obstacle_height_threshold,
                      config.obstacle_height_threshold);
  private_node_.param("path_point_spacing", config.path_point_spacing,
                      config.path_point_spacing);
  private_node_.param("goal_reached_xy", config.goal_reached_xy,
                      config.goal_reached_xy);
  private_node_.param("path_check_step", config.path_check_step,
                      config.path_check_step);
  private_node_.param("reuse_search_radius", config.reuse_search_radius,
                      config.reuse_search_radius);
  private_node_.param("goal_change_replan_dis",
                      config.goal_change_replan_distance,
                      config.goal_change_replan_distance);
  private_node_.param("heuristic_weight", config.heuristic_weight,
                      config.heuristic_weight);
  private_node_.param("waypoint_file_dir", config.waypoint_file,
                      std::string());
  private_node_.param("world_frame", config.world_frame,
                      config.world_frame);
  private_node_.param("transport/planning_frequency", planning_frequency_,
                      planning_frequency_);

  private_node_.param("terrain_topic", terrain_topic_, terrain_topic_);
  private_node_.param("odometry_topic", odometry_topic_, odometry_topic_);
  private_node_.param("goal_topic", goal_topic_, goal_topic_);
  private_node_.param("path_topic", path_topic_, path_topic_);
  private_node_.param("obstacle_grid_topic", obstacle_grid_topic_,
                      obstacle_grid_topic_);
  private_node_.param("queues/terrain", terrain_queue_size_,
                      terrain_queue_size_);
  private_node_.param("queues/odometry", odometry_queue_size_,
                      odometry_queue_size_);
  private_node_.param("queues/goal", goal_queue_size_, goal_queue_size_);
  private_node_.param("queues/path", path_queue_size_, path_queue_size_);
  private_node_.param("queues/obstacle_grid", obstacle_grid_queue_size_,
                      obstacle_grid_queue_size_);
  private_node_.param("transport/path_latched", path_latched_,
                      path_latched_);
  private_node_.param("transport/obstacle_grid_latched",
                      obstacle_grid_latched_, obstacle_grid_latched_);
  if (planning_frequency_ <= 0.0 || terrain_queue_size_ <= 0 ||
      odometry_queue_size_ <= 0 || goal_queue_size_ <= 0 ||
      path_queue_size_ <= 0 || obstacle_grid_queue_size_ <= 0) {
    ROS_ERROR("world_planner interface rates/queues must be positive");
    return false;
  }

  try {
    planner_.reset(new WorldPlanner(config));
  } catch (const std::exception& error) {
    ROS_ERROR_STREAM("world_planner configuration failed: " << error.what());
    return false;
  }

  terrain_subscriber_ = node_.subscribe<sensor_msgs::PointCloud2>(
      terrain_topic_, terrain_queue_size_, &Ros1Convert::TerrainCallback,
      this);
  odometry_subscriber_ = node_.subscribe<nav_msgs::Odometry>(
      odometry_topic_, odometry_queue_size_,
      &Ros1Convert::OdometryCallback, this);
  goal_subscriber_ = node_.subscribe<geometry_msgs::PoseStamped>(
      goal_topic_, goal_queue_size_, &Ros1Convert::GoalCallback, this);
  path_publisher_ = node_.advertise<nav_msgs::Path>(
      path_topic_, path_queue_size_, path_latched_);
  obstacle_grid_publisher_ =
      node_.advertise<nav_msgs::OccupancyGrid>(
          obstacle_grid_topic_, obstacle_grid_queue_size_,
          obstacle_grid_latched_);
  return true;
}

void Ros1Convert::TerrainCallback(
    const sensor_msgs::PointCloud2ConstPtr& message) {
  pcl::PointCloud<pcl::PointXYZI> terrain;
  pcl::fromROSMsg(*message, terrain);
  planner_->SetTerrain(terrain);
}

void Ros1Convert::OdometryCallback(
    const nav_msgs::OdometryConstPtr& message) {
  planner_->SetOdometry(ConvertPose(message->pose.pose));
}

void Ros1Convert::GoalCallback(
    const geometry_msgs::PoseStampedConstPtr& message) {
  planner_->SetGoal(ConvertPose(message->pose));
}

common_struct::Pose Ros1Convert::ConvertPose(const geometry_msgs::Pose& pose) {
  common_struct::Pose result;
  result.position = common_struct::Vector3d(
      pose.position.x, pose.position.y, pose.position.z);
  result.orientation.x = pose.orientation.x;
  result.orientation.y = pose.orientation.y;
  result.orientation.z = pose.orientation.z;
  result.orientation.w = pose.orientation.w;
  return result;
}

geometry_msgs::Pose Ros1Convert::ConvertPose(
    const common_struct::Pose& pose) {
  geometry_msgs::Pose result;
  result.position.x = pose.position.x;
  result.position.y = pose.position.y;
  result.position.z = pose.position.z;
  result.orientation.x = pose.orientation.x;
  result.orientation.y = pose.orientation.y;
  result.orientation.z = pose.orientation.z;
  result.orientation.w = pose.orientation.w;
  return result;
}

nav_msgs::Path Ros1Convert::ConvertPath(const common_struct::Path& path) {
  nav_msgs::Path result;
  result.header.seq = path.header.seq;
  result.header.stamp.fromNSec(path.header.timestamp);
  result.header.frame_id = path.header.frame_id;
  result.poses.reserve(path.poses.size());
  for (const common_struct::PoseStamped& source : path.poses) {
    geometry_msgs::PoseStamped pose;
    pose.header.seq = source.header.seq;
    pose.header.stamp.fromNSec(source.header.timestamp);
    pose.header.frame_id = source.header.frame_id;
    pose.pose = ConvertPose(source.pose);
    result.poses.push_back(pose);
  }
  return result;
}

nav_msgs::OccupancyGrid Ros1Convert::ConvertOccupancyGrid(
    const common_struct::OccupancyGrid& grid) {
  nav_msgs::OccupancyGrid result;
  result.header.seq = grid.header.seq;
  result.header.stamp.fromNSec(grid.header.timestamp);
  result.header.frame_id = grid.header.frame_id;
  result.info.map_load_time.fromNSec(grid.info.map_load_time);
  result.info.resolution = grid.info.resolution;
  result.info.width = grid.info.width;
  result.info.height = grid.info.height;
  result.info.origin = ConvertPose(grid.info.origin);
  result.data = grid.data;
  return result;
}

void Ros1Convert::Run() {
  ::ros::Rate rate(planning_frequency_);
  while (::ros::ok()) {
    ::ros::spinOnce();
    const WorldPlannerOutput output = planner_->Step(::ros::Time::now().toNSec());
    if (output.ready) {
      path_publisher_.publish(ConvertPath(output.path));
      if (output.has_obstacle_grid) {
        obstacle_grid_publisher_.publish(
            ConvertOccupancyGrid(output.obstacle_grid));
      }
    }
    rate.sleep();
  }
}

}  // namespace ros1
}  // namespace planning
}  // namespace jojo
