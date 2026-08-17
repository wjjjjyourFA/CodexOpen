#include "terrain_waypoint_exploration_ros1/ros1_convert.h"

#include <algorithm>

#include <pcl_conversions/pcl_conversions.h>

namespace terrain_waypoint_exploration {
namespace ros1 {

TerrainWaypointExplorerRos1Convert::TerrainWaypointExplorerRos1Convert(
    ::ros::NodeHandle& node, ::ros::NodeHandle& private_node)
    : node_(node), private_node_(private_node) {}

bool TerrainWaypointExplorerRos1Convert::Init() {
  TerrainWaypointExplorerConfig config;
  private_node_.param("grid_resolution", config.grid_resolution,
                      config.grid_resolution);
  private_node_.param("height_layer_resolution",
                      config.height_layer_resolution,
                      config.height_layer_resolution);
  private_node_.param("occupied_min_layers", config.occupied_min_layers,
                      config.occupied_min_layers);
  private_node_.param("ground_max_layers", config.ground_max_layers,
                      config.ground_max_layers);
  private_node_.param("ground_below_sensor", config.ground_below_sensor,
                      config.ground_below_sensor);
  private_node_.param("scan_point_stride", config.scan_point_stride,
                      config.scan_point_stride);
  private_node_.param("scan_max_range", config.scan_max_range,
                      config.scan_max_range);
  private_node_.param("planning_frequency", config.planning_frequency,
                      config.planning_frequency);
  private_node_.param("min_goal_distance", config.min_goal_distance,
                      config.min_goal_distance);
  private_node_.param("reachable_search_radius",
                      config.reachable_search_radius,
                      config.reachable_search_radius);
  private_node_.param("waypoint_lookahead_distance",
                      config.waypoint_lookahead_distance,
                      config.waypoint_lookahead_distance);
  private_node_.param("goal_reached_distance", config.goal_reached_distance,
                      config.goal_reached_distance);
  private_node_.param("goal_timeout", config.goal_timeout,
                      config.goal_timeout);
  private_node_.param("goal_publish_frequency",
                      config.goal_publish_frequency,
                      config.goal_publish_frequency);
  private_node_.param("obstacle_clearance", config.obstacle_clearance,
                      config.obstacle_clearance);
  private_node_.param("frontier_cluster_radius",
                      config.frontier_cluster_radius,
                      config.frontier_cluster_radius);
  private_node_.param("min_frontier_cluster_cells",
                      config.min_frontier_cluster_cells,
                      config.min_frontier_cluster_cells);
  private_node_.param("information_radius", config.information_radius,
                      config.information_radius);
  private_node_.param("gain_weight", config.gain_weight,
                      config.gain_weight);
  private_node_.param("distance_weight", config.distance_weight,
                      config.distance_weight);
  private_node_.param("heading_weight", config.heading_weight,
                      config.heading_weight);
  private_node_.param("continuation_angle_deg",
                      config.continuation_angle_deg,
                      config.continuation_angle_deg);
  private_node_.param("continuation_bonus", config.continuation_bonus,
                      config.continuation_bonus);
  private_node_.param("continuation_target_radius",
                      config.continuation_target_radius,
                      config.continuation_target_radius);
  private_node_.param("saved_branch_match_radius",
                      config.saved_branch_match_radius,
                      config.saved_branch_match_radius);
  private_node_.param("saved_branch_merge_radius",
                      config.saved_branch_merge_radius,
                      config.saved_branch_merge_radius);
  private_node_.param("blacklist_radius", config.blacklist_radius,
                      config.blacklist_radius);
  private_node_.param("blacklist_duration", config.blacklist_duration,
                      config.blacklist_duration);
  private_node_.param("finish_no_frontier_cycles",
                      config.finish_no_frontier_cycles,
                      config.finish_no_frontier_cycles);

  private_node_.param("world_frame", world_frame_, world_frame_);
  private_node_.param("odometry_topic", odometry_topic_, odometry_topic_);
  private_node_.param("registered_scan_topic", registered_scan_topic_,
                      registered_scan_topic_);
  private_node_.param("terrain_topic", terrain_topic_, terrain_topic_);
  private_node_.param("waypoint_topic", waypoint_topic_, waypoint_topic_);
  private_node_.param("goal_valid_topic", goal_valid_topic_,
                      goal_valid_topic_);
  private_node_.param("finished_topic", finished_topic_, finished_topic_);
  private_node_.param("debug_map_topic", debug_map_topic_,
                      debug_map_topic_);
  private_node_.param("debug_frontiers_topic", debug_frontiers_topic_,
                      debug_frontiers_topic_);
  private_node_.param("debug_candidates_topic", debug_candidates_topic_,
                      debug_candidates_topic_);
  private_node_.param("debug_goal_topic", debug_goal_topic_,
                      debug_goal_topic_);
  private_node_.param("queues/odometry", odometry_queue_size_,
                      odometry_queue_size_);
  private_node_.param("queues/scan", scan_queue_size_, scan_queue_size_);
  private_node_.param("queues/terrain", terrain_queue_size_,
                      terrain_queue_size_);
  private_node_.param("queues/waypoint", waypoint_queue_size_,
                      waypoint_queue_size_);
  private_node_.param("queues/goal_valid", goal_valid_queue_size_,
                      goal_valid_queue_size_);
  private_node_.param("queues/debug", debug_queue_size_, debug_queue_size_);
  private_node_.param("queues/finished", finished_queue_size_,
                      finished_queue_size_);
  private_node_.param("transport/output_latched", output_latched_,
                      output_latched_);
  if (odometry_queue_size_ <= 0 || scan_queue_size_ <= 0 ||
      terrain_queue_size_ <= 0 || waypoint_queue_size_ <= 0 ||
      goal_valid_queue_size_ <= 0 || debug_queue_size_ <= 0 ||
      finished_queue_size_ <= 0) {
    ROS_ERROR("terrain_waypoint_exploration queues must be positive");
    return false;
  }

  core_.reset(new TerrainWaypointExplorer(config));
  odometry_subscriber_ = node_.subscribe<nav_msgs::Odometry>(
      odometry_topic_, odometry_queue_size_,
      &TerrainWaypointExplorerRos1Convert::OdometryCallback, this);
  scan_subscriber_ = node_.subscribe<sensor_msgs::PointCloud2>(
      registered_scan_topic_, scan_queue_size_,
      &TerrainWaypointExplorerRos1Convert::ScanCallback, this);
  terrain_subscriber_ = node_.subscribe<sensor_msgs::PointCloud2>(
      terrain_topic_, terrain_queue_size_,
      &TerrainWaypointExplorerRos1Convert::TerrainCallback, this);

  waypoint_publisher_ =
      node_.advertise<geometry_msgs::PoseStamped>(
          waypoint_topic_, waypoint_queue_size_, output_latched_);
  goal_valid_publisher_ =
      node_.advertise<std_msgs::Bool>(
          goal_valid_topic_, goal_valid_queue_size_, output_latched_);
  map_publisher_ = private_node_.advertise<sensor_msgs::PointCloud2>(
      debug_map_topic_, debug_queue_size_, output_latched_);
  frontier_publisher_ = private_node_.advertise<sensor_msgs::PointCloud2>(
      debug_frontiers_topic_, debug_queue_size_, output_latched_);
  candidate_publisher_ = private_node_.advertise<sensor_msgs::PointCloud2>(
      debug_candidates_topic_, debug_queue_size_, output_latched_);
  goal_marker_publisher_ = private_node_.advertise<visualization_msgs::Marker>(
      debug_goal_topic_, debug_queue_size_, output_latched_);
  finished_publisher_ =
      node_.advertise<std_msgs::Bool>(
          finished_topic_, finished_queue_size_, output_latched_);
  planning_timer_ = node_.createTimer(
      ::ros::Duration(1.0 / std::max(0.1, core_->planning_frequency())),
      &TerrainWaypointExplorerRos1Convert::PlanningTimerCallback, this);

  std_msgs::Bool initial_validity;
  initial_validity.data = false;
  goal_valid_publisher_.publish(initial_validity);
  ROS_INFO_STREAM("terrain_waypoint_exploration started. Inputs: "
                  << odometry_topic_ << ", " << registered_scan_topic_ << ", "
                  << terrain_topic_ << "; output: " << waypoint_topic_);
  return true;
}

jojo::common_struct::Pose TerrainWaypointExplorerRos1Convert::FromRosPose(
    const geometry_msgs::Pose& pose) {
  jojo::common_struct::Pose converted;
  converted.position.x = pose.position.x;
  converted.position.y = pose.position.y;
  converted.position.z = pose.position.z;
  converted.orientation.x = pose.orientation.x;
  converted.orientation.y = pose.orientation.y;
  converted.orientation.z = pose.orientation.z;
  converted.orientation.w = pose.orientation.w;
  return converted;
}

geometry_msgs::Pose TerrainWaypointExplorerRos1Convert::ToRosPose(
    const jojo::common_struct::Pose& pose) {
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

void TerrainWaypointExplorerRos1Convert::OdometryCallback(
    const nav_msgs::OdometryConstPtr& message) {
  std::lock_guard<std::mutex> lock(core_mutex_);
  core_->SetOdometry(FromRosPose(message->pose.pose));
}

void TerrainWaypointExplorerRos1Convert::ScanCallback(
    const sensor_msgs::PointCloud2ConstPtr& message) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*message, cloud);
  std::lock_guard<std::mutex> lock(core_mutex_);
  core_->AddRegisteredScan(message->header.stamp.toSec(), cloud);
}

void TerrainWaypointExplorerRos1Convert::TerrainCallback(
    const sensor_msgs::PointCloud2ConstPtr& message) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*message, cloud);
  std::lock_guard<std::mutex> lock(core_mutex_);
  core_->AddTerrain(message->header.stamp.toSec(), cloud);
}

void TerrainWaypointExplorerRos1Convert::PlanningTimerCallback(
    const ::ros::TimerEvent&) {
  const ::ros::Time now = ::ros::Time::now();
  TerrainWaypointExplorerOutput output;
  {
    std::lock_guard<std::mutex> lock(core_mutex_);
    output = core_->Plan(now.toSec());
  }
  PublishOutput(output, now);
}

void TerrainWaypointExplorerRos1Convert::PublishOutput(
    const TerrainWaypointExplorerOutput& output,
    const ::ros::Time& timestamp) {
  std_msgs::Bool valid_message;
  valid_message.data = output.goal_valid;
  goal_valid_publisher_.publish(valid_message);

  if (output.publish_goal) {
    geometry_msgs::PoseStamped goal_message;
    goal_message.header.stamp = timestamp;
    goal_message.header.frame_id = world_frame_;
    goal_message.pose = ToRosPose(output.goal);
    waypoint_publisher_.publish(goal_message);
  }

  if (output.exploration_finished) {
    std_msgs::Bool finished_message;
    finished_message.data = true;
    finished_publisher_.publish(finished_message);
  }

  if (output.status != ExplorerStatus::kWaitingForInputs &&
      output.status != ExplorerStatus::kNoTraversableStart) {
    sensor_msgs::PointCloud2 map_message;
    pcl::toROSMsg(output.debug_map, map_message);
    map_message.header.stamp = timestamp;
    map_message.header.frame_id = world_frame_;
    map_publisher_.publish(map_message);

    sensor_msgs::PointCloud2 frontier_message;
    pcl::toROSMsg(output.debug_frontiers, frontier_message);
    frontier_message.header = map_message.header;
    frontier_publisher_.publish(frontier_message);

    sensor_msgs::PointCloud2 candidate_message;
    pcl::toROSMsg(output.debug_candidates, candidate_message);
    candidate_message.header = map_message.header;
    candidate_publisher_.publish(candidate_message);

    visualization_msgs::Marker marker;
    marker.header = map_message.header;
    marker.ns = "terrain_waypoint_exploration";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = output.goal_valid ? visualization_msgs::Marker::ADD
                                      : visualization_msgs::Marker::DELETE;
    marker.pose = ToRosPose(output.goal);
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.color.r = 1.0;
    marker.color.g = 0.2;
    marker.color.b = 0.1;
    marker.color.a = 1.0;
    goal_marker_publisher_.publish(marker);
  }

  switch (output.status) {
    case ExplorerStatus::kWaitingForInputs:
      ROS_WARN_THROTTLE(
          5.0, "Waiting for odometry, registered scan, and terrain map.");
      break;
    case ExplorerStatus::kNoTraversableStart:
      ROS_WARN_THROTTLE(
          2.0, "No traversable cell found around the current odometry.");
      break;
    case ExplorerStatus::kWaypointTimedOut:
      ROS_WARN_STREAM("Tracking waypoint timed out. Frontier temporarily "
                      "blacklisted at [" << output.selected_x << ", "
                      << output.selected_y << "].");
      break;
    case ExplorerStatus::kActive:
      ROS_INFO_STREAM_THROTTLE(
          1.0, "Reachable frontier target=[" << output.selected_x << ", "
          << output.selected_y << "], path_distance="
          << output.selected_path_distance
          << (output.returning_to_saved_branch
                  ? ", returning_to_saved_branch"
                  : "")
          << ", deferred_branches=" << output.deferred_branch_count
          << ", tracking waypoint=[" << output.goal.position.x << ", "
          << output.goal.position.y << "]");
      break;
    case ExplorerStatus::kFinished:
      ROS_INFO_THROTTLE(5.0,
                        "No reachable frontier. Exploration finished.");
      break;
    case ExplorerStatus::kNoFrontier:
      break;
  }
}

void TerrainWaypointExplorerRos1Convert::Run() {
  ::ros::spin();
}

}  // namespace ros1
}  // namespace terrain_waypoint_exploration
