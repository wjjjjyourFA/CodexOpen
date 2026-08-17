#include "terrain_analysis_ros1/ros1_convert.h"

#include <pcl_conversions/pcl_conversions.h>

namespace jojo {
namespace perception {
namespace ros1 {

TerrainAnalysisRos1Convert::TerrainAnalysisRos1Convert(
    ::ros::NodeHandle& node, ::ros::NodeHandle& private_node)
    : node_(node), private_node_(private_node) {}

bool TerrainAnalysisRos1Convert::Init() {
  TerrainAnalysisConfig config;
  private_node_.param("scanVoxelSize", config.scan_voxel_size,
                      config.scan_voxel_size);
  private_node_.param("decayTime", config.decay_time, config.decay_time);
  private_node_.param("noDecayDis", config.no_decay_distance,
                      config.no_decay_distance);
  private_node_.param("clearingDis", config.clearing_distance,
                      config.clearing_distance);
  private_node_.param("useSorting", config.use_sorting, config.use_sorting);
  private_node_.param("quantileZ", config.quantile_z, config.quantile_z);
  private_node_.param("considerDrop", config.consider_drop,
                      config.consider_drop);
  private_node_.param("limitGroundLift", config.limit_ground_lift,
                      config.limit_ground_lift);
  private_node_.param("maxGroundLift", config.max_ground_lift,
                      config.max_ground_lift);
  private_node_.param("clearDyObs", config.clear_dynamic_obstacles,
                      config.clear_dynamic_obstacles);
  private_node_.param("minDyObsDis", config.min_dynamic_obstacle_distance,
                      config.min_dynamic_obstacle_distance);
  private_node_.param("minDyObsAngle", config.min_dynamic_obstacle_angle,
                      config.min_dynamic_obstacle_angle);
  private_node_.param("minDyObsRelZ",
                      config.min_dynamic_obstacle_relative_z,
                      config.min_dynamic_obstacle_relative_z);
  private_node_.param(
      "absDyObsRelZThre",
      config.absolute_dynamic_obstacle_relative_z_threshold,
      config.absolute_dynamic_obstacle_relative_z_threshold);
  private_node_.param("minDyObsVFOV",
                      config.min_dynamic_obstacle_vertical_fov,
                      config.min_dynamic_obstacle_vertical_fov);
  private_node_.param("maxDyObsVFOV",
                      config.max_dynamic_obstacle_vertical_fov,
                      config.max_dynamic_obstacle_vertical_fov);
  private_node_.param("minDyObsPointNum",
                      config.min_dynamic_obstacle_point_count,
                      config.min_dynamic_obstacle_point_count);
  private_node_.param("noDataObstacle", config.no_data_obstacle,
                      config.no_data_obstacle);
  private_node_.param("noDataBlockSkipNum",
                      config.no_data_block_skip_count,
                      config.no_data_block_skip_count);
  private_node_.param("minBlockPointNum", config.min_block_point_count,
                      config.min_block_point_count);
  private_node_.param("vehicleHeight", config.vehicle_height,
                      config.vehicle_height);
  private_node_.param("voxelPointUpdateThre",
                      config.voxel_point_update_threshold,
                      config.voxel_point_update_threshold);
  private_node_.param("voxelTimeUpdateThre",
                      config.voxel_time_update_threshold,
                      config.voxel_time_update_threshold);
  private_node_.param("minRelZ", config.min_relative_z,
                      config.min_relative_z);
  private_node_.param("maxRelZ", config.max_relative_z,
                      config.max_relative_z);
  private_node_.param("disRatioZ", config.distance_ratio_z,
                      config.distance_ratio_z);
  private_node_.param("useAccumulation", config.use_accumulation,
                      config.use_accumulation);

  private_node_.param("inputTopic", input_topic_, input_topic_);
  private_node_.param("odometryTopic", odometry_topic_, odometry_topic_);
  private_node_.param("clearingTopic", clearing_topic_, clearing_topic_);
  private_node_.param("terrainMapTopic", terrain_map_topic_,
                      terrain_map_topic_);
  private_node_.param("worldFrame", world_frame_, world_frame_);
  private_node_.param("transport/processing_frequency",
                      processing_frequency_, processing_frequency_);
  private_node_.param("queues/odometry", odometry_queue_size_,
                      odometry_queue_size_);
  private_node_.param("queues/scan", scan_queue_size_, scan_queue_size_);
  private_node_.param("queues/clearing", clearing_queue_size_,
                      clearing_queue_size_);
  private_node_.param("queues/output", output_queue_size_,
                      output_queue_size_);
  if (processing_frequency_ <= 0.0 || odometry_queue_size_ <= 0 ||
      scan_queue_size_ <= 0 || clearing_queue_size_ <= 0 ||
      output_queue_size_ <= 0) {
    ROS_ERROR("terrain_analysis interface rates/queues must be positive");
    return false;
  }

  core_.reset(new TerrainAnalysis(config));
  odometry_subscriber_ = node_.subscribe<nav_msgs::Odometry>(
      odometry_topic_, odometry_queue_size_,
      &TerrainAnalysisRos1Convert::OdometryCallback, this);
  scan_subscriber_ = node_.subscribe<sensor_msgs::PointCloud2>(
      input_topic_, scan_queue_size_,
      &TerrainAnalysisRos1Convert::ScanCallback, this);
  clearing_subscriber_ = node_.subscribe<std_msgs::Float32>(
      clearing_topic_, clearing_queue_size_,
      &TerrainAnalysisRos1Convert::ClearingCallback, this);
  terrain_publisher_ =
      node_.advertise<sensor_msgs::PointCloud2>(terrain_map_topic_,
                                                output_queue_size_);
  return true;
}

common_struct::Pose TerrainAnalysisRos1Convert::FromRosPose(
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

void TerrainAnalysisRos1Convert::OdometryCallback(
    const nav_msgs::OdometryConstPtr& message) {
  core_->SetOdometry(FromRosPose(message->pose.pose));
}

void TerrainAnalysisRos1Convert::ScanCallback(
    const sensor_msgs::PointCloud2ConstPtr& message) {
  pending_scan_.clear();
  pcl::fromROSMsg(*message, pending_scan_);
  pending_scan_time_ = message->header.stamp;
  has_pending_scan_ = true;
}

void TerrainAnalysisRos1Convert::ClearingCallback(
    const std_msgs::Float32ConstPtr& message) {
  core_->RequestClearing(message->data);
}

void TerrainAnalysisRos1Convert::Run() {
  ::ros::Rate rate(processing_frequency_);
  while (::ros::ok()) {
    ::ros::spinOnce();
    if (has_pending_scan_) {
      has_pending_scan_ = false;
      const TerrainAnalysisOutput output =
          core_->Process(pending_scan_time_.toSec(), pending_scan_);
      if (output.ready) {
        sensor_msgs::PointCloud2 message;
        pcl::toROSMsg(output.terrain, message);
        message.header.stamp = pending_scan_time_;
        message.header.frame_id = world_frame_;
        terrain_publisher_.publish(message);
      }
    }
    rate.sleep();
  }
}

}  // namespace ros1
}  // namespace perception
}  // namespace jojo
