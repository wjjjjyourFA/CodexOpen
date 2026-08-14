#include "manual_loop_session_exporter_ros1/ros1_convert.h"

#include <sstream>
#include <stdexcept>

#include <boost/bind.hpp>
#include <pcl_conversions/pcl_conversions.h>

namespace jojo {
namespace tools {
namespace manual_loop {
namespace ros1 {

Ros1Convert::Ros1Convert(ros::NodeHandle& node, ros::NodeHandle& private_node)
    : node_(node), private_node_(private_node) {}

bool Ros1Convert::Init() {
  SessionExporterConfig config;
  private_node_.param<std::string>("odom_topic", odometry_topic_, odometry_topic_);
  private_node_.param<std::string>("cloud_topic", cloud_topic_, cloud_topic_);
  private_node_.param<std::string>("output_directory", config.output_directory,
                                   config.output_directory);
  private_node_.param("keyframe_distance", config.keyframe_distance, 1.5);
  private_node_.param("keyframe_angle_deg", config.keyframe_angle_deg, 0.0);
  private_node_.param("min_keyframe_time", config.min_keyframe_time, 0.0);
  private_node_.param("sync_queue_size", sync_queue_size_, 100);
  private_node_.param("sync_slop", sync_slop_, 0.05);
  private_node_.param("input_cloud_is_global", config.input_cloud_is_global, true);
  private_node_.param("voxel_leaf_size", config.voxel_leaf_size, 0.0);
  private_node_.param("translation_information",
                      config.translation_information, 1000.0);
  private_node_.param("rotation_information", config.rotation_information, 1000.0);
  private_node_.param("overwrite_existing", config.overwrite_existing, false);

  if (sync_queue_size_ <= 0 || sync_slop_ <= 0.0) {
    ROS_ERROR("sync_queue_size must be positive and sync_slop must be greater than zero");
    return false;
  }

  try {
    exporter_.reset(new SessionExporter(config));
  } catch (const std::exception& error) {
    ROS_ERROR_STREAM("Cannot initialize session exporter: " << error.what());
    return false;
  }

  odometry_subscriber_.subscribe(node_, odometry_topic_, sync_queue_size_);
  cloud_subscriber_.subscribe(node_, cloud_topic_, sync_queue_size_);
  synchronizer_.reset(new Synchronizer(
      SyncPolicy(sync_queue_size_), odometry_subscriber_, cloud_subscriber_));
  synchronizer_->setMaxIntervalDuration(ros::Duration(sync_slop_));
  synchronizer_->registerCallback(boost::bind(
      &Ros1Convert::SynchronizedCallback, this, _1, _2));
  finalize_service_ = private_node_.advertiseService(
      "finalize", &Ros1Convert::FinalizeService, this);

  ROS_INFO_STREAM("Manual-loop exporter ready: odom=" << odometry_topic_
                  << ", cloud=" << cloud_topic_
                  << ", output=" << exporter_->OutputDirectory());
  return true;
}

Eigen::Isometry3d Ros1Convert::PoseToIsometry(
    const geometry_msgs::Pose& pose) {
  Eigen::Quaterniond quaternion(pose.orientation.w, pose.orientation.x,
                                pose.orientation.y, pose.orientation.z);
  if (quaternion.norm() < 1e-9) {
    throw std::runtime_error("Received zero-norm odometry quaternion");
  }
  quaternion.normalize();
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = quaternion.toRotationMatrix();
  transform.translation() = Eigen::Vector3d(
      pose.position.x, pose.position.y, pose.position.z);
  return transform;
}

void Ros1Convert::SynchronizedCallback(
    const nav_msgs::OdometryConstPtr& odometry,
    const sensor_msgs::PointCloud2ConstPtr& cloud_message) {
  try {
    const Eigen::Isometry3d pose_world_sensor =
        PoseToIsometry(odometry->pose.pose);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*cloud_message, *cloud);
    const AddFrameResult result = exporter_->AddFrame(
        odometry->header.stamp.toSec(), pose_world_sensor, cloud);
    if (result.saved) {
      ROS_INFO_STREAM("Saved keyframe " << result.index
                      << ": points=" << result.point_count
                      << ", position=["
                      << pose_world_sensor.translation().transpose() << "]");
    }
  } catch (const std::exception& error) {
    ROS_ERROR_STREAM_THROTTLE(1.0, "Exporter callback failed: " << error.what());
  }
}

bool Ros1Convert::FinalizeService(std_srvs::Trigger::Request&,
                                  std_srvs::Trigger::Response& response) {
  try {
    exporter_->Finalize();
    response.success = exporter_->KeyframeCount() > 0;
    std::ostringstream message;
    message << "Session contains " << exporter_->KeyframeCount()
            << " keyframes at " << exporter_->OutputDirectory();
    response.message = message.str();
  } catch (const std::exception& error) {
    response.success = false;
    response.message = error.what();
  }
  return true;
}

}  // namespace ros1
}  // namespace manual_loop
}  // namespace tools
}  // namespace jojo
