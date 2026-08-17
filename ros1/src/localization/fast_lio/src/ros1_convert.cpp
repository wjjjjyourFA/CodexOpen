#include "robot_dog_fast_lio_ros1/ros1_convert.h"

#include <algorithm>
#include <exception>
#include <vector>

#include <pcl_conversions/pcl_conversions.h>

namespace jojo {
namespace localization {
namespace ros1 {

Ros1Convert::Ros1Convert(::ros::NodeHandle& node,
                         ::ros::NodeHandle& private_node)
    : node_(node), private_node_(private_node) {}

bool Ros1Convert::Init() {
  FastLioConfig config;
  private_node_.param("preprocess/lidar_type", config.lidar_type,
                      config.lidar_type);
  private_node_.param("common/time_offset_lidar_to_imu",
                      config.lidar_to_imu_time_offset,
                      config.lidar_to_imu_time_offset);
  private_node_.param("common/point_filter_num", config.point_filter_num,
                      config.point_filter_num);
  private_node_.param("preprocess/scan_line", config.scan_line_count,
                      config.scan_line_count);
  private_node_.param("preprocess/blind", config.blind_distance,
                      config.blind_distance);
  private_node_.param("preprocess/x_min", config.inner_x_min,
                      config.inner_x_min);
  private_node_.param("preprocess/x_max", config.inner_x_max,
                      config.inner_x_max);
  private_node_.param("preprocess/y_min", config.inner_y_min,
                      config.inner_y_min);
  private_node_.param("preprocess/y_max", config.inner_y_max,
                      config.inner_y_max);
  private_node_.param("preprocess/z_min", config.inner_z_min,
                      config.inner_z_min);
  private_node_.param("preprocess/z_max", config.inner_z_max,
                      config.inner_z_max);
  private_node_.param("preprocess/maxrange", config.maximum_range,
                      config.maximum_range);
  private_node_.param("mapping/max_iteration", config.maximum_iterations,
                      config.maximum_iterations);
  private_node_.param("mapping/filter_size_surf",
                      config.surface_filter_size,
                      config.surface_filter_size);
  private_node_.param("mapping/filter_size_map", config.map_filter_size,
                      config.map_filter_size);
  private_node_.param("mapping/cube_side_length",
                      config.cube_side_length, config.cube_side_length);
  private_node_.param("mapping/det_range", config.detection_range,
                      config.detection_range);
  private_node_.param("mapping/gyr_cov", config.gyroscope_covariance,
                      config.gyroscope_covariance);
  private_node_.param("mapping/acc_cov", config.acceleration_covariance,
                      config.acceleration_covariance);
  private_node_.param("mapping/b_gyr_cov",
                      config.gyroscope_bias_covariance,
                      config.gyroscope_bias_covariance);
  private_node_.param("mapping/b_acc_cov",
                      config.acceleration_bias_covariance,
                      config.acceleration_bias_covariance);
  private_node_.param("mapping/extrinsic_est_en",
                      config.estimate_extrinsic,
                      config.estimate_extrinsic);

  std::vector<double> translation(config.extrinsic_translation.begin(),
                                  config.extrinsic_translation.end());
  std::vector<double> rotation(config.extrinsic_rotation.begin(),
                               config.extrinsic_rotation.end());
  private_node_.param("mapping/extrinsic_T", translation, translation);
  private_node_.param("mapping/extrinsic_R", rotation, rotation);
  if (translation.size() != config.extrinsic_translation.size() ||
      rotation.size() != config.extrinsic_rotation.size()) {
    ROS_ERROR("Fast-LIO extrinsic_T must have 3 values and extrinsic_R 9");
    return false;
  }
  std::copy(translation.begin(), translation.end(),
            config.extrinsic_translation.begin());
  std::copy(rotation.begin(), rotation.end(),
            config.extrinsic_rotation.begin());

  private_node_.param("common/lid_topic", lidar_topic_, lidar_topic_);
  private_node_.param("common/imu_topic", imu_topic_, imu_topic_);
  private_node_.param("topics/odometry", odometry_topic_,
                      odometry_topic_);
  private_node_.param("topics/state_estimation", state_estimation_topic_,
                      state_estimation_topic_);
  private_node_.param("topics/path", path_topic_, path_topic_);
  private_node_.param("topics/registered_scan", registered_scan_topic_,
                      registered_scan_topic_);
  private_node_.param("publish/path_en", path_enabled_, path_enabled_);
  private_node_.param("publish/scan_publish_en", scan_publish_enabled_,
                      scan_publish_enabled_);
  private_node_.param("publish/dense_publish_en", dense_publish_enabled_,
                      dense_publish_enabled_);
  private_node_.param("publish/world_frame", world_frame_, world_frame_);
  private_node_.param("publish/odometry_frame", odometry_frame_,
                      odometry_frame_);
  private_node_.param("publish/sensor_frame", sensor_frame_, sensor_frame_);
  private_node_.param("io/output_root", output_root_, output_root_);
  private_node_.param("transport/processing_rate", processing_rate_,
                      processing_rate_);
  private_node_.param("transport/path_min_distance", path_min_distance_,
                      path_min_distance_);
  private_node_.param("queues/lidar", lidar_queue_size_, lidar_queue_size_);
  private_node_.param("queues/imu", imu_queue_size_, imu_queue_size_);
  private_node_.param("queues/output", output_queue_size_,
                      output_queue_size_);
  if (processing_rate_ <= 0.0 || path_min_distance_ < 0.0 ||
      lidar_queue_size_ <= 0 ||
      imu_queue_size_ <= 0 || output_queue_size_ <= 0) {
    ROS_ERROR("Fast-LIO interface rates/queues are invalid");
    return false;
  }

  try {
    core_.reset(new FastLioPipeline(config, output_root_));
  } catch (const std::exception& error) {
    ROS_ERROR_STREAM("Fast-LIO core initialization failed: "
                     << error.what());
    return false;
  }

  lidar_subscriber_ = node_.subscribe<LivoxMessage>(
      lidar_topic_, lidar_queue_size_, &Ros1Convert::LidarCallback, this);
  imu_subscriber_ = node_.subscribe<sensor_msgs::Imu>(
      imu_topic_, imu_queue_size_, &Ros1Convert::ImuCallback, this);
  odometry_publisher_ = node_.advertise<nav_msgs::Odometry>(
      odometry_topic_, output_queue_size_);
  state_estimation_publisher_ = node_.advertise<nav_msgs::Odometry>(
      state_estimation_topic_, output_queue_size_);
  path_publisher_ = node_.advertise<nav_msgs::Path>(
      path_topic_, output_queue_size_);
  registered_scan_publisher_ = node_.advertise<sensor_msgs::PointCloud2>(
      registered_scan_topic_, output_queue_size_);
  path_message_.header.frame_id = world_frame_;
  ROS_INFO_STREAM("robot_dog_fast_lio ROS1 adapter ready: lidar="
                  << lidar_topic_ << ", imu=" << imu_topic_);
  return true;
}

void Ros1Convert::LidarCallback(
    const typename LivoxMessage::ConstPtr& message) {
  FastLioLivoxCloud converted;
  converted.reserve(message->points.size());
  for (const auto& source : message->points) {
    FastLioLivoxPoint point;
    point.offset_time = source.offset_time;
    point.x = source.x;
    point.y = source.y;
    point.z = source.z;
    point.reflectivity = source.reflectivity;
    point.tag = source.tag;
    point.line = source.line;
    converted.push_back(point);
  }
  core_->PushLivoxCloud(message->header.stamp.toSec(), converted);
}

void Ros1Convert::ImuCallback(const sensor_msgs::ImuConstPtr& message) {
  fastlio::ImuData converted;
  converted.timestamp = message->header.stamp.toSec();
  converted.linear_acceleration = Eigen::Vector3d(
      message->linear_acceleration.x, message->linear_acceleration.y,
      message->linear_acceleration.z);
  converted.angular_velocity = Eigen::Vector3d(
      message->angular_velocity.x, message->angular_velocity.y,
      message->angular_velocity.z);
  core_->PushImu(converted);
}

nav_msgs::Odometry Ros1Convert::MakeOdometry(
    const FastLioOutput& output, const std::string& frame_id,
    const std::string& child_frame_id) const {
  nav_msgs::Odometry message;
  message.header.stamp.fromSec(output.timestamp);
  message.header.frame_id = frame_id;
  message.child_frame_id = child_frame_id;
  message.pose.pose.position.x = output.pose.pos.x();
  message.pose.pose.position.y = output.pose.pos.y();
  message.pose.pose.position.z = output.pose.pos.z();
  message.pose.pose.orientation.x = output.pose.rot.x();
  message.pose.pose.orientation.y = output.pose.rot.y();
  message.pose.pose.orientation.z = output.pose.rot.z();
  message.pose.pose.orientation.w = output.pose.rot.w();
  message.twist.twist.linear.x = output.velocity.x();
  message.twist.twist.linear.y = output.velocity.y();
  message.twist.twist.linear.z = output.velocity.z();
  std::copy(output.pose_covariance.begin(), output.pose_covariance.end(),
            message.pose.covariance.begin());
  return message;
}

void Ros1Convert::PublishOutput(const FastLioOutput& output) {
  if (!output.pose_ready) {
    return;
  }
  const ::ros::Time stamp = ::ros::Time().fromSec(output.timestamp);
  odometry_publisher_.publish(
      MakeOdometry(output, odometry_frame_, std::string()));
  state_estimation_publisher_.publish(
      MakeOdometry(output, world_frame_, sensor_frame_));

  tf::Transform transform;
  transform.setOrigin(tf::Vector3(output.pose.pos.x(), output.pose.pos.y(),
                                  output.pose.pos.z()));
  transform.setRotation(tf::Quaternion(
      output.pose.rot.x(), output.pose.rot.y(), output.pose.rot.z(),
      output.pose.rot.w()));
  transform_broadcaster_.sendTransform(
      tf::StampedTransform(transform, stamp, world_frame_, sensor_frame_));

  if (path_enabled_ &&
      (output.pose.pos - last_path_position_).norm() > path_min_distance_) {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = world_frame_;
    pose.pose.position.x = output.pose.pos.x();
    pose.pose.position.y = output.pose.pos.y();
    pose.pose.position.z = output.pose.pos.z();
    pose.pose.orientation.x = output.pose.rot.x();
    pose.pose.orientation.y = output.pose.rot.y();
    pose.pose.orientation.z = output.pose.rot.z();
    pose.pose.orientation.w = output.pose.rot.w();
    path_message_.header.stamp = stamp;
    path_message_.poses.push_back(pose);
    path_publisher_.publish(path_message_);
    last_path_position_ = output.pose.pos;
  }

  if (scan_publish_enabled_ && output.registered_scan) {
    sensor_msgs::PointCloud2 message;
    pcl::toROSMsg(*output.registered_scan, message);
    message.header.stamp = stamp;
    message.header.frame_id = world_frame_;
    registered_scan_publisher_.publish(message);
  }
}

void Ros1Convert::Run() {
  ::ros::Rate rate(processing_rate_);
  while (::ros::ok()) {
    ::ros::spinOnce();
    PublishOutput(core_->Step(scan_publish_enabled_,
                              dense_publish_enabled_));
    rate.sleep();
  }
}

}  // namespace ros1
}  // namespace localization
}  // namespace jojo
