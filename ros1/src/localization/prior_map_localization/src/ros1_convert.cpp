#include "prior_map_localization_ros1/ros1_convert.h"

#include <cmath>
#include <exception>
#include <utility>

#include <pcl_conversions/pcl_conversions.h>

namespace jojo {
namespace localization {
namespace ros1 {

PriorMapLocalizationRos1Convert::PriorMapLocalizationRos1Convert(
    ::ros::NodeHandle& node, ::ros::NodeHandle& private_node,
    std::string runtime_config, std::string map_path,
    std::string log_directory)
    : node_(node),
      private_node_(private_node),
      runtime_config_(std::move(runtime_config)),
      map_path_(std::move(map_path)),
      log_directory_(std::move(log_directory)) {}

bool PriorMapLocalizationRos1Convert::Init() {
  private_node_.param("topics/lidar_livox", lidar_livox_topic_,
                      lidar_livox_topic_);
  private_node_.param("topics/imu_livox", imu_livox_topic_,
                      imu_livox_topic_);
  private_node_.param("topics/lidar_standard", lidar_standard_topic_,
                      lidar_standard_topic_);
  private_node_.param("topics/imu_standard", imu_standard_topic_,
                      imu_standard_topic_);
  private_node_.param("topics/global_pose", global_pose_topic_,
                      global_pose_topic_);
  private_node_.param("topics/initial_pose", initial_pose_topic_,
                      initial_pose_topic_);
  private_node_.param("topics/map", map_topic_, map_topic_);
  private_node_.param("topics/path", path_topic_, path_topic_);
  private_node_.param("topics/lio_odom", lio_odometry_topic_,
                      lio_odometry_topic_);
  private_node_.param("topics/lio_path", lio_path_topic_,
                      lio_path_topic_);
  private_node_.param("topics/state_estimation", state_estimation_topic_,
                      state_estimation_topic_);
  private_node_.param("topics/registered_scan", registered_scan_topic_,
                      registered_scan_topic_);
  private_node_.param("topics/lidar_global_pose",
                      lidar_global_pose_topic_, lidar_global_pose_topic_);
  private_node_.param("frames/map", map_frame_, map_frame_);
  private_node_.param("frames/world", world_frame_, world_frame_);
  private_node_.param("frames/sensor", sensor_frame_, sensor_frame_);
  private_node_.param("queues/lidar", lidar_queue_size_, lidar_queue_size_);
  private_node_.param("queues/imu", imu_queue_size_, imu_queue_size_);
  private_node_.param("queues/initial_pose", initial_pose_queue_size_,
                      initial_pose_queue_size_);
  private_node_.param("queues/global_pose", global_pose_queue_size_,
                      global_pose_queue_size_);
  private_node_.param("queues/map", map_queue_size_, map_queue_size_);
  private_node_.param("queues/path", path_queue_size_, path_queue_size_);
  private_node_.param("queues/odometry", odometry_queue_size_,
                      odometry_queue_size_);
  private_node_.param("queues/lio_path", lio_path_queue_size_,
                      lio_path_queue_size_);
  private_node_.param("queues/registered_scan",
                      registered_scan_queue_size_,
                      registered_scan_queue_size_);
  private_node_.param("transport/map_latched", map_latched_, map_latched_);
  private_node_.param("transport/processing_frequency",
                      processing_frequency_, processing_frequency_);
  private_node_.param("transport/path_min_distance", path_min_distance_,
                      path_min_distance_);
  private_node_.param("conversion/standard_lidar_time_offset",
                      standard_lidar_time_offset_,
                      standard_lidar_time_offset_);
  private_node_.param("conversion/standard_imu_gyro_scale",
                      standard_imu_gyro_scale_,
                      standard_imu_gyro_scale_);
  private_node_.param("conversion/standard_imu_swap_xy",
                      standard_imu_swap_xy_, standard_imu_swap_xy_);
  private_node_.param("conversion/standard_imu_invert_y",
                      standard_imu_invert_y_, standard_imu_invert_y_);
  if (processing_frequency_ <= 0.0 || path_min_distance_ < 0.0 ||
      lidar_queue_size_ <= 0 || imu_queue_size_ <= 0 ||
      initial_pose_queue_size_ <= 0 || global_pose_queue_size_ <= 0 ||
      map_queue_size_ <= 0 || path_queue_size_ <= 0 ||
      odometry_queue_size_ <= 0 || lio_path_queue_size_ <= 0 ||
      registered_scan_queue_size_ <= 0) {
    ROS_ERROR("prior_map_localization interface rates/queues are invalid");
    return false;
  }

  try {
    const PriorMapLocalizationConfig config =
        PriorMapLocalizationConfig::LoadFromFile(runtime_config_);
    core_.reset(new PriorMapLocalization(
        config, map_path_, log_directory_));
  } catch (const std::exception& error) {
    ROS_ERROR_STREAM("prior_map_localization initialization failed: "
                     << error.what());
    return false;
  }

  if (core_->lidar_type() == AVIA) {
    lidar_subscriber_ = node_.subscribe<livox_ros_driver2::CustomMsg>(
        lidar_livox_topic_, lidar_queue_size_,
        &PriorMapLocalizationRos1Convert::LivoxCloudCallback, this);
    imu_subscriber_ = node_.subscribe<sensor_msgs::Imu>(
        imu_livox_topic_, imu_queue_size_,
        &PriorMapLocalizationRos1Convert::LivoxImuCallback, this);
  } else {
    lidar_subscriber_ = node_.subscribe<sensor_msgs::PointCloud2>(
        lidar_standard_topic_, lidar_queue_size_,
        &PriorMapLocalizationRos1Convert::StandardCloudCallback, this);
    imu_subscriber_ = node_.subscribe<sensor_msgs::Imu>(
        imu_standard_topic_, imu_queue_size_,
        &PriorMapLocalizationRos1Convert::StandardImuCallback, this);
  }
  initial_pose_subscriber_ =
      node_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
          initial_pose_topic_, initial_pose_queue_size_,
          &PriorMapLocalizationRos1Convert::InitialPoseCallback, this);
  global_pose_subscriber_ = node_.subscribe<self_state::GlobalPose>(
      global_pose_topic_, global_pose_queue_size_,
      &PriorMapLocalizationRos1Convert::GlobalPoseCallback, this);

  map_publisher_ = node_.advertise<sensor_msgs::PointCloud2>(
      map_topic_, map_queue_size_, map_latched_);
  path_publisher_ =
      node_.advertise<nav_msgs::Path>(path_topic_, path_queue_size_);
  lio_odometry_publisher_ = node_.advertise<nav_msgs::Odometry>(
      lio_odometry_topic_, odometry_queue_size_);
  lio_path_publisher_ =
      node_.advertise<nav_msgs::Path>(lio_path_topic_, lio_path_queue_size_);
  state_estimation_publisher_ = node_.advertise<nav_msgs::Odometry>(
      state_estimation_topic_, odometry_queue_size_);
  registered_scan_publisher_ = node_.advertise<sensor_msgs::PointCloud2>(
      registered_scan_topic_, registered_scan_queue_size_);
  lidar_global_pose_publisher_ =
      node_.advertise<self_state::LidarGlobalPose>(
          lidar_global_pose_topic_, odometry_queue_size_);

  path_.header.frame_id = map_frame_;
  lio_path_.header.frame_id = map_frame_;
  PublishMap();
  ROS_INFO_STREAM("prior_map_localization ROS1 adapter initialized: lidar="
                  << (core_->lidar_type() == AVIA
                          ? lidar_livox_topic_ : lidar_standard_topic_)
                  << ", imu="
                  << (core_->lidar_type() == AVIA
                          ? imu_livox_topic_ : imu_standard_topic_));
  return true;
}

void PriorMapLocalizationRos1Convert::StandardCloudCallback(
    const sensor_msgs::PointCloud2ConstPtr& message) {
  const double timestamp =
      message->header.stamp.toSec() + standard_lidar_time_offset_;
  if (core_->lidar_type() == RS128) {
    pcl::PointCloud<rs_lidar::Point> converted;
    pcl::fromROSMsg(*message, converted);
    core_->PushRsCloud(timestamp, converted);
  } else if (core_->lidar_type() == VELO16) {
    pcl::PointCloud<velodyne_lidar::Point> converted;
    pcl::fromROSMsg(*message, converted);
    core_->PushVelodyneCloud(timestamp, converted);
  } else {
    ROS_ERROR_THROTTLE(2.0, "standard cloud received for Livox mode");
  }
}

void PriorMapLocalizationRos1Convert::LivoxCloudCallback(
    const livox_ros_driver2::CustomMsgConstPtr& message) {
  LivoxPointCloud converted;
  converted.reserve(message->points.size());
  for (const auto& source : message->points) {
    LivoxPointData point;
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

ImuDataConstPtr PriorMapLocalizationRos1Convert::ConvertStandardImu(
    const sensor_msgs::Imu& message) const {
  std::shared_ptr<ImuData> converted(new ImuData());
  converted->timestamp = message.header.stamp.toSec();
  if (standard_imu_swap_xy_) {
    converted->linear_acceleration.x = message.linear_acceleration.y;
    converted->linear_acceleration.y = message.linear_acceleration.x;
    converted->angular_velocity.x = message.angular_velocity.y;
    converted->angular_velocity.y = message.angular_velocity.x;
  } else {
    converted->linear_acceleration.x = message.linear_acceleration.x;
    converted->linear_acceleration.y = message.linear_acceleration.y;
    converted->angular_velocity.x = message.angular_velocity.x;
    converted->angular_velocity.y = message.angular_velocity.y;
  }
  if (standard_imu_invert_y_) {
    converted->linear_acceleration.y =
        -converted->linear_acceleration.y;
    converted->angular_velocity.y = -converted->angular_velocity.y;
  }
  converted->linear_acceleration.z = message.linear_acceleration.z;
  converted->angular_velocity.x *= standard_imu_gyro_scale_;
  converted->angular_velocity.y *= standard_imu_gyro_scale_;
  converted->angular_velocity.z =
      message.angular_velocity.z * standard_imu_gyro_scale_;
  return converted;
}

ImuDataConstPtr PriorMapLocalizationRos1Convert::ConvertLivoxImu(
    const sensor_msgs::Imu& message) {
  std::shared_ptr<ImuData> converted(new ImuData());
  converted->timestamp = message.header.stamp.toSec();
  converted->linear_acceleration.x = message.linear_acceleration.x;
  converted->linear_acceleration.y = message.linear_acceleration.y;
  converted->linear_acceleration.z = message.linear_acceleration.z;
  converted->angular_velocity.x = message.angular_velocity.x;
  converted->angular_velocity.y = message.angular_velocity.y;
  converted->angular_velocity.z = message.angular_velocity.z;
  return converted;
}

void PriorMapLocalizationRos1Convert::StandardImuCallback(
    const sensor_msgs::ImuConstPtr& message) {
  core_->PushImu(ConvertStandardImu(*message));
}

void PriorMapLocalizationRos1Convert::LivoxImuCallback(
    const sensor_msgs::ImuConstPtr& message) {
  core_->PushImu(ConvertLivoxImu(*message));
}

void PriorMapLocalizationRos1Convert::InitialPoseCallback(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr& message) {
  ExternalPose converted;
  converted.timestamp = message->header.stamp.toSec();
  converted.state.pos = V3D(message->pose.pose.position.x,
                            message->pose.pose.position.y,
                            message->pose.pose.position.z);
  const auto& orientation = message->pose.pose.orientation;
  converted.state.rot = Eigen::Quaterniond(
      orientation.w, orientation.x, orientation.y, orientation.z);
  converted.quality_valid = true;
  core_->SetExternalPose(converted);
  ROS_INFO_STREAM("received external initial pose at "
                  << converted.state.pos.transpose());
}

void PriorMapLocalizationRos1Convert::GlobalPoseCallback(
    const self_state::GlobalPoseConstPtr& message) {
  ExternalPose converted;
  converted.timestamp = message->local_time / 1000.0;
  const std::vector<double>& center = core_->map_center();
  converted.state.pos = V3D(message->gaussX - center[0],
                            message->gaussY - center[1],
                            message->height - center[2]);
  converted.state.rot = EulerToSO3(
      message->roll * DEG_2_RAD,
      -message->pitch * DEG_2_RAD,
      message->azimuth * DEG_2_RAD);
  converted.quality_valid = message->ins_status.ins_status == 3 &&
      message->dev_gaussX <= 5.0 && message->dev_gaussY <= 5.0;
  core_->SetExternalPose(converted);
}

void PriorMapLocalizationRos1Convert::PublishMap() {
  sensor_msgs::PointCloud2 message;
  pcl::toROSMsg(*core_->map(), message);
  message.header.stamp = ::ros::Time::now();
  message.header.frame_id = map_frame_;
  map_publisher_.publish(message);
}

nav_msgs::Odometry PriorMapLocalizationRos1Convert::MakeOdometry(
    const PriorMapLocalizationOutput& output, const std::string& frame_id,
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

void PriorMapLocalizationRos1Convert::PublishOutput(
    const PriorMapLocalizationOutput& output) {
  if (!output.pose_ready) {
    if (output.status == LocalizationStepStatus::kWaitingForInitialPose) {
      ROS_WARN_THROTTLE(2.0,
                        "prior_map_localization is waiting for initial pose");
    }
    return;
  }

  const ::ros::Time stamp = ::ros::Time().fromSec(output.timestamp);
  nav_msgs::Odometry lio_odometry =
      MakeOdometry(output, world_frame_, std::string());
  lio_odometry_publisher_.publish(lio_odometry);
  state_estimation_publisher_.publish(
      MakeOdometry(output, map_frame_, sensor_frame_));

  geometry_msgs::PoseStamped pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = map_frame_;
  pose.pose = lio_odometry.pose.pose;
  path_.header.stamp = stamp;
  path_.poses.push_back(pose);
  path_publisher_.publish(path_);
  if (!has_last_lio_path_position_ ||
      (output.pose.pos - last_lio_path_position_).norm() >
          path_min_distance_) {
    lio_path_.header.stamp = stamp;
    lio_path_.poses.push_back(pose);
    lio_path_publisher_.publish(lio_path_);
    last_lio_path_position_ = output.pose.pos;
    has_last_lio_path_position_ = true;
  }

  if (output.registered_scan) {
    sensor_msgs::PointCloud2 cloud;
    pcl::toROSMsg(*output.registered_scan, cloud);
    cloud.header.stamp = stamp;
    cloud.header.frame_id = map_frame_;
    registered_scan_publisher_.publish(cloud);
  }

  tf::Transform transform;
  transform.setOrigin(tf::Vector3(output.pose.pos.x(), output.pose.pos.y(),
                                  output.pose.pos.z()));
  transform.setRotation(tf::Quaternion(
      output.pose.rot.x(), output.pose.rot.y(), output.pose.rot.z(),
      output.pose.rot.w()));
  transform_broadcaster_.sendTransform(
      tf::StampedTransform(transform, stamp, map_frame_, sensor_frame_));

  self_state::LidarGlobalPose global_pose;
  global_pose.local_time = output.timestamp * 1000.0;
  global_pose.pos_type = 2;
  const std::vector<double>& center = core_->map_center();
  global_pose.x = output.pose.pos.x() + center[0];
  global_pose.y = output.pose.pos.y() + center[1];
  global_pose.z = output.pose.pos.z() + center[2];
  global_pose.x_speed = output.velocity.x();
  global_pose.y_speed = output.velocity.y();
  global_pose.z_speed = output.velocity.z();
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf::Matrix3x3(tf::Quaternion(
      output.pose.rot.x(), output.pose.rot.y(), output.pose.rot.z(),
      output.pose.rot.w())).getRPY(roll, pitch, yaw);
  global_pose.roll = roll * RAD_2_DEG;
  global_pose.pitch = pitch * RAD_2_DEG;
  global_pose.azimuth = yaw * RAD_2_DEG;
  lidar_global_pose_publisher_.publish(global_pose);
}

void PriorMapLocalizationRos1Convert::Run() {
  ::ros::Rate rate(processing_frequency_);
  while (::ros::ok()) {
    ::ros::spinOnce();
    PublishOutput(core_->Step());
    rate.sleep();
  }
}

}  // namespace ros1
}  // namespace localization
}  // namespace jojo
