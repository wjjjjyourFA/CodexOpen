#include "robot_dog_fast_lio_ros1/ros1_convert.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <stdexcept>

#include <pcl_conversions/pcl_conversions.h>

namespace jojo {
namespace localization {
namespace ros1 {

Ros1Convert::Ros1Convert(ros::NodeHandle& node, ros::NodeHandle& private_node)
    : node_(node), private_node_(private_node) {}

Ros1Convert::~Ros1Convert() {
  if (odometry_) {
    odometry_->Close();
  }
}

bool Ros1Convert::Init() {
  private_node_.param<std::string>("common/lid_topic", lidar_topic_, lidar_topic_);
  private_node_.param<std::string>("common/imu_topic", imu_topic_, imu_topic_);
  private_node_.param<std::string>("topics/odometry", odometry_topic_,
                                   odometry_topic_);
  private_node_.param<std::string>("topics/state_estimation",
                                   state_estimation_topic_,
                                   state_estimation_topic_);
  private_node_.param<std::string>("topics/path", path_topic_, path_topic_);
  private_node_.param<std::string>("topics/registered_scan",
                                   registered_scan_topic_,
                                   registered_scan_topic_);
  private_node_.param("common/time_sync_en", time_sync_enabled_, false);
  private_node_.param("common/time_offset_lidar_to_imu",
                      lidar_to_imu_time_offset_, 0.0);
  private_node_.param("common/point_filter_num", point_filter_num_, 3);
  private_node_.param("common/processing_rate", processing_rate_, 20.0);

  private_node_.param("preprocess/scan_line", scan_line_count_, 4);
  private_node_.param("preprocess/blind", blind_distance_, 0.5);
  private_node_.param("preprocess/x_min", inner_x_min_, -0.7);
  private_node_.param("preprocess/x_max", inner_x_max_, 0.7);
  private_node_.param("preprocess/y_min", inner_y_min_, -0.4);
  private_node_.param("preprocess/y_max", inner_y_max_, 0.4);
  private_node_.param("preprocess/z_min", inner_z_min_, -0.6);
  private_node_.param("preprocess/z_max", inner_z_max_, 0.5);
  private_node_.param("preprocess/maxrange", max_range_, 15.0);

  private_node_.param("publish/path_en", path_enabled_, true);
  private_node_.param("publish/scan_publish_en", scan_publish_enabled_, true);
  private_node_.param("publish/dense_publish_en", dense_publish_enabled_, true);
  private_node_.param<std::string>("publish/world_frame", world_frame_, "map");
  private_node_.param<std::string>("publish/sensor_frame", sensor_frame_, "sensor");
  private_node_.param<std::string>("io/output_root", output_root_, output_root_);

  if (point_filter_num_ <= 0 || scan_line_count_ <= 0 || processing_rate_ <= 0.0 ||
      max_range_ <= 0.0 || inner_x_min_ >= inner_x_max_ ||
      inner_y_min_ >= inner_y_max_ || inner_z_min_ >= inner_z_max_) {
    ROS_ERROR("Invalid robot_dog_fast_lio_ros1 configuration");
    return false;
  }

  static_config_ = std::make_shared<jojo::localization::StaticConfig>();
  static_config_->lidar_type = AVIA;
  static_config_->point_filter_num = point_filter_num_;
  static_config_->feature_enabled = false;
  static_config_->blind = blind_distance_;
  static_config_->N_SCANS = scan_line_count_;
  static_config_->time_unit = NS;
  static_config_->SCAN_RATE = 10;
  private_node_.param("mapping/max_iteration", static_config_->NUM_MAX_ITERATIONS, 3);
  private_node_.param("mapping/filter_size_surf",
                      static_config_->filter_size_surf_min, 0.2);
  private_node_.param("mapping/filter_size_map",
                      static_config_->filter_size_map_min, 0.2);
  private_node_.param("mapping/cube_side_length", static_config_->cube_len, 1000.0);
  private_node_.param("mapping/det_range", static_config_->DET_RANGE, 100.0f);
  private_node_.param("mapping/gyr_cov", static_config_->gyr_cov, 0.1);
  private_node_.param("mapping/acc_cov", static_config_->acc_cov, 0.1);
  private_node_.param("mapping/b_gyr_cov", static_config_->b_gyr_cov, 0.0001);
  private_node_.param("mapping/b_acc_cov", static_config_->b_acc_cov, 0.0001);
  private_node_.param("mapping/extrinsic_est_en",
                      static_config_->extrinsic_est_en, false);

  std::vector<double> extrinsic_translation{-0.011, -0.02329, 0.04412};
  std::vector<double> extrinsic_rotation{1.0, 0.0, 0.0,
                                         0.0, 1.0, 0.0,
                                         0.0, 0.0, 1.0};
  private_node_.param("mapping/extrinsic_T", extrinsic_translation,
                      extrinsic_translation);
  private_node_.param("mapping/extrinsic_R", extrinsic_rotation,
                      extrinsic_rotation);
  if (extrinsic_translation.size() != 3 || extrinsic_rotation.size() != 9) {
    ROS_ERROR("mapping/extrinsic_T must contain 3 values and extrinsic_R 9 values");
    return false;
  }

  Eigen::Matrix4f lidar_to_imu = Eigen::Matrix4f::Identity();
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      lidar_to_imu(row, column) =
          static_cast<float>(extrinsic_rotation[row * 3 + column]);
    }
    lidar_to_imu(row, 3) = static_cast<float>(extrinsic_translation[row]);
  }

  runtime_config_ = std::make_shared<jojo::localization::RuntimeConfig>();
  runtime_config_->root_path = output_root_;
  runtime_config_->file_name = "online";
  runtime_config_->b_save_pcd = false;
  runtime_config_->b_only_times = false;

  std::error_code directory_error;
  std::filesystem::create_directories(
      std::filesystem::path(output_root_) / "online-O", directory_error);
  if (directory_error) {
    ROS_ERROR_STREAM("Cannot prepare Fast-LIO output directory "
                     << output_root_ << ": " << directory_error.message());
    return false;
  }

  odometry_ = std::make_shared<RobotDogLidarOdometry>();
  odometry_->SetGravityImuExtrinsicMatrix(Eigen::Matrix4f::Identity());
  odometry_->SetExtrinsicMatrix(lidar_to_imu);
  odometry_->Init(runtime_config_, static_config_);

  lidar_subscriber_ = node_.subscribe<LivoxMessage>(
      lidar_topic_, 1, &Ros1Convert::LidarCallback, this);
  imu_subscriber_ = node_.subscribe<sensor_msgs::Imu>(
      imu_topic_, 200000, &Ros1Convert::ImuCallback, this);

  odometry_publisher_ =
      node_.advertise<nav_msgs::Odometry>(odometry_topic_, 100);
  state_estimation_publisher_ =
      node_.advertise<nav_msgs::Odometry>(state_estimation_topic_, 100);
  path_publisher_ = node_.advertise<nav_msgs::Path>(path_topic_, 1);
  registered_scan_publisher_ =
      node_.advertise<sensor_msgs::PointCloud2>(registered_scan_topic_, 10);

  path_message_.header.frame_id = world_frame_;
  ROS_INFO_STREAM("robot_dog_fast_lio_ros1 ready: lidar=" << lidar_topic_
                  << ", imu=" << imu_topic_);
  return true;
}

bool Ros1Convert::IsValidPoint(double x, double y, double z) const {
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
    return false;
  }
  if (x * x + y * y + z * z >= max_range_ * max_range_) {
    return false;
  }
  const bool inside_robot =
      x > inner_x_min_ && x < inner_x_max_ && y > inner_y_min_ &&
      y < inner_y_max_ && z > inner_z_min_ && z < inner_z_max_;
  return !inside_robot;
}

fastlio::PointCloudXYZI::Ptr Ros1Convert::ConvertLivoxMessage(
    const typename LivoxMessage::ConstPtr& message) const {
  fastlio::PointCloudXYZI::Ptr cloud(new fastlio::PointCloudXYZI());
  cloud->reserve(message->points.size() / std::max(1, point_filter_num_));

  std::size_t valid_count = 0;
  for (std::size_t index = 1; index < message->points.size(); ++index) {
    const auto& source = message->points[index];
    const auto& previous = message->points[index - 1];
    if (source.line >= scan_line_count_ ||
        ((source.tag & 0x30) != 0x10 && (source.tag & 0x30) != 0x00)) {
      continue;
    }
    ++valid_count;
    if (valid_count % static_cast<std::size_t>(point_filter_num_) != 0 ||
        !IsValidPoint(source.x, source.y, source.z)) {
      continue;
    }
    if (std::abs(source.x - previous.x) <= 1e-7 &&
        std::abs(source.y - previous.y) <= 1e-7 &&
        std::abs(source.z - previous.z) <= 1e-7) {
      continue;
    }

    fastlio::PointType point;
    point.x = source.x;
    point.y = source.y;
    point.z = source.z;
    point.intensity = source.reflectivity;
    point.normal_x = 0.0f;
    point.normal_y = 0.0f;
    point.normal_z = 0.0f;
    // FAST-LIO stores the per-point offset in milliseconds in curvature.
    point.curvature = source.offset_time / 1000000.0f;
    cloud->push_back(point);
  }
  cloud->width = cloud->size();
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

void Ros1Convert::LidarCallback(const typename LivoxMessage::ConstPtr& message) {
  const double timestamp = message->header.stamp.toSec();
  fastlio::PointCloudXYZI::Ptr cloud = ConvertLivoxMessage(message);

  std::lock_guard<std::mutex> lock(buffer_mutex_);
  if (timestamp < last_lidar_timestamp_) {
    ROS_ERROR("LiDAR timestamp looped backwards; clearing LiDAR buffer");
    lidar_buffer_.clear();
    lidar_time_buffer_.clear();
    lidar_pending_ = false;
  }
  last_lidar_timestamp_ = timestamp;
  lidar_buffer_.push_back(cloud);
  lidar_time_buffer_.push_back(timestamp);
}

void Ros1Convert::ImuCallback(const sensor_msgs::ImuConstPtr& message) {
  fastlio::ImuData imu;
  imu.timestamp = message->header.stamp.toSec() - lidar_to_imu_time_offset_;
  imu.linear_acceleration = Eigen::Vector3d(
      message->linear_acceleration.x, message->linear_acceleration.y,
      message->linear_acceleration.z);
  imu.angular_velocity = Eigen::Vector3d(
      message->angular_velocity.x, message->angular_velocity.y,
      message->angular_velocity.z);

  std::lock_guard<std::mutex> lock(buffer_mutex_);
  if (imu.timestamp < last_imu_timestamp_) {
    ROS_WARN("IMU timestamp looped backwards; clearing IMU buffer");
    imu_buffer_.clear();
  }
  last_imu_timestamp_ = imu.timestamp;
  imu_buffer_.push_back(imu);
}

bool Ros1Convert::BuildMeasureGroup(fastlio::MeasureGroup& group) {
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  if (lidar_buffer_.empty() || imu_buffer_.empty()) {
    return false;
  }

  if (!lidar_pending_) {
    pending_lidar_ = lidar_buffer_.front();
    pending_lidar_begin_time_ = lidar_time_buffer_.front();
    if (pending_lidar_->size() <= 1) {
      pending_lidar_end_time_ = pending_lidar_begin_time_ + lidar_mean_scan_time_;
    } else {
      const double measured_scan_time =
          pending_lidar_->points.back().curvature / 1000.0;
      if (measured_scan_time < 0.5 * lidar_mean_scan_time_) {
        pending_lidar_end_time_ = pending_lidar_begin_time_ + lidar_mean_scan_time_;
      } else {
        ++lidar_scan_count_;
        pending_lidar_end_time_ = pending_lidar_begin_time_ + measured_scan_time;
        lidar_mean_scan_time_ +=
            (measured_scan_time - lidar_mean_scan_time_) / lidar_scan_count_;
      }
    }
    lidar_pending_ = true;
  }

  if (last_imu_timestamp_ < pending_lidar_end_time_) {
    return false;
  }

  group.lidar = pending_lidar_;
  group.lidar_beg_time = pending_lidar_begin_time_;
  group.lidar_end_time = pending_lidar_end_time_;
  group.imu.clear();
  while (!imu_buffer_.empty() &&
         imu_buffer_.front().timestamp <= pending_lidar_end_time_) {
    group.imu.push_back(imu_buffer_.front());
    imu_buffer_.pop_front();
  }

  lidar_buffer_.pop_front();
  lidar_time_buffer_.pop_front();
  lidar_pending_ = false;
  return !group.imu.empty();
}

void Ros1Convert::PublishOdometry() {
  const auto& pose = odometry_->Pose();
  ros::Time stamp;
  stamp.fromSec(odometry_->LidarEndTime());
  const Eigen::Quaterniond quaternion = pose.rot.normalized();

  nav_msgs::Odometry odometry_message;
  odometry_message.header.stamp = stamp;
  odometry_message.header.frame_id = "world";
  odometry_message.pose.pose.position.x = pose.pos.x();
  odometry_message.pose.pose.position.y = pose.pos.y();
  odometry_message.pose.pose.position.z = pose.pos.z();
  odometry_message.pose.pose.orientation.x = quaternion.x();
  odometry_message.pose.pose.orientation.y = quaternion.y();
  odometry_message.pose.pose.orientation.z = quaternion.z();
  odometry_message.pose.pose.orientation.w = quaternion.w();
  odometry_message.twist.twist.linear.x = odometry_->Velocity().x();
  odometry_message.twist.twist.linear.y = odometry_->Velocity().y();
  odometry_message.twist.twist.linear.z = odometry_->Velocity().z();

  const auto& covariance = odometry_->Covariance();
  for (int row = 0; row < 6; ++row) {
    const int source_row = row < 3 ? row + 3 : row - 3;
    for (int column = 0; column < 6; ++column) {
      const int source_column = column < 3 ? column + 3 : column - 3;
      odometry_message.pose.covariance[row * 6 + column] =
          covariance(source_row, source_column);
    }
  }
  odometry_publisher_.publish(odometry_message);

  nav_msgs::Odometry state_estimation = odometry_message;
  state_estimation.header.frame_id = world_frame_;
  state_estimation.child_frame_id = sensor_frame_;
  state_estimation_publisher_.publish(state_estimation);

  tf::Transform transform;
  transform.setOrigin(tf::Vector3(pose.pos.x(), pose.pos.y(), pose.pos.z()));
  transform.setRotation(tf::Quaternion(quaternion.x(), quaternion.y(),
                                       quaternion.z(), quaternion.w()));
  transform_broadcaster_.sendTransform(
      tf::StampedTransform(transform, stamp, world_frame_, sensor_frame_));
}

void Ros1Convert::PublishPath() {
  const auto& pose = odometry_->Pose();
  if ((pose.pos - last_path_position_).norm() <= 0.1) {
    return;
  }
  const Eigen::Quaterniond quaternion = pose.rot.normalized();
  geometry_msgs::PoseStamped stamped_pose;
  stamped_pose.header.stamp.fromSec(odometry_->LidarEndTime());
  stamped_pose.header.frame_id = world_frame_;
  stamped_pose.pose.position.x = pose.pos.x();
  stamped_pose.pose.position.y = pose.pos.y();
  stamped_pose.pose.position.z = pose.pos.z();
  stamped_pose.pose.orientation.x = quaternion.x();
  stamped_pose.pose.orientation.y = quaternion.y();
  stamped_pose.pose.orientation.z = quaternion.z();
  stamped_pose.pose.orientation.w = quaternion.w();
  path_message_.header.stamp = stamped_pose.header.stamp;
  path_message_.poses.push_back(stamped_pose);
  path_publisher_.publish(path_message_);
  last_path_position_ = pose.pos;
}

void Ros1Convert::PublishRegisteredScan() {
  fastlio::PointCloudXYZI::Ptr cloud =
      odometry_->RegisteredScan(dense_publish_enabled_);
  sensor_msgs::PointCloud2 message;
  pcl::toROSMsg(*cloud, message);
  message.header.stamp.fromSec(odometry_->LidarEndTime());
  message.header.frame_id = world_frame_;
  registered_scan_publisher_.publish(message);
}

void Ros1Convert::Run() {
  ros::Rate rate(processing_rate_);
  fastlio::MeasureGroup group;
  while (ros::ok()) {
    ros::spinOnce();
    if (BuildMeasureGroup(group) && odometry_->run_odometry(group) &&
        odometry_->HasPose()) {
      PublishOdometry();
      if (path_enabled_) {
        PublishPath();
      }
      if (scan_publish_enabled_) {
        PublishRegisteredScan();
      }
    }
    rate.sleep();
  }
}

}  // namespace ros1
}  // namespace localization
}  // namespace jojo
