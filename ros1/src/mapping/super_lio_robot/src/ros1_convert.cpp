#include "super_lio_robot_ros1/ros1_convert.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

#include <pcl/common/io.h>
#include <pcl_conversions/pcl_conversions.h>

#include "basic/logs.h"
#include "lio/params.h"

namespace LI2Sup {
namespace ros1 {
namespace {

bool ValidPoint(double x, double y, double z) {
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
    return false;
  }
  const double squared_distance = x * x + y * y + z * z;
  if (squared_distance >= g_maxrange2) {
    return false;
  }
  const bool inside_inner_box =
      x > g_box_x_min && x < g_box_x_max &&
      y > g_box_y_min && y < g_box_y_max &&
      z > g_box_z_min && z < g_box_z_max;
  return !inside_inner_box;
}

bool HasField(const sensor_msgs::PointCloud2& message,
              const std::string& name) {
  return std::any_of(
      message.fields.begin(), message.fields.end(),
      [&name](const sensor_msgs::PointField& field) {
        return field.name == name;
      });
}

std::string LidarTypeToString(int type) {
  if (type <= 0 || type >= static_cast<int>(LID_TYPE_NAMES.size())) {
    return "UNKNOWN";
  }
  return LID_TYPE_NAMES[type];
}

}  // namespace

Ros1Convert::Ros1Convert(::ros::NodeHandle& node,
                         ::ros::NodeHandle& private_node)
    : node_(node), private_node_(private_node) {}

bool Ros1Convert::Init(const std::string& map_root) {
  if (!LoadAlgorithmParameters(map_root) ||
      !LoadInterfaceParameters() || !ValidateParameters()) {
    return false;
  }
  ConfigureRosIo();
  ROS_INFO_STREAM("super_lio_robot ROS1 adapter initialized: lidar="
                  << lidar_topic_ << " (" << LidarTypeToString(g_lidar_type)
                  << "), imu=" << imu_topic_
                  << ", map_directory=" << g_save_map_dir);
  return true;
}

bool Ros1Convert::LoadAlgorithmParameters(const std::string& map_root) {
  private_node_.param("map/save_map", g_save_map, false);
  private_node_.param("map/if_filter", g_if_filter, true);
  std::string configured_map_directory{"map"};
  private_node_.param("map/save_map_dir", configured_map_directory,
                      configured_map_directory);
  private_node_.param("map/map_name", g_map_name, std::string("map.pcd"));
  private_node_.param("map/ds_size", g_map_ds_size, 0.25F);
  private_node_.param("map/save_interval", g_pcd_save_interval, 500);
  if (!map_root.empty() &&
      !std::filesystem::path(configured_map_directory).is_absolute()) {
    g_save_map_dir =
        (std::filesystem::path(map_root) / configured_map_directory).string();
  } else {
    g_save_map_dir = configured_map_directory;
  }

  private_node_.param("eva/timer", g_time_eva, true);

  double range = 0.0;
  private_node_.param("sensor/lidar_type", g_lidar_type,
                      static_cast<int>(LID_TYPE::LIVOX));
  private_node_.param("sensor/blind", range, 0.4);
  g_blind2 = static_cast<float>(range * range);
  private_node_.param("sensor/maxrange", range, 60.0);
  g_maxrange2 = static_cast<float>(range * range);
  const float blind_range = std::sqrt(g_blind2);
  private_node_.param("sensor/x_min", g_box_x_min, -blind_range);
  private_node_.param("sensor/x_max", g_box_x_max, blind_range);
  private_node_.param("sensor/y_min", g_box_y_min, -blind_range);
  private_node_.param("sensor/y_max", g_box_y_max, blind_range);
  private_node_.param("sensor/z_min", g_box_z_min, -blind_range);
  private_node_.param("sensor/z_max", g_box_z_max, blind_range);
  private_node_.param("sensor/filter_rate", g_filter_rate, 3);
  private_node_.param("sensor/enable_downsample", g_enable_downsample, true);
  private_node_.param("sensor/voxel_fliter_size", g_voxel_fliter_size, 0.5F);
  private_node_.param("sensor/gravity_norm", g_gravity_norm, 9.7946);
  private_node_.param("sensor/imu_type", g_imu_type, 1);
  private_node_.param("sensor/imu_na", g_imu_na, 0.1);
  private_node_.param("sensor/imu_ng", g_imu_ng, 0.1);
  private_node_.param("sensor/imu_nba", g_imu_nba, 0.0001);
  private_node_.param("sensor/imu_nbg", g_imu_nbg, 0.0001);

  private_node_.param("extrinsic/qlu_or_rqu", g_qlu_or_rqu, true);
  private_node_.param("extrinsic/b_rawimu", g_b_rawimu, true);
  std::vector<double> lidar_imu{
      0.0, 0.0, 0.0,
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0};
  std::vector<double> odom_robot{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  private_node_.param("extrinsic/lidar_imu", lidar_imu, lidar_imu);
  private_node_.param("extrinsic/odom_robo", odom_robot, odom_robot);
  if (lidar_imu.size() != 12 || odom_robot.size() != 6) {
    ROS_ERROR("extrinsic/lidar_imu must have 12 values and odom_robo 6");
    return false;
  }
  BASIC::V3 translation(
      static_cast<BASIC::scalar>(lidar_imu[0]),
      static_cast<BASIC::scalar>(lidar_imu[1]),
      static_cast<BASIC::scalar>(lidar_imu[2]));
  BASIC::M3 rotation;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      rotation(row, column) = static_cast<BASIC::scalar>(
          lidar_imu[3 + row * 3 + column]);
    }
  }
  g_lidar_imu = BASIC::SE3(rotation, translation);

  translation = BASIC::V3(
      static_cast<BASIC::scalar>(odom_robot[0]),
      static_cast<BASIC::scalar>(odom_robot[1]),
      static_cast<BASIC::scalar>(odom_robot[2]));
  const Eigen::Matrix3d robot_rotation =
      (Eigen::AngleAxisd(odom_robot[5] / 180.0 * M_PI,
                         Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(odom_robot[4] / 180.0 * M_PI,
                         Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(odom_robot[3] / 180.0 * M_PI,
                         Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  rotation = robot_rotation.cast<BASIC::scalar>().transpose();
  g_odom_robo = BASIC::SE3(rotation, translation);
  g_lidar_robo_yaw =
      Eigen::AngleAxisd(odom_robot[5] / 180.0 * M_PI,
                        Eigen::Vector3d::UnitZ())
          .toRotationMatrix()
          .cast<BASIC::scalar>();

  int hash_capacity = 100000;
  private_node_.param("hash_map/hash_capacity", hash_capacity,
                      hash_capacity);
  g_ivox_capacity = static_cast<std::size_t>(std::max(hash_capacity, 1));
  private_node_.param("hash_map/vox_resolution", g_ivox_resolution, 0.5F);
  private_node_.param("hash_map/insert_resolution", g_insert_resolution,
                      g_ivox_resolution);

  private_node_.param("occupy_map/occ_resolution", g_occ_resolution, 0.1F);
  private_node_.param("occupy_map/occ_size_x", g_occ_size_x, 60.0F);
  private_node_.param("occupy_map/occ_size_y", g_occ_size_y, 60.0F);
  private_node_.param("occupy_map/occ_size_z", g_occ_size_z, 20.0F);
  private_node_.param("occupy_map/occ_update_range", g_occ_update_range,
                      20.0F);
  private_node_.param("occupy_map/enable_virtual_wall",
                      g_enable_virtual_wall, true);
  private_node_.param("occupy_map/slide_update", g_slide_update, true);

  private_node_.param("kf/kf_type", g_kf_type, 0);
  private_node_.param("kf/kf_max_iterations", g_kf_max_iterations, 4);
  private_node_.param("kf/kf_align_gravity", g_kf_align_gravity, true);
  private_node_.param("kf/kf_quit_eps", g_kf_quit_eps, 0.001);
  private_node_.param("submap/submap_resolution", g_submap_resolution, 0.5);
  private_node_.param("submap/submap_capacity", g_submap_capacity, 1000000);

  private_node_.param("output/robot", g_2_robot, false);
  private_node_.param("output/plan_env_world", g_2_plan_env_world, false);
  private_node_.param("output/plan_env_body", g_2_plan_env_body, false);
  private_node_.param("output/ml_map", g_2_ml_map, false);
  private_node_.param("output/planner", g_planner_enable, false);
  private_node_.param("output/map", g_visual_map, true);
  private_node_.param("output/dense", g_visual_dense, true);
  private_node_.param("output/pub_step", g_pub_step, 1);
  private_node_.param("output/ax7_compat_output", g_ax7_compat_output, false);

  private_node_.param("relocation/update_map", g_update_map, false);
  std::vector<double> initial_pose(6, 0.0);
  private_node_.param("relocation/init_pose", initial_pose, initial_pose);
  if (initial_pose.size() != 6) {
    ROS_ERROR("relocation/init_pose must have 6 values");
    return false;
  }
  g_init_px = initial_pose[0];
  g_init_py = initial_pose[1];
  g_init_pz = initial_pose[2];
  g_init_roll = initial_pose[3];
  g_init_pitch = initial_pose[4];
  g_init_yaw = initial_pose[5];
  return true;
}

bool Ros1Convert::LoadInterfaceParameters() {
  private_node_.param("topics/lidar", lidar_topic_, lidar_topic_);
  private_node_.param("topics/imu", imu_topic_, imu_topic_);
  private_node_.param("topics/odometry", odometry_topic_, odometry_topic_);
  private_node_.param("topics/state_estimation", state_estimation_topic_,
                      state_estimation_topic_);
  private_node_.param("topics/path", path_topic_, path_topic_);
  private_node_.param("topics/registered_scan", registered_scan_topic_,
                      registered_scan_topic_);
  private_node_.param("topics/world_cloud", world_cloud_topic_,
                      world_cloud_topic_);
  private_node_.param("topics/imu_odometry", imu_odometry_topic_,
                      imu_odometry_topic_);
  private_node_.param("topics/robot_odometry", robot_odometry_topic_,
                      robot_odometry_topic_);
  private_node_.param("topics/robot_pose", robot_pose_topic_,
                      robot_pose_topic_);
  private_node_.param("topics/global_map", global_map_topic_,
                      global_map_topic_);
  private_node_.param("topics/initial_pose", initial_pose_topic_,
                      initial_pose_topic_);
  private_node_.param("frames/world", world_frame_, world_frame_);
  private_node_.param("frames/map", map_frame_, map_frame_);
  private_node_.param("frames/sensor", sensor_frame_, sensor_frame_);
  private_node_.param("queues/lidar", lidar_queue_size_, lidar_queue_size_);
  private_node_.param("queues/imu", imu_queue_size_, imu_queue_size_);
  private_node_.param("queues/odometry", odometry_queue_size_,
                      odometry_queue_size_);
  private_node_.param("queues/path", path_queue_size_, path_queue_size_);
  private_node_.param("queues/cloud", cloud_queue_size_, cloud_queue_size_);
  private_node_.param("queues/initial_pose", initial_pose_queue_size_,
                      initial_pose_queue_size_);
  private_node_.param("transport/processing_rate", processing_rate_,
                      processing_rate_);
  private_node_.param("transport/path_min_distance", path_min_distance_,
                      path_min_distance_);
  private_node_.param("transport/global_map_latched", global_map_latched_,
                      global_map_latched_);
  return true;
}

bool Ros1Convert::ValidateParameters() const {
  if (g_lidar_type < static_cast<int>(LID_TYPE::LIVOX) ||
      g_lidar_type > static_cast<int>(LID_TYPE::NOR) ||
      g_filter_rate <= 0 || g_voxel_fliter_size <= 0.0F ||
      g_ivox_resolution <= 0.0F || g_kf_max_iterations <= 0 ||
      g_pub_step <= 0 || processing_rate_ <= 0.0 ||
      path_min_distance_ < 0.0 || lidar_queue_size_ <= 0 ||
      imu_queue_size_ <= 0 || odometry_queue_size_ <= 0 ||
      path_queue_size_ <= 0 || cloud_queue_size_ <= 0 ||
      initial_pose_queue_size_ <= 0) {
    ROS_ERROR("super_lio_robot configuration contains invalid values");
    return false;
  }
  if (g_box_x_min >= g_box_x_max || g_box_y_min >= g_box_y_max ||
      g_box_z_min >= g_box_z_max || g_maxrange2 <= g_blind2) {
    ROS_ERROR("super_lio_robot sensor range configuration is invalid");
    return false;
  }
  return true;
}

void Ros1Convert::ConfigureRosIo() {
  node_.setCallbackQueue(&callback_queue_);
  const ::ros::TransportHints transport =
      ::ros::TransportHints().tcpNoDelay();
  if (g_lidar_type == LID_TYPE::LIVOX) {
    lidar_subscriber_ = node_.subscribe<LivoxMessage>(
        lidar_topic_, lidar_queue_size_, &Ros1Convert::LivoxCallback, this,
        transport);
  } else {
    lidar_subscriber_ = node_.subscribe<sensor_msgs::PointCloud2>(
        lidar_topic_, lidar_queue_size_,
        &Ros1Convert::StandardCloudCallback, this, transport);
  }
  imu_subscriber_ = node_.subscribe<sensor_msgs::Imu>(
      imu_topic_, imu_queue_size_, &Ros1Convert::ImuCallback, this, transport);

  odometry_publisher_ = node_.advertise<nav_msgs::Odometry>(
      odometry_topic_, odometry_queue_size_);
  state_estimation_publisher_ = node_.advertise<nav_msgs::Odometry>(
      state_estimation_topic_, odometry_queue_size_);
  path_publisher_ =
      node_.advertise<nav_msgs::Path>(path_topic_, path_queue_size_);
  registered_scan_publisher_ = node_.advertise<sensor_msgs::PointCloud2>(
      registered_scan_topic_, cloud_queue_size_);
  world_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2>(
      world_cloud_topic_, cloud_queue_size_);
  imu_odometry_publisher_ = node_.advertise<nav_msgs::Odometry>(
      imu_odometry_topic_, cloud_queue_size_);
  robot_odometry_publisher_ = node_.advertise<nav_msgs::Odometry>(
      robot_odometry_topic_, cloud_queue_size_);
  robot_pose_publisher_ = node_.advertise<geometry_msgs::PoseStamped>(
      robot_pose_topic_, cloud_queue_size_);
  global_map_publisher_ = node_.advertise<sensor_msgs::PointCloud2>(
      global_map_topic_, 1, global_map_latched_);
  global_map_timer_ = node_.createTimer(
      ::ros::Duration(1.0), &Ros1Convert::GlobalMapTimerCallback, this,
      false, false);
  path_.header.frame_id = world_frame_;
}

void Ros1Convert::SpinOnce() {
  callback_queue_.callAvailable();
}

void Ros1Convert::setESKF(std::shared_ptr<ESKF>& eskf) {
  eskf_ = eskf;
}

void Ros1Convert::LivoxCallback(
    const typename LivoxMessage::ConstPtr& message) {
  if (message->point_num < 10) {
    return;
  }
  LidarData lidar;
  lidar.pc.reset(new pcl::PointCloud<PointXTZIT>());
  lidar.pc->reserve(message->point_num / g_filter_rate + 1);
  double offset_time = 0.0;
  for (std::size_t index = 0; index < message->point_num;
       index += static_cast<std::size_t>(g_filter_rate)) {
    const auto& point = message->points[index];
    const auto tag = point.tag & 0x30;
    if ((tag == 0x10 || tag == 0x00) &&
        ValidPoint(point.x, point.y, point.z)) {
      offset_time = point.offset_time * 1e-9;
      lidar.pc->emplace_back(point.x, point.y, point.z,
                             point.reflectivity, offset_time);
    }
  }
  lidar.start_time = message->header.stamp.toSec();
  lidar.end_time = lidar.start_time + offset_time;
  lidar_buffer_.push_back(std::move(lidar));
}

void Ros1Convert::StandardCloudCallback(
    const sensor_msgs::PointCloud2ConstPtr& message) {
  if (message->data.size() < 10) {
    return;
  }
  LidarData lidar;
  lidar.pc.reset(new pcl::PointCloud<PointXTZIT>());
  double offset_time = 0.0;

  switch (g_lidar_type) {
    case LID_TYPE::HESAI16: {
      pcl::PointCloud<hesai_ros::Point> source;
      pcl::fromROSMsg(*message, source);
      if (source.empty()) return;
      lidar.pc->reserve(source.size() / g_filter_rate + 1);
      lidar.start_time = source.front().timestamp;
      for (std::size_t index = 0; index < source.size();
           index += static_cast<std::size_t>(g_filter_rate)) {
        const auto& point = source[index];
        if (!ValidPoint(point.x, point.y, point.z)) continue;
        offset_time = point.timestamp - lidar.start_time;
        lidar.pc->emplace_back(point.x, point.y, point.z,
                               point.intensity, offset_time);
      }
      lidar.end_time = lidar.start_time + offset_time;
      break;
    }
    case LID_TYPE::VEL_NCLT: {
      pcl::PointCloud<NCLT::Point> source;
      pcl::fromROSMsg(*message, source);
      lidar.pc->reserve(source.size() / g_filter_rate + 1);
      lidar.start_time = message->header.stamp.toSec();
      for (std::size_t index = 0; index < source.size();
           index += static_cast<std::size_t>(g_filter_rate)) {
        const auto& point = source[index];
        if (!ValidPoint(point.x, point.y, point.z)) continue;
        offset_time = point.time * 1e-6;
        lidar.pc->emplace_back(point.x, point.y, point.z, 1.0F,
                               offset_time);
      }
      lidar.end_time = lidar.start_time + offset_time;
      break;
    }
    case LID_TYPE::VELO16:
    case LID_TYPE::VELO32: {
      pcl::PointCloud<velodyne_ros::Point> source;
      pcl::fromROSMsg(*message, source);
      lidar.pc->reserve(source.size() / g_filter_rate + 1);
      lidar.start_time = message->header.stamp.toSec();
      for (std::size_t index = 0; index < source.size();
           index += static_cast<std::size_t>(g_filter_rate)) {
        const auto& point = source[index];
        if (!ValidPoint(point.x, point.y, point.z)) continue;
        offset_time = point.time;
        lidar.pc->emplace_back(point.x, point.y, point.z,
                               point.intensity, offset_time);
      }
      lidar.end_time = lidar.start_time + offset_time;
      break;
    }
    case LID_TYPE::OUSTER: {
      pcl::PointCloud<ouster_ros::Point> source;
      pcl::fromROSMsg(*message, source);
      lidar.pc->reserve(source.size() / g_filter_rate + 1);
      lidar.start_time = message->header.stamp.toSec();
      for (std::size_t index = 0; index < source.size();
           index += static_cast<std::size_t>(g_filter_rate)) {
        const auto& point = source[index];
        if (!ValidPoint(point.x, point.y, point.z)) continue;
        offset_time = point.t * 1e-9;
        lidar.pc->emplace_back(point.x, point.y, point.z,
                               point.intensity, offset_time);
      }
      lidar.end_time = lidar.start_time + offset_time;
      break;
    }
    case LID_TYPE::RS128: {
      pcl::PointCloud<rsm1_ros::Point> source;
      pcl::fromROSMsg(*message, source);
      if (source.empty()) return;
      lidar.pc->reserve(source.size() / g_filter_rate + 1);
      lidar.start_time = source.front().timestamp;
      for (std::size_t index = 0; index < source.size();
           index += static_cast<std::size_t>(g_filter_rate)) {
        const auto& point = source[index];
        if (!ValidPoint(point.x, point.y, point.z)) continue;
        offset_time = point.timestamp - lidar.start_time;
        lidar.pc->emplace_back(point.x, point.y, point.z,
                               point.intensity, offset_time);
      }
      lidar.end_time = lidar.start_time + offset_time;
      break;
    }
    case LID_TYPE::NOR: {
      lidar.start_time = message->header.stamp.toSec();
      lidar.end_time = lidar.start_time;
      if (HasField(*message, "intensity")) {
        pcl::PointCloud<pcl::PointXYZI> source;
        pcl::fromROSMsg(*message, source);
        lidar.pc->reserve(source.size() / g_filter_rate + 1);
        for (std::size_t index = 0; index < source.size();
             index += static_cast<std::size_t>(g_filter_rate)) {
          const auto& point = source[index];
          if (!ValidPoint(point.x, point.y, point.z)) continue;
          lidar.pc->emplace_back(point.x, point.y, point.z,
                                 point.intensity, 0.0);
        }
      } else {
        pcl::PointCloud<pcl::PointXYZ> source;
        pcl::fromROSMsg(*message, source);
        lidar.pc->reserve(source.size() / g_filter_rate + 1);
        for (std::size_t index = 0; index < source.size();
             index += static_cast<std::size_t>(g_filter_rate)) {
          const auto& point = source[index];
          if (!ValidPoint(point.x, point.y, point.z)) continue;
          lidar.pc->emplace_back(point.x, point.y, point.z, 0.0F, 0.0);
        }
      }
      break;
    }
    default:
      ROS_WARN_THROTTLE(5.0, "Unsupported standard lidar type: %d",
                        g_lidar_type);
      return;
  }
  lidar_buffer_.push_back(std::move(lidar));
}

void Ros1Convert::ImuCallback(const sensor_msgs::ImuConstPtr& message) {
  IMUData data;
  data.secs = message->header.stamp.toSec();
  double gyro_x = message->angular_velocity.x;
  double gyro_y = message->angular_velocity.y;
  double gyro_z = message->angular_velocity.z;
  if (!g_b_rawimu) {
    gyro_x *= M_PI / 180.0;
    gyro_y *= M_PI / 180.0;
    gyro_z *= M_PI / 180.0;
  }
  if (g_qlu_or_rqu) {
    data.acc = BASIC::V3(message->linear_acceleration.x,
                         message->linear_acceleration.y,
                         message->linear_acceleration.z);
    data.gyr = BASIC::V3(gyro_x, gyro_y, gyro_z);
  } else {
    data.acc = BASIC::V3(message->linear_acceleration.y,
                         -message->linear_acceleration.x,
                         message->linear_acceleration.z);
    data.gyr = BASIC::V3(gyro_y, -gyro_x, gyro_z);
  }

  if (data.secs < last_imu_timestamp_) {
    ROS_WARN("IMU timestamp looped back; clearing IMU buffer");
    imu_buffer_.clear();
  }
  imu_buffer_.push_back(data);
  last_imu_timestamp_ = data.secs;

  if (!eskf_) return;
  DynamicState imu_state;
  DynamicState robot_state;
  if (!eskf_->Predict(data, imu_state, robot_state)) return;

  nav_msgs::Odometry imu_odometry;
  imu_odometry.header.stamp = message->header.stamp;
  imu_odometry.header.frame_id = world_frame_;
  imu_odometry.child_frame_id = sensor_frame_;
  imu_odometry.pose.pose.position.x = imu_state.p.x();
  imu_odometry.pose.pose.position.y = imu_state.p.y();
  imu_odometry.pose.pose.position.z = imu_state.p.z();
  BASIC::Quat imu_orientation(imu_state.R);
  imu_orientation.normalize();
  imu_odometry.pose.pose.orientation.x = imu_orientation.x();
  imu_odometry.pose.pose.orientation.y = imu_orientation.y();
  imu_odometry.pose.pose.orientation.z = imu_orientation.z();
  imu_odometry.pose.pose.orientation.w = imu_orientation.w();
  imu_odometry.twist.twist.linear.x = imu_state.v.x();
  imu_odometry.twist.twist.linear.y = imu_state.v.y();
  imu_odometry.twist.twist.linear.z = imu_state.v.z();
  imu_odometry.twist.twist.angular.x = imu_state.w.x();
  imu_odometry.twist.twist.angular.y = imu_state.w.y();
  imu_odometry.twist.twist.angular.z = imu_state.w.z();
  imu_odometry_publisher_.publish(imu_odometry);

  nav_msgs::Odometry robot_odometry;
  robot_odometry.header.stamp = message->header.stamp;
  robot_odometry.header.frame_id = world_frame_;
  robot_odometry.pose.pose.position.x = robot_state.p.x();
  robot_odometry.pose.pose.position.y = robot_state.p.y();
  robot_odometry.pose.pose.position.z = robot_state.p.z();
  BASIC::Quat robot_orientation(robot_state.R);
  robot_orientation.normalize();
  robot_odometry.pose.pose.orientation.x = robot_orientation.x();
  robot_odometry.pose.pose.orientation.y = robot_orientation.y();
  robot_odometry.pose.pose.orientation.z = robot_orientation.z();
  robot_odometry.pose.pose.orientation.w = robot_orientation.w();
  robot_odometry_publisher_.publish(robot_odometry);
}

bool Ros1Convert::sync_measure(MeasureGroup& measurements) {
  if (lidar_buffer_.empty() || imu_buffer_.empty()) return false;
  if (!lidar_pushed_) {
    measurements.lidar = lidar_buffer_.front();
    lidar_pushed_ = true;
  }
  if (last_lidar_timestamp_ > measurements.lidar.end_time) {
    lidar_buffer_.pop_front();
    lidar_pushed_ = false;
    return false;
  }
  if (last_imu_timestamp_ < measurements.lidar.end_time) return false;

  measurements.imu.clear();
  while (!imu_buffer_.empty() &&
         imu_buffer_.front().secs <= measurements.lidar.end_time) {
    measurements.imu.push_back(imu_buffer_.front());
    imu_buffer_.pop_front();
  }
  last_lidar_timestamp_ = measurements.lidar.end_time;
  lidar_buffer_.pop_front();
  lidar_pushed_ = false;
  return true;
}

void Ros1Convert::pub_odom(const NavState& state) {
  nav_msgs::Odometry odometry;
  odometry.header.frame_id = world_frame_;
  odometry.child_frame_id = sensor_frame_;
  odometry.header.stamp.fromSec(state.timestamp);
  odometry.pose.pose.position.x = state.p.x();
  odometry.pose.pose.position.y = state.p.y();
  odometry.pose.pose.position.z = state.p.z();
  const BASIC::V4 orientation = state.R.coeffs();
  odometry.pose.pose.orientation.x = orientation[0];
  odometry.pose.pose.orientation.y = orientation[1];
  odometry.pose.pose.orientation.z = orientation[2];
  odometry.pose.pose.orientation.w = orientation[3];
  odometry.twist.twist.linear.x = state.v.x();
  odometry.twist.twist.linear.y = state.v.y();
  odometry.twist.twist.linear.z = state.v.z();
  odometry_publisher_.publish(odometry);

  if (g_ax7_compat_output) {
    nav_msgs::Odometry state_estimation = odometry;
    state_estimation.header.frame_id = map_frame_;
    state_estimation.child_frame_id = sensor_frame_;
    state_estimation_publisher_.publish(state_estimation);
  }

  const BASIC::V3 robot_position =
      state.R.R_ * (-g_odom_robo.R_ * g_odom_robo.t_) + state.p;
  if (g_2_robot) {
    geometry_msgs::PoseStamped robot_pose;
    robot_pose.header.stamp = odometry.header.stamp;
    robot_pose.header.frame_id = world_frame_;
    robot_pose.pose.position.x = robot_position.x();
    robot_pose.pose.position.y = robot_position.y();
    robot_pose.pose.position.z = robot_position.z();
    BASIC::Quat robot_orientation(state.R.R_ * g_odom_robo.R_);
    robot_orientation.normalize();
    robot_pose.pose.orientation.x = robot_orientation.x();
    robot_pose.pose.orientation.y = robot_orientation.y();
    robot_pose.pose.orientation.z = robot_orientation.z();
    robot_pose.pose.orientation.w = robot_orientation.w();
    robot_pose_publisher_.publish(robot_pose);
  }

  if ((last_path_point_ - robot_position).norm() > path_min_distance_) {
    path_.header.stamp = odometry.header.stamp;
    geometry_msgs::PoseStamped point;
    point.header = odometry.header;
    point.pose = odometry.pose.pose;
    path_.poses.push_back(point);
    path_publisher_.publish(path_);
    last_path_point_ = robot_position;
  }

  tf::Transform transform;
  transform.setOrigin(tf::Vector3(state.p.x(), state.p.y(), state.p.z()));
  tf::Quaternion quaternion(
      orientation[0], orientation[1], orientation[2], orientation[3]);
  transform.setRotation(quaternion);
  transform_broadcaster_.sendTransform(tf::StampedTransform(
      transform, odometry.header.stamp, map_frame_, sensor_frame_));
}

void Ros1Convert::pub_cloud_world(const BASIC::CloudPtr& cloud,
                                  double timestamp) {
  sensor_msgs::PointCloud2 message;
  pcl::toROSMsg(*cloud, message);
  message.header.frame_id = world_frame_;
  message.header.stamp.fromSec(timestamp);
  world_cloud_publisher_.publish(message);
}

void Ros1Convert::pub_registered_scan(const BASIC::CloudPtr& cloud,
                                      double timestamp) {
  if (!g_ax7_compat_output) return;
  sensor_msgs::PointCloud2 message;
  pcl::toROSMsg(*cloud, message);
  message.header.frame_id = map_frame_;
  message.header.stamp.fromSec(timestamp);
  registered_scan_publisher_.publish(message);
}

void Ros1Convert::set_global_map(const BASIC::CloudPtr& global_map) {
  pcl::toROSMsg(*global_map, global_map_message_);
  global_map_message_.header.frame_id = world_frame_;
  global_map_message_.header.stamp = ::ros::Time::now();
  global_map_publisher_.publish(global_map_message_);
  global_map_timer_count_ = 0;
  global_map_publish_interval_ = 1;
  global_map_timer_.start();
}

void Ros1Convert::GlobalMapTimerCallback(const ::ros::TimerEvent&) {
  if (global_map_message_.data.empty()) return;
  ++global_map_timer_count_;
  if (global_map_timer_count_ % global_map_publish_interval_ != 0) return;
  global_map_timer_count_ = 0;
  global_map_publish_interval_ =
      std::min(global_map_publish_interval_ + 1, 10);
  global_map_message_.header.stamp = ::ros::Time::now();
  global_map_publisher_.publish(global_map_message_);
}

void Ros1Convert::set_initial_data(BASIC::SE3& initial_pose,
                                   bool& has_initial_guess,
                                   bool initialization_finished) {
  if (initialization_finished) {
    initial_pose_subscriber_.shutdown();
    initial_pose_ = nullptr;
    has_initial_guess_ = nullptr;
    return;
  }
  initial_pose_ = &initial_pose;
  has_initial_guess_ = &has_initial_guess;
  if (!initial_pose_subscriber_) {
    initial_pose_subscriber_ =
        node_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
            initial_pose_topic_, initial_pose_queue_size_,
            &Ros1Convert::InitialPoseCallback, this);
  }
}

void Ros1Convert::InitialPoseCallback(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr& message) {
  if (initial_pose_ == nullptr || has_initial_guess_ == nullptr) return;
  BASIC::V3 translation(message->pose.pose.position.x,
                        message->pose.pose.position.y, 0.2);
  BASIC::Quat rotation(message->pose.pose.orientation.w,
                       message->pose.pose.orientation.x,
                       message->pose.pose.orientation.y,
                       message->pose.pose.orientation.z);
  rotation.normalize();
  *initial_pose_ = BASIC::SE3(
      BASIC::SO3(rotation.toRotationMatrix()), translation);
  *has_initial_guess_ = true;
  LOG(INFO) << YELLOW << " ---> GET Initial guess: "
            << translation.transpose() << " yaw: "
            << rotation.toRotationMatrix().eulerAngles(0, 1, 2).transpose()
            << RESET;
}

}  // namespace ros1
}  // namespace LI2Sup
