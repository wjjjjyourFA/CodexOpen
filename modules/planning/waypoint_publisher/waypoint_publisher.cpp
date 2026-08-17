#include "modules/planning/waypoint_publisher/waypoint_publisher.h"

#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace jojo {
namespace planning {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kNanosecondsPerSecond = 1.0e9;

}  // namespace

WaypointPublisher::WaypointPublisher(const WaypointPublisherConfig& config)
    : config_(config) {
  if (config_.frame_rate <= 0.0 || config_.wait_time < 0.0 ||
      config_.waypoint_xy_radius < 0.0 || config_.waypoint_z_bound < 0.0 ||
      config_.waypoint_yaw_threshold < 0.0 ||
      config_.waypoint_frame.empty() || config_.boundary_frame.empty()) {
    throw std::invalid_argument("Invalid waypoint publisher configuration");
  }
  ReadWaypointFile();
  if (config_.send_boundary) {
    ReadBoundaryFile();
  }
}

common_struct::Quaternion WaypointPublisher::QuaternionFromRpy(
    double roll, double pitch, double yaw) {
  const double half_roll = roll * 0.5;
  const double half_pitch = pitch * 0.5;
  const double half_yaw = yaw * 0.5;
  const double cr = std::cos(half_roll);
  const double sr = std::sin(half_roll);
  const double cp = std::cos(half_pitch);
  const double sp = std::sin(half_pitch);
  const double cy = std::cos(half_yaw);
  const double sy = std::sin(half_yaw);
  common_struct::Quaternion quaternion;
  quaternion.w = cr * cp * cy + sr * sp * sy;
  quaternion.x = sr * cp * cy - cr * sp * sy;
  quaternion.y = cr * sp * cy + sr * cp * sy;
  quaternion.z = cr * cp * sy - sr * sp * cy;
  return quaternion;
}

double WaypointPublisher::Yaw(
    const common_struct::Quaternion& quaternion) {
  const double sin_yaw =
      2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  const double cos_yaw =
      1.0 - 2.0 * (quaternion.y * quaternion.y +
                   quaternion.z * quaternion.z);
  return std::atan2(sin_yaw, cos_yaw);
}

void WaypointPublisher::ReadWaypointFile() {
  if (config_.waypoint_file.empty()) {
    return;
  }
  FILE* file = std::fopen(config_.waypoint_file.c_str(), "r");
  if (file == nullptr) {
    throw std::runtime_error("Cannot read waypoint file: " +
                             config_.waypoint_file);
  }
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  float roll = 0.0F;
  float pitch = 0.0F;
  float yaw = 0.0F;
  while (std::fscanf(file, "%f %f %f %f %f %f", &x, &y, &z, &roll,
                     &pitch, &yaw) == 6) {
    common_struct::Pose pose;
    pose.position = common_struct::Vector3d(x, y, z);
    pose.orientation = QuaternionFromRpy(
        roll * kPi / 180.0, pitch * kPi / 180.0, yaw * kPi / 180.0);
    waypoints_.push_back(pose);
  }
  std::fclose(file);
}

void WaypointPublisher::ReadBoundaryFile() {
  if (config_.boundary_file.empty()) {
    throw std::runtime_error("Boundary file is not configured");
  }
  FILE* file = std::fopen(config_.boundary_file.c_str(), "r");
  if (file == nullptr) {
    throw std::runtime_error("Cannot read boundary file: " +
                             config_.boundary_file);
  }
  common_struct::Vector3f point;
  while (std::fscanf(file, "%f %f %f", &point.x, &point.y, &point.z) == 3) {
    boundary_points_.push_back(point);
  }
  std::fclose(file);
}

void WaypointPublisher::SetOdometry(std::uint64_t timestamp_ns,
                                    const common_struct::Pose& pose) {
  current_time_ns_ = timestamp_ns;
  vehicle_pose_ = pose;
}

std::size_t WaypointPublisher::AddNavigationGoal(
    const common_struct::Pose& pose) {
  waypoints_.push_back(pose);
  return waypoints_.size();
}

WaypointPublisherOutput WaypointPublisher::Step() {
  WaypointPublisherOutput output;
  if (waypoints_.empty()) {
    return output;
  }
  if (waypoint_index_ >= waypoints_.size()) {
    waypoint_index_ = waypoints_.size() - 1;
  }

  const common_struct::Pose& target = waypoints_[waypoint_index_];
  const double delta_x = vehicle_pose_.position.x - target.position.x;
  const double delta_y = vehicle_pose_.position.y - target.position.y;
  const double delta_z = vehicle_pose_.position.z - target.position.z;
  const double yaw_difference = std::atan2(
      std::sin(Yaw(target.orientation) - Yaw(vehicle_pose_.orientation)),
      std::cos(Yaw(target.orientation) - Yaw(vehicle_pose_.orientation)));

  if (std::hypot(delta_x, delta_y) < config_.waypoint_xy_radius &&
      std::abs(delta_z) < config_.waypoint_z_bound &&
      std::abs(yaw_difference) < config_.waypoint_yaw_threshold &&
      !waiting_) {
    wait_start_time_ns_ = current_time_ns_;
    waiting_ = true;
  }

  const std::uint64_t wait_duration_ns = static_cast<std::uint64_t>(
      config_.wait_time * kNanosecondsPerSecond);
  if (waiting_ && wait_start_time_ns_ + wait_duration_ns < current_time_ns_ &&
      waypoint_index_ + 1 < waypoints_.size()) {
    ++waypoint_index_;
    waiting_ = false;
  }

  const std::uint64_t publish_period_ns = static_cast<std::uint64_t>(
      kNanosecondsPerSecond / config_.frame_rate);
  if (current_time_ns_ - waypoint_publish_time_ns_ > publish_period_ns) {
    const common_struct::Pose& current_target = waypoints_[waypoint_index_];
    if (!waiting_) {
      output.publish_waypoint = true;
      output.waypoint.header =
          common_struct::Header(current_time_ns_, config_.waypoint_frame);
      output.waypoint.pose = current_target;
      output.waypoint_show.header = output.waypoint.header;
      output.waypoint_show.point = current_target.position;
    }
    if (config_.send_speed) {
      output.publish_speed = true;
      output.speed = static_cast<float>(config_.speed);
    }
    if (config_.send_boundary) {
      output.publish_boundary = true;
      output.boundary.header =
          common_struct::Header(current_time_ns_, config_.boundary_frame);
      output.boundary.points = boundary_points_;
    }
    waypoint_publish_time_ns_ = current_time_ns_;
  }
  output.goal_valid = !waiting_;
  return output;
}

}  // namespace planning
}  // namespace jojo
