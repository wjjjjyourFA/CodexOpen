#include "tools/manual_loop_session_exporter/session_exporter.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <stdexcept>

#include <boost/filesystem.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

namespace jojo {
namespace tools {
namespace manual_loop {
namespace fs = boost::filesystem;

SessionExporter::SessionExporter(const SessionExporterConfig& config)
    : config_(config) {
  ValidateConfig();
  PrepareOutputDirectory();
}

SessionExporter::~SessionExporter() {
  try {
    Finalize();
  } catch (...) {
    // Destructors must not throw. AddFrame already persists text outputs after
    // every accepted keyframe, so a shutdown-time failure loses no new frame.
  }
}

void SessionExporter::ValidateConfig() const {
  if (config_.output_directory.empty() || config_.keyframe_distance < 0.0 ||
      config_.keyframe_angle_deg < 0.0 || config_.min_keyframe_time < 0.0 ||
      config_.voxel_leaf_size < 0.0) {
    throw std::invalid_argument("Invalid manual-loop session exporter configuration");
  }
}

void SessionExporter::PrepareOutputDirectory() {
  const fs::path root(config_.output_directory);
  const fs::path keyframe_directory = root / "key_point_frame";
  if (fs::exists(root) && !fs::is_empty(root)) {
    if (!config_.overwrite_existing) {
      throw std::runtime_error(
          "Output directory is not empty: " + config_.output_directory +
          ". Choose a new directory or enable overwrite_existing.");
    }
    fs::remove_all(root);
  }
  fs::create_directories(keyframe_directory);
}

bool SessionExporter::ShouldCreateKeyframe(
    const Eigen::Isometry3d& current_pose, double timestamp) const {
  if (keyframes_.empty()) {
    return true;
  }
  const ExportedKeyframe& previous = keyframes_.back();
  if (config_.min_keyframe_time > 0.0 &&
      timestamp - previous.timestamp < config_.min_keyframe_time) {
    return false;
  }
  const Eigen::Isometry3d relative =
      previous.pose_world_sensor.inverse() * current_pose;
  const double distance = relative.translation().norm();
  const double angle_deg =
      Eigen::AngleAxisd(relative.linear()).angle() * 180.0 / M_PI;
  const bool distance_reached = config_.keyframe_distance > 0.0 &&
                                distance >= config_.keyframe_distance;
  const bool angle_reached = config_.keyframe_angle_deg > 0.0 &&
                             angle_deg >= config_.keyframe_angle_deg;
  return distance_reached || angle_reached;
}

AddFrameResult SessionExporter::AddFrame(
    double timestamp, const Eigen::Isometry3d& pose_world_sensor,
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& input_cloud) {
  AddFrameResult result;
  if (!ShouldCreateKeyframe(pose_world_sensor, timestamp)) {
    return result;
  }
  if (!input_cloud) {
    throw std::invalid_argument("Input keyframe cloud is null");
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>(*input_cloud));
  std::vector<int> finite_indices;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, finite_indices);
  if (cloud->empty()) {
    return result;
  }

  if (config_.input_cloud_is_global) {
    pcl::transformPointCloud(*cloud, *cloud,
                             pose_world_sensor.inverse().matrix().cast<float>());
  }
  if (config_.voxel_leaf_size > 0.0) {
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
    const float leaf = static_cast<float>(config_.voxel_leaf_size);
    voxel_grid.setLeafSize(leaf, leaf, leaf);
    voxel_grid.setInputCloud(cloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(
        new pcl::PointCloud<pcl::PointXYZI>());
    voxel_grid.filter(*filtered);
    cloud = filtered;
  }

  result.index = keyframes_.size();
  const fs::path pcd_path = fs::path(config_.output_directory) /
                            "key_point_frame" /
                            (std::to_string(result.index) + ".pcd");
  if (pcl::io::savePCDFileBinary(pcd_path.string(), *cloud) != 0) {
    throw std::runtime_error("Failed to save keyframe: " + pcd_path.string());
  }

  ExportedKeyframe keyframe;
  keyframe.timestamp = timestamp;
  keyframe.pose_world_sensor = pose_world_sensor;
  keyframes_.push_back(keyframe);
  Finalize();
  result.saved = true;
  result.point_count = cloud->size();
  return result;
}

Eigen::Quaterniond SessionExporter::NormalizedQuaternion(
    const Eigen::Matrix3d& rotation) {
  Eigen::Quaterniond quaternion(rotation);
  quaternion.normalize();
  if (quaternion.w() < 0.0) {
    quaternion.coeffs() *= -1.0;
  }
  return quaternion;
}

void SessionExporter::Finalize() const {
  if (keyframes_.empty()) {
    return;
  }
  WriteTum();
  WriteG2o();
}

void SessionExporter::WriteTum() const {
  std::ofstream stream(
      (fs::path(config_.output_directory) / "optimized_poses_tum.txt").string(),
      std::ios::out | std::ios::trunc);
  if (!stream) {
    throw std::runtime_error("Cannot write optimized_poses_tum.txt");
  }
  stream << std::setprecision(15);
  for (const ExportedKeyframe& keyframe : keyframes_) {
    const Eigen::Quaterniond quaternion =
        NormalizedQuaternion(keyframe.pose_world_sensor.linear());
    const Eigen::Vector3d translation =
        keyframe.pose_world_sensor.translation();
    stream << keyframe.timestamp << ' ' << translation.x() << ' '
           << translation.y() << ' ' << translation.z() << ' '
           << quaternion.x() << ' ' << quaternion.y() << ' '
           << quaternion.z() << ' ' << quaternion.w() << '\n';
  }
}

void SessionExporter::WriteG2o() const {
  std::ofstream stream(
      (fs::path(config_.output_directory) / "pose_graph.g2o").string(),
      std::ios::out | std::ios::trunc);
  if (!stream) {
    throw std::runtime_error("Cannot write pose_graph.g2o");
  }
  stream << std::setprecision(15);
  for (std::size_t index = 0; index < keyframes_.size(); ++index) {
    const Eigen::Vector3d translation =
        keyframes_[index].pose_world_sensor.translation();
    const Eigen::Quaterniond quaternion =
        NormalizedQuaternion(keyframes_[index].pose_world_sensor.linear());
    stream << "VERTEX_SE3:QUAT " << index << ' ' << translation.x() << ' '
           << translation.y() << ' ' << translation.z() << ' '
           << quaternion.x() << ' ' << quaternion.y() << ' '
           << quaternion.z() << ' ' << quaternion.w() << '\n';
  }

  for (std::size_t index = 1; index < keyframes_.size(); ++index) {
    const Eigen::Isometry3d relative =
        keyframes_[index - 1].pose_world_sensor.inverse() *
        keyframes_[index].pose_world_sensor;
    const Eigen::Vector3d translation = relative.translation();
    const Eigen::Quaterniond quaternion =
        NormalizedQuaternion(relative.linear());
    stream << "EDGE_SE3:QUAT " << index - 1 << ' ' << index << ' '
           << translation.x() << ' ' << translation.y() << ' '
           << translation.z() << ' ' << quaternion.x() << ' '
           << quaternion.y() << ' ' << quaternion.z() << ' '
           << quaternion.w();
    for (int row = 0; row < 6; ++row) {
      for (int column = row; column < 6; ++column) {
        double value = 0.0;
        if (row == column) {
          value = row < 3 ? config_.translation_information
                          : config_.rotation_information;
        }
        stream << ' ' << value;
      }
    }
    stream << '\n';
  }
}

}  // namespace manual_loop
}  // namespace tools
}  // namespace jojo
