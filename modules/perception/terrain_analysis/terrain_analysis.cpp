#include "modules/perception/terrain_analysis/terrain_analysis.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace jojo {
namespace perception {
namespace {

constexpr double kPi = 3.14159265358979323846;

void QuaternionToRpy(const common_struct::Quaternion& quaternion,
                     double* roll, double* pitch, double* yaw) {
  const double sin_roll =
      2.0 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z);
  const double cos_roll =
      1.0 - 2.0 * (quaternion.x * quaternion.x +
                   quaternion.y * quaternion.y);
  *roll = std::atan2(sin_roll, cos_roll);

  const double sin_pitch =
      2.0 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x);
  if (std::fabs(sin_pitch) >= 1.0) {
    *pitch = std::copysign(kPi / 2.0, sin_pitch);
  } else {
    *pitch = std::asin(sin_pitch);
  }

  const double sin_yaw =
      2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  const double cos_yaw =
      1.0 - 2.0 * (quaternion.y * quaternion.y +
                   quaternion.z * quaternion.z);
  *yaw = std::atan2(sin_yaw, cos_yaw);
}

}  // namespace

TerrainAnalysis::TerrainAnalysis(const TerrainAnalysisConfig& config)
    : config_(config),
      cropped_scan_(new Cloud),
      downsampled_scan_(new Cloud),
      terrain_cloud_(new Cloud),
      elevation_cloud_(new Cloud),
      clearing_distance_(config.clearing_distance) {
  for (CloudPtr& cloud : terrain_voxel_clouds_) {
    cloud.reset(new Cloud);
  }
  downsample_filter_.setLeafSize(config_.scan_voxel_size,
                                 config_.scan_voxel_size,
                                 config_.scan_voxel_size);
}

void TerrainAnalysis::SetOdometry(const common_struct::Pose& pose) {
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  QuaternionToRpy(pose.orientation, &roll, &pitch, &yaw);
  vehicle_roll_ = static_cast<float>(roll);
  vehicle_pitch_ = static_cast<float>(pitch);
  vehicle_yaw_ = static_cast<float>(yaw);
  vehicle_x_ = static_cast<float>(pose.position.x);
  vehicle_y_ = static_cast<float>(pose.position.y);
  vehicle_z_ = static_cast<float>(pose.position.z);

  sin_vehicle_roll_ = std::sin(vehicle_roll_);
  cos_vehicle_roll_ = std::cos(vehicle_roll_);
  sin_vehicle_pitch_ = std::sin(vehicle_pitch_);
  cos_vehicle_pitch_ = std::cos(vehicle_pitch_);
  sin_vehicle_yaw_ = std::sin(vehicle_yaw_);
  cos_vehicle_yaw_ = std::cos(vehicle_yaw_);

  if (no_data_initialized_ == 0) {
    recorded_vehicle_x_ = vehicle_x_;
    recorded_vehicle_y_ = vehicle_y_;
    no_data_initialized_ = 1;
  }
  if (no_data_initialized_ == 1) {
    const float dx = vehicle_x_ - recorded_vehicle_x_;
    const float dy = vehicle_y_ - recorded_vehicle_y_;
    if (std::sqrt(dx * dx + dy * dy) >= config_.no_decay_distance) {
      no_data_initialized_ = 2;
    }
  }
}

void TerrainAnalysis::RequestClearing(double distance) {
  no_data_initialized_ = 0;
  clearing_distance_ = distance;
  clearing_requested_ = true;
}

void TerrainAnalysis::CropScan(double timestamp_seconds,
                               const Cloud& registered_scan) {
  scan_time_ = timestamp_seconds;
  if (!system_initialized_) {
    system_initial_time_ = scan_time_;
    system_initialized_ = true;
  }

  cropped_scan_->clear();
  cropped_scan_->reserve(registered_scan.size());
  for (const pcl::PointXYZI& input_point : registered_scan.points) {
    pcl::PointXYZI point = input_point;
    const float dx = point.x - vehicle_x_;
    const float dy = point.y - vehicle_y_;
    const float distance = std::sqrt(dx * dx + dy * dy);
    if (point.z - vehicle_z_ >
            config_.min_relative_z - config_.distance_ratio_z * distance &&
        point.z - vehicle_z_ <
            config_.max_relative_z + config_.distance_ratio_z * distance &&
        distance < kTerrainVoxelSize * (kTerrainVoxelHalfWidth + 1)) {
      point.intensity = static_cast<float>(scan_time_ - system_initial_time_);
      cropped_scan_->push_back(point);
    }
  }
}

void TerrainAnalysis::RollTerrainVoxels() {
  float center_x = kTerrainVoxelSize * terrain_voxel_shift_x_;
  float center_y = kTerrainVoxelSize * terrain_voxel_shift_y_;

  while (vehicle_x_ - center_x < -kTerrainVoxelSize) {
    for (int y = 0; y < kTerrainVoxelWidth; ++y) {
      CloudPtr recycled =
          terrain_voxel_clouds_[kTerrainVoxelWidth *
                                    (kTerrainVoxelWidth - 1) +
                                y];
      for (int x = kTerrainVoxelWidth - 1; x >= 1; --x) {
        terrain_voxel_clouds_[kTerrainVoxelWidth * x + y] =
            terrain_voxel_clouds_[kTerrainVoxelWidth * (x - 1) + y];
      }
      terrain_voxel_clouds_[y] = recycled;
      terrain_voxel_clouds_[y]->clear();
    }
    --terrain_voxel_shift_x_;
    center_x = kTerrainVoxelSize * terrain_voxel_shift_x_;
  }

  while (vehicle_x_ - center_x > kTerrainVoxelSize) {
    for (int y = 0; y < kTerrainVoxelWidth; ++y) {
      CloudPtr recycled = terrain_voxel_clouds_[y];
      for (int x = 0; x < kTerrainVoxelWidth - 1; ++x) {
        terrain_voxel_clouds_[kTerrainVoxelWidth * x + y] =
            terrain_voxel_clouds_[kTerrainVoxelWidth * (x + 1) + y];
      }
      const int destination =
          kTerrainVoxelWidth * (kTerrainVoxelWidth - 1) + y;
      terrain_voxel_clouds_[destination] = recycled;
      terrain_voxel_clouds_[destination]->clear();
    }
    ++terrain_voxel_shift_x_;
    center_x = kTerrainVoxelSize * terrain_voxel_shift_x_;
  }

  while (vehicle_y_ - center_y < -kTerrainVoxelSize) {
    for (int x = 0; x < kTerrainVoxelWidth; ++x) {
      CloudPtr recycled =
          terrain_voxel_clouds_[kTerrainVoxelWidth * x +
                                (kTerrainVoxelWidth - 1)];
      for (int y = kTerrainVoxelWidth - 1; y >= 1; --y) {
        terrain_voxel_clouds_[kTerrainVoxelWidth * x + y] =
            terrain_voxel_clouds_[kTerrainVoxelWidth * x + y - 1];
      }
      terrain_voxel_clouds_[kTerrainVoxelWidth * x] = recycled;
      terrain_voxel_clouds_[kTerrainVoxelWidth * x]->clear();
    }
    --terrain_voxel_shift_y_;
    center_y = kTerrainVoxelSize * terrain_voxel_shift_y_;
  }

  while (vehicle_y_ - center_y > kTerrainVoxelSize) {
    for (int x = 0; x < kTerrainVoxelWidth; ++x) {
      CloudPtr recycled = terrain_voxel_clouds_[kTerrainVoxelWidth * x];
      for (int y = 0; y < kTerrainVoxelWidth - 1; ++y) {
        terrain_voxel_clouds_[kTerrainVoxelWidth * x + y] =
            terrain_voxel_clouds_[kTerrainVoxelWidth * x + y + 1];
      }
      const int destination =
          kTerrainVoxelWidth * x + (kTerrainVoxelWidth - 1);
      terrain_voxel_clouds_[destination] = recycled;
      terrain_voxel_clouds_[destination]->clear();
    }
    ++terrain_voxel_shift_y_;
    center_y = kTerrainVoxelSize * terrain_voxel_shift_y_;
  }
}

void TerrainAnalysis::AccumulateCurrentScan() {
  if (!config_.use_accumulation) {
    for (int index = 0; index < kTerrainVoxelCount; ++index) {
      terrain_voxel_clouds_[index]->clear();
      terrain_voxel_update_count_[index] = 0;
      terrain_voxel_update_time_[index] =
          static_cast<float>(scan_time_ - system_initial_time_);
    }
  }

  for (const pcl::PointXYZI& point : cropped_scan_->points) {
    int x = static_cast<int>(
                (point.x - vehicle_x_ + kTerrainVoxelSize / 2.0F) /
                kTerrainVoxelSize) +
            kTerrainVoxelHalfWidth;
    int y = static_cast<int>(
                (point.y - vehicle_y_ + kTerrainVoxelSize / 2.0F) /
                kTerrainVoxelSize) +
            kTerrainVoxelHalfWidth;
    if (point.x - vehicle_x_ + kTerrainVoxelSize / 2.0F < 0.0F) {
      --x;
    }
    if (point.y - vehicle_y_ + kTerrainVoxelSize / 2.0F < 0.0F) {
      --y;
    }
    if (x >= 0 && x < kTerrainVoxelWidth && y >= 0 &&
        y < kTerrainVoxelWidth) {
      const int index = kTerrainVoxelWidth * x + y;
      terrain_voxel_clouds_[index]->push_back(point);
      ++terrain_voxel_update_count_[index];
    }
  }
}

void TerrainAnalysis::RefreshTerrainVoxels(double timestamp_seconds) {
  for (int index = 0; index < kTerrainVoxelCount; ++index) {
    const double relative_time = timestamp_seconds - system_initial_time_;
    if (!config_.use_accumulation ||
        terrain_voxel_update_count_[index] >=
            config_.voxel_point_update_threshold ||
        relative_time - terrain_voxel_update_time_[index] >=
            config_.voxel_time_update_threshold ||
        clearing_requested_) {
      CloudPtr cloud = terrain_voxel_clouds_[index];
      downsampled_scan_->clear();
      downsample_filter_.setInputCloud(cloud);
      downsample_filter_.filter(*downsampled_scan_);

      cloud->clear();
      for (const pcl::PointXYZI& point : downsampled_scan_->points) {
        const float dx = point.x - vehicle_x_;
        const float dy = point.y - vehicle_y_;
        const float distance = std::sqrt(dx * dx + dy * dy);
        if (point.z - vehicle_z_ >
                config_.min_relative_z -
                    config_.distance_ratio_z * distance &&
            point.z - vehicle_z_ <
                config_.max_relative_z +
                    config_.distance_ratio_z * distance &&
            (!config_.use_accumulation ||
             relative_time - point.intensity < config_.decay_time ||
             distance < config_.no_decay_distance) &&
            !(distance < clearing_distance_ && clearing_requested_)) {
          cloud->push_back(point);
        }
      }
      terrain_voxel_update_count_[index] = 0;
      terrain_voxel_update_time_[index] = static_cast<float>(relative_time);
    }
  }
}

void TerrainAnalysis::BuildPlanarElevationMap() {
  terrain_cloud_->clear();
  for (int x = kTerrainVoxelHalfWidth - 5;
       x <= kTerrainVoxelHalfWidth + 5; ++x) {
    for (int y = kTerrainVoxelHalfWidth - 5;
         y <= kTerrainVoxelHalfWidth + 5; ++y) {
      *terrain_cloud_ +=
          *terrain_voxel_clouds_[kTerrainVoxelWidth * x + y];
    }
  }

  planar_voxel_elevation_.fill(0.0F);
  planar_voxel_edge_.fill(0);
  planar_voxel_dynamic_obstacle_.fill(0);
  for (std::vector<float>& elevations : planar_point_elevation_) {
    elevations.clear();
  }

  for (const pcl::PointXYZI& point : terrain_cloud_->points) {
    int x = static_cast<int>(
                (point.x - vehicle_x_ + kPlanarVoxelSize / 2.0F) /
                kPlanarVoxelSize) +
            kPlanarVoxelHalfWidth;
    int y = static_cast<int>(
                (point.y - vehicle_y_ + kPlanarVoxelSize / 2.0F) /
                kPlanarVoxelSize) +
            kPlanarVoxelHalfWidth;
    if (point.x - vehicle_x_ + kPlanarVoxelSize / 2.0F < 0.0F) {
      --x;
    }
    if (point.y - vehicle_y_ + kPlanarVoxelSize / 2.0F < 0.0F) {
      --y;
    }

    if (point.z - vehicle_z_ > config_.min_relative_z &&
        point.z - vehicle_z_ < config_.max_relative_z) {
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (x + dx >= 0 && x + dx < kPlanarVoxelWidth && y + dy >= 0 &&
              y + dy < kPlanarVoxelWidth) {
            planar_point_elevation_[kPlanarVoxelWidth * (x + dx) + y + dy]
                .push_back(point.z);
          }
        }
      }
    }

    if (config_.clear_dynamic_obstacles && x >= 0 &&
        x < kPlanarVoxelWidth && y >= 0 && y < kPlanarVoxelWidth) {
      const float point_x1 = point.x - vehicle_x_;
      const float point_y1 = point.y - vehicle_y_;
      const float point_z1 = point.z - vehicle_z_;
      const float distance1 =
          std::sqrt(point_x1 * point_x1 + point_y1 * point_y1);
      const int index = kPlanarVoxelWidth * x + y;
      if (distance1 > config_.min_dynamic_obstacle_distance) {
        const float angle1 =
            std::atan2(point_z1 - config_.min_dynamic_obstacle_relative_z,
                       distance1) *
            180.0F / static_cast<float>(kPi);
        if (angle1 > config_.min_dynamic_obstacle_angle) {
          const float point_x2 =
              point_x1 * cos_vehicle_yaw_ + point_y1 * sin_vehicle_yaw_;
          const float point_y2 =
              -point_x1 * sin_vehicle_yaw_ + point_y1 * cos_vehicle_yaw_;
          const float point_z2 = point_z1;
          const float point_x3 =
              point_x2 * cos_vehicle_pitch_ - point_z2 * sin_vehicle_pitch_;
          const float point_y3 = point_y2;
          const float point_z3 =
              point_x2 * sin_vehicle_pitch_ + point_z2 * cos_vehicle_pitch_;
          const float point_x4 = point_x3;
          const float point_y4 =
              point_y3 * cos_vehicle_roll_ + point_z3 * sin_vehicle_roll_;
          const float point_z4 =
              -point_y3 * sin_vehicle_roll_ + point_z3 * cos_vehicle_roll_;
          const float distance4 =
              std::sqrt(point_x4 * point_x4 + point_y4 * point_y4);
          const float angle4 = std::atan2(point_z4, distance4) * 180.0F /
                               static_cast<float>(kPi);
          if ((angle4 > config_.min_dynamic_obstacle_vertical_fov &&
               angle4 < config_.max_dynamic_obstacle_vertical_fov) ||
              std::fabs(point_z4) <
                  config_.absolute_dynamic_obstacle_relative_z_threshold) {
            ++planar_voxel_dynamic_obstacle_[index];
          }
        }
      } else {
        planar_voxel_dynamic_obstacle_[index] +=
            config_.min_dynamic_obstacle_point_count;
      }
    }
  }
}

void TerrainAnalysis::MarkDynamicObstacles() {
  if (!config_.clear_dynamic_obstacles) {
    return;
  }
  for (const pcl::PointXYZI& point : cropped_scan_->points) {
    int x = static_cast<int>(
                (point.x - vehicle_x_ + kPlanarVoxelSize / 2.0F) /
                kPlanarVoxelSize) +
            kPlanarVoxelHalfWidth;
    int y = static_cast<int>(
                (point.y - vehicle_y_ + kPlanarVoxelSize / 2.0F) /
                kPlanarVoxelSize) +
            kPlanarVoxelHalfWidth;
    if (point.x - vehicle_x_ + kPlanarVoxelSize / 2.0F < 0.0F) {
      --x;
    }
    if (point.y - vehicle_y_ + kPlanarVoxelSize / 2.0F < 0.0F) {
      --y;
    }
    if (x < 0 || x >= kPlanarVoxelWidth || y < 0 ||
        y >= kPlanarVoxelWidth) {
      continue;
    }
    const float point_x = point.x - vehicle_x_;
    const float point_y = point.y - vehicle_y_;
    const float point_z = point.z - vehicle_z_;
    const float distance = std::sqrt(point_x * point_x + point_y * point_y);
    const float angle =
        std::atan2(point_z - config_.min_dynamic_obstacle_relative_z,
                   distance) *
        180.0F / static_cast<float>(kPi);
    if (angle > config_.min_dynamic_obstacle_angle) {
      planar_voxel_dynamic_obstacle_[kPlanarVoxelWidth * x + y] = 0;
    }
  }
}

void TerrainAnalysis::SelectPlanarElevations() {
  for (int index = 0; index < kPlanarVoxelCount; ++index) {
    std::vector<float>& elevations = planar_point_elevation_[index];
    if (elevations.empty()) {
      continue;
    }
    if (config_.use_sorting) {
      std::sort(elevations.begin(), elevations.end());
      int quantile_index =
          static_cast<int>(config_.quantile_z * elevations.size());
      quantile_index = std::max(0, std::min(
          quantile_index, static_cast<int>(elevations.size()) - 1));
      if (config_.limit_ground_lift &&
          elevations[quantile_index] >
              elevations.front() + config_.max_ground_lift) {
        planar_voxel_elevation_[index] =
            elevations.front() + config_.max_ground_lift;
      } else {
        planar_voxel_elevation_[index] = elevations[quantile_index];
      }
    } else {
      planar_voxel_elevation_[index] =
          *std::min_element(elevations.begin(), elevations.end());
    }
  }
}

void TerrainAnalysis::BuildElevationCloud() {
  elevation_cloud_->clear();
  elevation_cloud_->reserve(terrain_cloud_->size());
  for (const pcl::PointXYZI& input_point : terrain_cloud_->points) {
    if (input_point.z - vehicle_z_ <= config_.min_relative_z ||
        input_point.z - vehicle_z_ >= config_.max_relative_z) {
      continue;
    }
    int x = static_cast<int>(
                (input_point.x - vehicle_x_ + kPlanarVoxelSize / 2.0F) /
                kPlanarVoxelSize) +
            kPlanarVoxelHalfWidth;
    int y = static_cast<int>(
                (input_point.y - vehicle_y_ + kPlanarVoxelSize / 2.0F) /
                kPlanarVoxelSize) +
            kPlanarVoxelHalfWidth;
    if (input_point.x - vehicle_x_ + kPlanarVoxelSize / 2.0F < 0.0F) {
      --x;
    }
    if (input_point.y - vehicle_y_ + kPlanarVoxelSize / 2.0F < 0.0F) {
      --y;
    }
    if (x < 0 || x >= kPlanarVoxelWidth || y < 0 ||
        y >= kPlanarVoxelWidth) {
      continue;
    }
    const int index = kPlanarVoxelWidth * x + y;
    if (config_.clear_dynamic_obstacles &&
        planar_voxel_dynamic_obstacle_[index] >=
            config_.min_dynamic_obstacle_point_count) {
      continue;
    }
    float relative_elevation =
        input_point.z - planar_voxel_elevation_[index];
    if (config_.consider_drop) {
      relative_elevation = std::fabs(relative_elevation);
    }
    if (relative_elevation >= 0.0F &&
        relative_elevation < config_.vehicle_height &&
        static_cast<int>(planar_point_elevation_[index].size()) >=
            config_.min_block_point_count) {
      pcl::PointXYZI point = input_point;
      point.intensity = relative_elevation;
      elevation_cloud_->push_back(point);
    }
  }
}

void TerrainAnalysis::AddNoDataObstacles() {
  if (!config_.no_data_obstacle || no_data_initialized_ != 2) {
    return;
  }
  for (int index = 0; index < kPlanarVoxelCount; ++index) {
    if (static_cast<int>(planar_point_elevation_[index].size()) <
        config_.min_block_point_count) {
      planar_voxel_edge_[index] = 1;
    }
  }

  for (int iteration = 0;
       iteration < config_.no_data_block_skip_count; ++iteration) {
    for (int index = 0; index < kPlanarVoxelCount; ++index) {
      if (planar_voxel_edge_[index] < 1) {
        continue;
      }
      const int x = index / kPlanarVoxelWidth;
      const int y = index % kPlanarVoxelWidth;
      bool edge_voxel = false;
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (x + dx >= 0 && x + dx < kPlanarVoxelWidth && y + dy >= 0 &&
              y + dy < kPlanarVoxelWidth &&
              planar_voxel_edge_[kPlanarVoxelWidth * (x + dx) + y + dy] <
                  planar_voxel_edge_[index]) {
            edge_voxel = true;
          }
        }
      }
      if (!edge_voxel) {
        ++planar_voxel_edge_[index];
      }
    }
  }

  for (int index = 0; index < kPlanarVoxelCount; ++index) {
    if (planar_voxel_edge_[index] <= config_.no_data_block_skip_count) {
      continue;
    }
    const int x = index / kPlanarVoxelWidth;
    const int y = index % kPlanarVoxelWidth;
    pcl::PointXYZI point;
    point.x = kPlanarVoxelSize * (x - kPlanarVoxelHalfWidth) + vehicle_x_;
    point.y = kPlanarVoxelSize * (y - kPlanarVoxelHalfWidth) + vehicle_y_;
    point.z = vehicle_z_;
    point.intensity = config_.vehicle_height;
    point.x -= kPlanarVoxelSize / 4.0F;
    point.y -= kPlanarVoxelSize / 4.0F;
    elevation_cloud_->push_back(point);
    point.x += kPlanarVoxelSize / 2.0F;
    elevation_cloud_->push_back(point);
    point.y += kPlanarVoxelSize / 2.0F;
    elevation_cloud_->push_back(point);
    point.x -= kPlanarVoxelSize / 2.0F;
    elevation_cloud_->push_back(point);
  }
}

TerrainAnalysisOutput TerrainAnalysis::Process(
    double timestamp_seconds, const Cloud& registered_scan) {
  CropScan(timestamp_seconds, registered_scan);
  RollTerrainVoxels();
  AccumulateCurrentScan();
  RefreshTerrainVoxels(timestamp_seconds);
  BuildPlanarElevationMap();
  MarkDynamicObstacles();
  SelectPlanarElevations();
  BuildElevationCloud();
  AddNoDataObstacles();
  clearing_requested_ = false;

  TerrainAnalysisOutput output;
  output.ready = true;
  output.timestamp_seconds = timestamp_seconds;
  output.terrain = *elevation_cloud_;
  return output;
}

}  // namespace perception
}  // namespace jojo
