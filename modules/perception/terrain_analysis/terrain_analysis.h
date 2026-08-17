#pragma once

#include <array>
#include <vector>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "modules/common_struct/basic_msgs/Pose.h"

namespace jojo {
namespace perception {

struct TerrainAnalysisConfig {
  double scan_voxel_size{0.05};
  double decay_time{2.0};
  double no_decay_distance{4.0};
  double clearing_distance{8.0};
  bool use_sorting{true};
  double quantile_z{0.25};
  bool consider_drop{false};
  bool limit_ground_lift{false};
  double max_ground_lift{0.15};
  bool clear_dynamic_obstacles{false};
  double min_dynamic_obstacle_distance{0.3};
  double min_dynamic_obstacle_angle{0.0};
  double min_dynamic_obstacle_relative_z{-0.5};
  double absolute_dynamic_obstacle_relative_z_threshold{0.2};
  double min_dynamic_obstacle_vertical_fov{-16.0};
  double max_dynamic_obstacle_vertical_fov{16.0};
  int min_dynamic_obstacle_point_count{1};
  bool no_data_obstacle{false};
  int no_data_block_skip_count{0};
  int min_block_point_count{10};
  double vehicle_height{1.5};
  int voxel_point_update_threshold{100};
  double voxel_time_update_threshold{2.0};
  double min_relative_z{-1.5};
  double max_relative_z{0.2};
  double distance_ratio_z{0.2};
  bool use_accumulation{true};
};

struct TerrainAnalysisOutput {
  bool ready{false};
  double timestamp_seconds{0.0};
  pcl::PointCloud<pcl::PointXYZI> terrain;
};

// Middleware-independent terrain elevation analysis. PCL is the algorithm's
// native data representation; transport headers and ROS messages stay in the
// adapter layer.
class TerrainAnalysis {
 public:
  explicit TerrainAnalysis(const TerrainAnalysisConfig& config);

  void SetOdometry(const common_struct::Pose& pose);
  void RequestClearing(double distance);
  TerrainAnalysisOutput Process(
      double timestamp_seconds,
      const pcl::PointCloud<pcl::PointXYZI>& registered_scan);

 private:
  static constexpr int kTerrainVoxelWidth = 21;
  static constexpr int kTerrainVoxelHalfWidth =
      (kTerrainVoxelWidth - 1) / 2;
  static constexpr int kTerrainVoxelCount =
      kTerrainVoxelWidth * kTerrainVoxelWidth;
  static constexpr int kPlanarVoxelWidth = 51;
  static constexpr int kPlanarVoxelHalfWidth =
      (kPlanarVoxelWidth - 1) / 2;
  static constexpr int kPlanarVoxelCount =
      kPlanarVoxelWidth * kPlanarVoxelWidth;
  static constexpr float kTerrainVoxelSize = 1.0F;
  static constexpr float kPlanarVoxelSize = 0.2F;

  using Cloud = pcl::PointCloud<pcl::PointXYZI>;
  using CloudPtr = Cloud::Ptr;

  void RollTerrainVoxels();
  void CropScan(double timestamp_seconds, const Cloud& registered_scan);
  void AccumulateCurrentScan();
  void RefreshTerrainVoxels(double timestamp_seconds);
  void BuildPlanarElevationMap();
  void MarkDynamicObstacles();
  void SelectPlanarElevations();
  void BuildElevationCloud();
  void AddNoDataObstacles();

  TerrainAnalysisConfig config_;
  std::array<CloudPtr, kTerrainVoxelCount> terrain_voxel_clouds_;
  std::array<int, kTerrainVoxelCount> terrain_voxel_update_count_{};
  std::array<float, kTerrainVoxelCount> terrain_voxel_update_time_{};
  std::array<float, kPlanarVoxelCount> planar_voxel_elevation_{};
  std::array<int, kPlanarVoxelCount> planar_voxel_edge_{};
  std::array<int, kPlanarVoxelCount> planar_voxel_dynamic_obstacle_{};
  std::array<std::vector<float>, kPlanarVoxelCount> planar_point_elevation_;

  CloudPtr cropped_scan_;
  CloudPtr downsampled_scan_;
  CloudPtr terrain_cloud_;
  CloudPtr elevation_cloud_;
  pcl::VoxelGrid<pcl::PointXYZI> downsample_filter_;

  bool system_initialized_{false};
  double system_initial_time_{0.0};
  double scan_time_{0.0};
  bool clearing_requested_{false};
  double clearing_distance_{8.0};
  int no_data_initialized_{0};

  int terrain_voxel_shift_x_{0};
  int terrain_voxel_shift_y_{0};
  float vehicle_roll_{0.0F};
  float vehicle_pitch_{0.0F};
  float vehicle_yaw_{0.0F};
  float vehicle_x_{0.0F};
  float vehicle_y_{0.0F};
  float vehicle_z_{0.0F};
  float recorded_vehicle_x_{0.0F};
  float recorded_vehicle_y_{0.0F};
  float sin_vehicle_roll_{0.0F};
  float cos_vehicle_roll_{1.0F};
  float sin_vehicle_pitch_{0.0F};
  float cos_vehicle_pitch_{1.0F};
  float sin_vehicle_yaw_{0.0F};
  float cos_vehicle_yaw_{1.0F};
};

}  // namespace perception
}  // namespace jojo
