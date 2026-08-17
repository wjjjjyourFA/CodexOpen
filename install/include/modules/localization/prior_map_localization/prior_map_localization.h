#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "modules/localization/prior_map_localization/include/global_define.h"

namespace jojo {
namespace localization {

enum class InitializationMethod {
  kGnss = 1,
  kManual = 2,
  kConfig = 3,
  kFusion = 4,
  kExternalPose = 5,
};

struct PriorMapLocalizationConfig {
  int lidar_type{AVIA};
  int point_filter_num{5};
  InitializationMethod initialization_method{
      InitializationMethod::kExternalPose};
  std::vector<double> initial_pose{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> map_center{0.0, 0.0, 0.0};
  bool refine_initial_pose_twice{true};
  double initialization_match_radius{80.0};
  double relocation_position_threshold{15.0};
  double relocation_rotation_threshold{10.0 * M_PI / 180.0};
  double z_offset{0.0};

  bool feature_enabled{false};
  double blind_distance{0.5};
  int scan_lines{32};
  int timestamp_unit{SEC};
  int scan_rate{10};
  double body_x_min{-0.7};
  double body_x_max{0.7};
  double body_y_min{-0.4};
  double body_y_max{0.4};
  double body_z_min{-0.6};
  double body_z_max{0.5};
  double maximum_range{15.0};

  double gyroscope_covariance{0.1};
  double acceleration_covariance{0.1};
  double gyroscope_bias_covariance{0.0001};
  double acceleration_bias_covariance{0.0001};
  std::vector<double> extrinsic_translation{0.0, 0.0, 0.0};
  std::vector<double> extrinsic_rotation{1.0, 0.0, 0.0,
                                         0.0, 1.0, 0.0,
                                         0.0, 0.0, 1.0};

  bool estimate_extrinsic{false};
  float detection_range{100.0F};
  double cube_side_length{1000.0};
  double surface_filter_size{0.5};
  double map_filter_size{0.5};
  int maximum_iterations{5};

  static PriorMapLocalizationConfig LoadFromFile(const std::string& path);
  void Validate() const;
};

struct ExternalPose {
  double timestamp{0.0};
  state_group state;
  bool quality_valid{true};
};

enum class LocalizationStepStatus {
  kWaitingForData,
  kWaitingForInitialPose,
  kInitializingImu,
  kInitialPoseAccepted,
  kPoseUpdated,
  kFrameRejected,
};

struct PriorMapLocalizationOutput {
  LocalizationStepStatus status{LocalizationStepStatus::kWaitingForData};
  double timestamp{0.0};
  bool pose_ready{false};
  state_group pose;
  V3D velocity{V3D::Zero()};
  std::array<double, 36> pose_covariance{};
  PointCloudXYZI::ConstPtr registered_scan;
};

class PriorMapLocalization {
 public:
  PriorMapLocalization(const PriorMapLocalizationConfig& config,
                       const std::string& map_path,
                       const std::string& log_directory);
  ~PriorMapLocalization();

  PriorMapLocalization(const PriorMapLocalization&) = delete;
  PriorMapLocalization& operator=(const PriorMapLocalization&) = delete;

  int lidar_type() const;
  const std::vector<double>& map_center() const;
  double z_offset() const;
  PointCloudXYZI::ConstPtr map() const;

  void PushLivoxCloud(double timestamp, const LivoxPointCloud& cloud);
  void PushRsCloud(double timestamp,
                   const pcl::PointCloud<rs_lidar::Point>& cloud);
  void PushVelodyneCloud(
      double timestamp,
      const pcl::PointCloud<velodyne_lidar::Point>& cloud);
  void PushImu(const ImuDataConstPtr& imu);
  void SetExternalPose(const ExternalPose& pose);

  PriorMapLocalizationOutput Step();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace localization
}  // namespace jojo
