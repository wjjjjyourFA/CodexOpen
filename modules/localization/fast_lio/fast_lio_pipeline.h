#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "modules/localization/fast_lio/include/common_lib.h"
#include "modules/localization/fast_lio/include/preprocess.h"

namespace jojo {
namespace localization {

struct FastLioLivoxPoint {
  std::uint32_t offset_time{0};
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  std::uint8_t reflectivity{0};
  std::uint8_t tag{0};
  std::uint8_t line{0};
};

using FastLioLivoxCloud = std::vector<FastLioLivoxPoint>;

struct FastLioConfig {
  int lidar_type{AVIA};
  int point_filter_num{3};
  int scan_line_count{4};
  double lidar_to_imu_time_offset{0.0};
  double blind_distance{0.5};
  double inner_x_min{-0.7};
  double inner_x_max{0.7};
  double inner_y_min{-0.4};
  double inner_y_max{0.4};
  double inner_z_min{-0.6};
  double inner_z_max{0.5};
  double maximum_range{15.0};

  int maximum_iterations{3};
  float surface_filter_size{0.2F};
  float map_filter_size{0.2F};
  double cube_side_length{1000.0};
  float detection_range{100.0F};
  double gyroscope_covariance{0.1};
  double acceleration_covariance{0.1};
  double gyroscope_bias_covariance{0.0001};
  double acceleration_bias_covariance{0.0001};
  bool estimate_extrinsic{false};
  std::array<double, 3> extrinsic_translation{{-0.011, -0.02329, 0.04412}};
  std::array<double, 9> extrinsic_rotation{{1.0, 0.0, 0.0,
                                             0.0, 1.0, 0.0,
                                             0.0, 0.0, 1.0}};

  void Validate() const;
};

enum class FastLioStepStatus {
  kWaitingForData,
  kInitializing,
  kPoseUpdated,
};

struct FastLioOutput {
  FastLioStepStatus status{FastLioStepStatus::kWaitingForData};
  bool pose_ready{false};
  double timestamp{0.0};
  fastlio::OdomData pose;
  fastlio::V3D velocity{fastlio::V3D::Zero()};
  std::array<double, 36> pose_covariance{};
  fastlio::PointCloudXYZI::ConstPtr registered_scan;
};

class FastLioPipeline {
 public:
  FastLioPipeline(const FastLioConfig& config,
                  const std::string& output_root);
  ~FastLioPipeline();

  FastLioPipeline(const FastLioPipeline&) = delete;
  FastLioPipeline& operator=(const FastLioPipeline&) = delete;

  void PushLivoxCloud(double timestamp, const FastLioLivoxCloud& cloud);
  void PushImu(const fastlio::ImuData& imu);
  FastLioOutput Step(bool include_registered_scan,
                     bool dense_registered_scan);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace localization
}  // namespace jojo
