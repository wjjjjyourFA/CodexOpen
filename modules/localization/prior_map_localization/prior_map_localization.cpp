#include "modules/localization/prior_map_localization/prior_map_localization.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <filesystem>
#include <mutex>
#include <stdexcept>
#include <utility>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <yaml-cpp/yaml.h>

#include "modules/localization/prior_map_localization/src/ImuProcess.h"
#include "modules/localization/prior_map_localization/src/MapProcess.h"
#include "modules/localization/prior_map_localization/src/PreProcess.h"

namespace jojo {
namespace localization {
namespace {

template <typename T>
T ReadValue(const YAML::Node& parent, const char* key,
            const T& default_value) {
  const YAML::Node value = parent[key];
  return value ? value.as<T>() : default_value;
}

void RequireSize(const std::vector<double>& values, std::size_t size,
                 const char* name) {
  if (values.size() != size) {
    throw std::invalid_argument(std::string(name) + " must contain " +
                                std::to_string(size) + " values");
  }
}

state_group StateFromXyzRpy(const std::vector<double>& xyz_rpy,
                            const std::vector<double>& map_center) {
  state_group state;
  state.pos = V3D(xyz_rpy[0] - map_center[0],
                  xyz_rpy[1] - map_center[1],
                  xyz_rpy[2] - map_center[2]);
  state.rot = EulerToSO3(xyz_rpy[3] * DEG_2_RAD,
                         xyz_rpy[4] * DEG_2_RAD,
                         xyz_rpy[5] * DEG_2_RAD);
  return state;
}

}  // namespace

PriorMapLocalizationConfig PriorMapLocalizationConfig::LoadFromFile(
    const std::string& path) {
  const YAML::Node root = YAML::LoadFile(path);
  PriorMapLocalizationConfig config;

  const YAML::Node common = root["common"];
  config.lidar_type = ReadValue(common, "lidar_type", config.lidar_type);
  config.point_filter_num =
      ReadValue(common, "point_filter_num", config.point_filter_num);

  const YAML::Node init = root["init"];
  config.initialization_method = static_cast<InitializationMethod>(
      ReadValue(init, "init_method",
                static_cast<int>(config.initialization_method)));
  config.initial_pose =
      ReadValue(init, "init_pose", config.initial_pose);
  config.map_center = ReadValue(init, "map_center", config.map_center);
  config.refine_initial_pose_twice = ReadValue(
      init, "b_twice_init", config.refine_initial_pose_twice);
  config.initialization_match_radius = ReadValue(
      init, "match_radius", config.initialization_match_radius);

  const YAML::Node threshold = root["threshold"];
  config.relocation_position_threshold = ReadValue(
      threshold, "threshold_position",
      config.relocation_position_threshold);
  const double rotation_degrees = ReadValue(
      threshold, "threshold_rotation",
      config.relocation_rotation_threshold * 180.0 / M_PI);
  config.relocation_rotation_threshold = rotation_degrees * M_PI / 180.0;

  const YAML::Node preprocess = root["preprocess"];
  config.feature_enabled = ReadValue(
      preprocess, "feature_extract_enable", config.feature_enabled);
  config.blind_distance =
      ReadValue(preprocess, "blind", config.blind_distance);
  config.scan_lines =
      ReadValue(preprocess, "scan_line", config.scan_lines);
  config.timestamp_unit =
      ReadValue(preprocess, "timestamp_unit", config.timestamp_unit);
  config.scan_rate =
      ReadValue(preprocess, "scan_rate", config.scan_rate);
  config.body_x_min = ReadValue(preprocess, "x_min", config.body_x_min);
  config.body_x_max = ReadValue(preprocess, "x_max", config.body_x_max);
  config.body_y_min = ReadValue(preprocess, "y_min", config.body_y_min);
  config.body_y_max = ReadValue(preprocess, "y_max", config.body_y_max);
  config.body_z_min = ReadValue(preprocess, "z_min", config.body_z_min);
  config.body_z_max = ReadValue(preprocess, "z_max", config.body_z_max);
  config.maximum_range =
      ReadValue(preprocess, "maxrange", config.maximum_range);

  const YAML::Node imu = root["imuprocess"];
  config.gyroscope_covariance =
      ReadValue(imu, "gyr_cov", config.gyroscope_covariance);
  config.acceleration_covariance =
      ReadValue(imu, "acc_cov", config.acceleration_covariance);
  config.gyroscope_bias_covariance =
      ReadValue(imu, "b_gyr_cov", config.gyroscope_bias_covariance);
  config.acceleration_bias_covariance =
      ReadValue(imu, "b_acc_cov", config.acceleration_bias_covariance);
  config.extrinsic_translation = ReadValue(
      imu, "extrinsic_T", config.extrinsic_translation);
  config.extrinsic_rotation = ReadValue(
      imu, "extrinsic_R", config.extrinsic_rotation);

  const YAML::Node map = root["mapprocess"];
  config.estimate_extrinsic = ReadValue(
      map, "extrinsic_est_en", config.estimate_extrinsic);
  config.detection_range =
      ReadValue(map, "det_range", config.detection_range);
  config.cube_side_length = ReadValue(
      map, "cube_side_length", config.cube_side_length);
  config.surface_filter_size = ReadValue(
      map, "filter_size_surf", config.surface_filter_size);
  config.map_filter_size = ReadValue(
      map, "filter_size_map", config.map_filter_size);
  config.maximum_iterations = ReadValue(
      map, "max_iteration", config.maximum_iterations);

  const YAML::Node offset = root["offset"];
  config.z_offset = ReadValue(offset, "z_offset", config.z_offset);
  config.Validate();
  return config;
}

void PriorMapLocalizationConfig::Validate() const {
  if (lidar_type != AVIA && lidar_type != RS128 && lidar_type != VELO16) {
    throw std::invalid_argument("unsupported prior-map lidar_type");
  }
  if (point_filter_num <= 0 || scan_lines <= 0 || scan_rate <= 0) {
    throw std::invalid_argument(
        "point_filter_num, scan_lines and scan_rate must be positive");
  }
  if (!(body_x_min < body_x_max && body_y_min < body_y_max &&
        body_z_min < body_z_max) || maximum_range <= 0.0 ||
      blind_distance < 0.0) {
    throw std::invalid_argument("invalid preprocessing region");
  }
  RequireSize(initial_pose, 6U, "init_pose");
  RequireSize(map_center, 3U, "map_center");
  RequireSize(extrinsic_translation, 3U, "extrinsic_T");
  RequireSize(extrinsic_rotation, 9U, "extrinsic_R");
  if (surface_filter_size <= 0.0 || map_filter_size <= 0.0 ||
      maximum_iterations <= 0 || initialization_match_radius <= 0.0) {
    throw std::invalid_argument("invalid map processing configuration");
  }
  const int method = static_cast<int>(initialization_method);
  if (method < static_cast<int>(InitializationMethod::kGnss) ||
      method > static_cast<int>(InitializationMethod::kExternalPose)) {
    throw std::invalid_argument("init_method must be in [1, 5]");
  }
}

class PriorMapLocalization::Impl {
 public:
  Impl(const PriorMapLocalizationConfig& config,
       const std::string& map_path, const std::string& log_directory)
      : config_(config) {
    config_.Validate();
    if (!log_directory.empty()) {
      std::filesystem::create_directories(log_directory);
      SetLocalizationDebugDirectory(log_directory);
    }

    preprocess_.reset(new PreProcess());
    imu_process_.reset(new ImuProcess());
    map_process_.reset(new MapProcess());
    ConfigureAlgorithms();
    map_process_->InitConfig(std::string());
    map_process_->SetMap(map_path);

    cloud_frame_.reset(new PointCloudXYZI());
    filtered_frame_.reset(new PointCloudXYZI());
    if (config_.initialization_method == InitializationMethod::kConfig) {
      external_pose_.state =
          StateFromXyzRpy(config_.initial_pose, config_.map_center);
      external_pose_.quality_valid = true;
      has_external_pose_ = true;
    }
  }

  void ConfigureAlgorithms() {
    preprocess_->lidar_type = config_.lidar_type;
    preprocess_->blind = config_.blind_distance;
    preprocess_->N_SCANS = config_.scan_lines;
    preprocess_->time_unit = config_.timestamp_unit;
    preprocess_->SCAN_RATE = config_.scan_rate;
    preprocess_->feature_enabled = config_.feature_enabled;
    preprocess_->point_filter_num = config_.point_filter_num;
    preprocess_->SetValidRegion(
        config_.body_x_min, config_.body_x_max,
        config_.body_y_min, config_.body_y_max,
        config_.body_z_min, config_.body_z_max,
        config_.maximum_range);

    const V3D translation(config_.extrinsic_translation[0],
                          config_.extrinsic_translation[1],
                          config_.extrinsic_translation[2]);
    M3D rotation;
    rotation << config_.extrinsic_rotation[0],
                config_.extrinsic_rotation[1],
                config_.extrinsic_rotation[2],
                config_.extrinsic_rotation[3],
                config_.extrinsic_rotation[4],
                config_.extrinsic_rotation[5],
                config_.extrinsic_rotation[6],
                config_.extrinsic_rotation[7],
                config_.extrinsic_rotation[8];
    imu_process_->set_extrinsic(translation, rotation);
    imu_process_->set_gyr_cov(V3D::Constant(config_.gyroscope_covariance));
    imu_process_->set_acc_cov(V3D::Constant(config_.acceleration_covariance));
    imu_process_->set_gyr_bias_cov(
        V3D::Constant(config_.gyroscope_bias_covariance));
    imu_process_->set_acc_bias_cov(
        V3D::Constant(config_.acceleration_bias_covariance));

    MapProcess::extrinsic_est_en = config_.estimate_extrinsic;
    map_process_->DET_RANGE = config_.detection_range;
    map_process_->cube_len = config_.cube_side_length;
    map_process_->filter_size_surf_min = config_.surface_filter_size;
    map_process_->filter_size_map_min = config_.map_filter_size;
    map_process_->NUM_MAX_ITERATIONS = config_.maximum_iterations;
  }

  template <typename CloudT>
  void PushStandardCloud(double timestamp, const CloudT& cloud) {
    std::lock_guard<std::mutex> lock(mutex_);
    PointCloudXYZI::Ptr processed(new PointCloudXYZI());
    preprocess_->Process(cloud, processed);
    PushProcessedCloud(timestamp, std::move(processed));
  }

  void PushLivoxCloud(double timestamp, const LivoxPointCloud& cloud) {
    std::lock_guard<std::mutex> lock(mutex_);
    PointCloudXYZI::Ptr processed(new PointCloudXYZI());
    preprocess_->Process(cloud, processed);
    PushProcessedCloud(timestamp, std::move(processed));
  }

  void PushProcessedCloud(double timestamp, PointCloudXYZI::Ptr cloud) {
    if (timestamp < last_lidar_timestamp_) {
      lidar_buffer_.clear();
      lidar_time_buffer_.clear();
      lidar_pushed_ = false;
      LOG(WARNING) << "lidar timestamp looped back; buffer cleared";
    }
    last_lidar_timestamp_ = timestamp;
    lidar_buffer_.push_back(std::move(cloud));
    lidar_time_buffer_.push_back(timestamp);
  }

  void PushImu(const ImuDataConstPtr& imu) {
    if (!imu) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (imu->timestamp < last_imu_timestamp_) {
      imu_buffer_.clear();
      LOG(WARNING) << "IMU timestamp looped back; buffer cleared";
    }
    last_imu_timestamp_ = imu->timestamp;
    imu_buffer_.push_back(imu);
  }

  void SetExternalPose(const ExternalPose& pose) {
    if (!pose.quality_valid) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    external_pose_ = pose;
    if (external_pose_.state.rot.norm() < 1.0e-9) {
      external_pose_.state.rot = Eigen::Quaterniond::Identity();
    } else {
      external_pose_.state.rot.normalize();
    }
    external_pose_.state.pos.z() += config_.z_offset;
    has_external_pose_ = true;
  }

  bool SyncPackages(MeasureGroup* measurement) {
    if (lidar_buffer_.empty() || imu_buffer_.empty()) {
      return false;
    }
    if (!lidar_pushed_) {
      measurement->lidar = lidar_buffer_.front();
      measurement->lidar_bag_time = lidar_time_buffer_.front();
      if (measurement->lidar->size() <= 1U) {
        lidar_end_time_ =
            measurement->lidar_bag_time + lidar_mean_scan_time_;
      } else {
        const double measured_duration =
            measurement->lidar->points.back().curvature / 1000.0;
        if (measured_duration < 0.5 * lidar_mean_scan_time_) {
          lidar_end_time_ =
              measurement->lidar_bag_time + lidar_mean_scan_time_;
        } else {
          ++scan_count_;
          lidar_end_time_ = measurement->lidar_bag_time + measured_duration;
          lidar_mean_scan_time_ +=
              (measured_duration - lidar_mean_scan_time_) / scan_count_;
        }
      }
      measurement->lidar_end_time = lidar_end_time_;
      lidar_pushed_ = true;
    }
    if (last_imu_timestamp_ < lidar_end_time_) {
      return false;
    }

    measurement->imu.clear();
    while (!imu_buffer_.empty() &&
           imu_buffer_.front()->timestamp <= lidar_end_time_) {
      measurement->imu.push_back(imu_buffer_.front());
      imu_buffer_.pop_front();
    }
    if (measurement->imu.empty()) {
      return false;
    }
    lidar_buffer_.pop_front();
    lidar_time_buffer_.pop_front();
    lidar_pushed_ = false;
    return true;
  }

  bool RefineInitialPose(state_group* state,
                         const PointCloudXYZI::Ptr& frame) {
    Eigen::Affine3d imu_from_lidar = Eigen::Affine3d::Identity();
    imu_from_lidar.rotate(imu_process_->Lidar_R_wrt_IMU);
    imu_from_lidar.translation() = imu_process_->Lidar_T_wrt_IMU;

    Eigen::Affine3d world_from_imu = Eigen::Affine3d::Identity();
    world_from_imu.rotate(state->rot.matrix());
    world_from_imu.translation() = state->pos;
    const Eigen::Affine3f imu_from_world =
        world_from_imu.inverse().cast<float>();
    const Eigen::Affine3f imu_from_lidar_float =
        imu_from_lidar.cast<float>();

    pcl::PointCloud<pcl::PointXYZ>::Ptr source(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr target(
        new pcl::PointCloud<pcl::PointXYZ>());
    source->reserve(frame->size());
    target->reserve(map_process_->cloud_map->size());
    pcl::PointXYZ input;
    const double radius_squared =
        config_.initialization_match_radius *
        config_.initialization_match_radius;
    for (const PointType& point : *frame) {
      input.x = point.x;
      input.y = point.y;
      input.z = point.z;
      source->push_back(
          pcl::transformPoint(input, imu_from_lidar_float));
    }
    for (const PointType& point : *map_process_->cloud_map) {
      input.x = point.x;
      input.y = point.y;
      input.z = point.z;
      const pcl::PointXYZ transformed =
          pcl::transformPoint(input, imu_from_world);
      if (transformed.x * transformed.x +
          transformed.y * transformed.y < radius_squared) {
        target->push_back(transformed);
      }
    }
    if (source->size() < 10U || target->size() < 10U) {
      LOG(ERROR) << "insufficient points for initial-pose refinement";
      return false;
    }

    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setLeafSize(0.1F, 0.1F, 0.1F);
    voxel_filter.setInputCloud(source);
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(
        new pcl::PointCloud<pcl::PointXYZ>());
    voxel_filter.filter(*downsampled);
    source = downsampled;

    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    ndt.setTransformationEpsilon(0.00001);
    ndt.setStepSize(0.1);
    ndt.setResolution(1.0);
    ndt.setMaximumIterations(1000);
    ndt.setInputSource(source);
    ndt.setInputTarget(target);
    pcl::PointCloud<pcl::PointXYZ> ndt_output;
    ndt.align(ndt_output);
    if (!ndt.hasConverged()) {
      LOG(ERROR) << "initial NDT registration did not converge";
      return false;
    }
    Eigen::Matrix4f correction = ndt.getFinalTransformation();

    if (config_.refine_initial_pose_twice) {
      pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
      icp.setTransformationEpsilon(0.00001);
      icp.setMaxCorrespondenceDistance(0.1);
      icp.setMaximumIterations(100);
      icp.setInputSource(source);
      icp.setInputTarget(target);
      pcl::PointCloud<pcl::PointXYZ> icp_output;
      icp.align(icp_output, correction);
      if (icp.hasConverged()) {
        correction = icp.getFinalTransformation();
      } else {
        LOG(WARNING) << "initial ICP refinement did not converge; using NDT";
      }
    }

    world_from_imu =
        world_from_imu * Eigen::Affine3d(correction.cast<double>());
    state->rot = Eigen::Quaterniond(world_from_imu.rotation()).normalized();
    state->pos = world_from_imu.translation();
    return true;
  }

  std::array<double, 36> PoseCovariance() const {
    std::array<double, 36> result{};
    const auto covariance = map_process_->kf.get_P();
    for (int row = 0; row < 6; ++row) {
      const int source_row = row < 3 ? row + 3 : row - 3;
      result[row * 6 + 0] = covariance(source_row, 3);
      result[row * 6 + 1] = covariance(source_row, 4);
      result[row * 6 + 2] = covariance(source_row, 5);
      result[row * 6 + 3] = covariance(source_row, 0);
      result[row * 6 + 4] = covariance(source_row, 1);
      result[row * 6 + 5] = covariance(source_row, 2);
    }
    return result;
  }

  PriorMapLocalizationOutput Step() {
    std::lock_guard<std::mutex> lock(mutex_);
    PriorMapLocalizationOutput output;
    if (!SyncPackages(&measurement_)) {
      return output;
    }
    output.timestamp = measurement_.lidar_end_time;

    if (!pose_initialized_) {
      if (!has_external_pose_) {
        output.status = LocalizationStepStatus::kWaitingForInitialPose;
        return output;
      }
      state_group initial_state = external_pose_.state;
      if (!RefineInitialPose(&initial_state, measurement_.lidar)) {
        output.status = LocalizationStepStatus::kFrameRejected;
        return output;
      }
      map_process_->SetPose(initial_state);
      imu_process_->first_lidar_time = measurement_.lidar_bag_time;
      pose_initialized_ = true;
      output.status = LocalizationStepStatus::kInitialPoseAccepted;
      return output;
    }

    if (config_.initialization_method == InitializationMethod::kFusion &&
        has_external_pose_) {
      state_group current;
      map_process_->GetPose(current);
      const double position_error =
          (external_pose_.state.pos - current.pos).norm();
      const double rotation_error =
          external_pose_.state.rot.angularDistance(current.rot);
      if (position_error > config_.relocation_position_threshold ||
          rotation_error > config_.relocation_rotation_threshold) {
        map_process_->SetPose(external_pose_.state);
      }
    }

    cloud_frame_->clear();
    if (!imu_process_->Process(measurement_, map_process_->kf,
                               cloud_frame_)) {
      output.status = LocalizationStepStatus::kInitializingImu;
      return output;
    }
    filtered_frame_->clear();
    filtered_frame_->reserve(cloud_frame_->size() /
                             config_.point_filter_num + 1U);
    for (std::size_t index = 0; index < cloud_frame_->size();
         index += static_cast<std::size_t>(config_.point_filter_num)) {
      filtered_frame_->push_back(cloud_frame_->points[index]);
    }
    if (!map_process_->Process(filtered_frame_)) {
      output.status = LocalizationStepStatus::kFrameRejected;
      return output;
    }

    output.status = LocalizationStepStatus::kPoseUpdated;
    output.pose_ready = true;
    map_process_->GetPose(output.pose);
    output.velocity = map_process_->state_point.vel;
    output.pose_covariance = PoseCovariance();
    output.registered_scan = map_process_->feats_undistort_world;
    return output;
  }

  PriorMapLocalizationConfig config_;
  std::shared_ptr<PreProcess> preprocess_;
  std::shared_ptr<ImuProcess> imu_process_;
  std::shared_ptr<MapProcess> map_process_;
  PointCloudXYZI::Ptr cloud_frame_;
  PointCloudXYZI::Ptr filtered_frame_;

  mutable std::mutex mutex_;
  std::deque<double> lidar_time_buffer_;
  std::deque<PointCloudXYZI::Ptr> lidar_buffer_;
  std::deque<ImuDataConstPtr> imu_buffer_;
  MeasureGroup measurement_;
  ExternalPose external_pose_;
  bool has_external_pose_{false};
  bool pose_initialized_{false};
  bool lidar_pushed_{false};
  double last_lidar_timestamp_{-1.0};
  double last_imu_timestamp_{-1.0};
  double lidar_end_time_{0.0};
  double lidar_mean_scan_time_{0.0};
  int scan_count_{0};
};

PriorMapLocalization::PriorMapLocalization(
    const PriorMapLocalizationConfig& config, const std::string& map_path,
    const std::string& log_directory)
    : impl_(new Impl(config, map_path, log_directory)) {}

PriorMapLocalization::~PriorMapLocalization() = default;

int PriorMapLocalization::lidar_type() const {
  return impl_->config_.lidar_type;
}

const std::vector<double>& PriorMapLocalization::map_center() const {
  return impl_->config_.map_center;
}

double PriorMapLocalization::z_offset() const {
  return impl_->config_.z_offset;
}

PointCloudXYZI::ConstPtr PriorMapLocalization::map() const {
  return impl_->map_process_->cloud_map;
}

void PriorMapLocalization::PushLivoxCloud(
    double timestamp, const LivoxPointCloud& cloud) {
  impl_->PushLivoxCloud(timestamp, cloud);
}

void PriorMapLocalization::PushRsCloud(
    double timestamp, const pcl::PointCloud<rs_lidar::Point>& cloud) {
  impl_->PushStandardCloud(timestamp, cloud);
}

void PriorMapLocalization::PushVelodyneCloud(
    double timestamp, const pcl::PointCloud<velodyne_lidar::Point>& cloud) {
  impl_->PushStandardCloud(timestamp, cloud);
}

void PriorMapLocalization::PushImu(const ImuDataConstPtr& imu) {
  impl_->PushImu(imu);
}

void PriorMapLocalization::SetExternalPose(const ExternalPose& pose) {
  impl_->SetExternalPose(pose);
}

PriorMapLocalizationOutput PriorMapLocalization::Step() {
  return impl_->Step();
}

}  // namespace localization
}  // namespace jojo
