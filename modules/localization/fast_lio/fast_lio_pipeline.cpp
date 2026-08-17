#include "modules/localization/fast_lio/fast_lio_pipeline.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <filesystem>
#include <mutex>
#include <stdexcept>

#include "modules/localization/fast_lio/config/runtime_config.h"
#include "modules/localization/fast_lio/config/static_config.h"
#include "modules/localization/fast_lio/lidar_odometry.h"

namespace jojo {
namespace localization {

void FastLioConfig::Validate() const {
  if (lidar_type != AVIA) {
    throw std::invalid_argument(
        "robot-dog Fast-LIO currently requires Livox AVIA input");
  }
  if (point_filter_num <= 0 || scan_line_count <= 0 ||
      blind_distance < 0.0 || maximum_range <= 0.0 ||
      !(inner_x_min < inner_x_max && inner_y_min < inner_y_max &&
        inner_z_min < inner_z_max)) {
    throw std::invalid_argument("invalid Fast-LIO preprocessing config");
  }
  if (maximum_iterations <= 0 || surface_filter_size <= 0.0F ||
      map_filter_size <= 0.0F || cube_side_length <= 0.0 ||
      detection_range <= 0.0F) {
    throw std::invalid_argument("invalid Fast-LIO mapping config");
  }
}

class FastLioPipeline::Impl {
 public:
  class CoreOdometry final : public fastlio::LidarOdometry {
   public:
    bool HasPose() const { return pose_inited; }
    double LidarEndTime() const { return lidar_end_time; }
    const fastlio::OdomData& Pose() const { return o_pose; }
    const fastlio::V3D& Velocity() const { return state_point.vel; }
    decltype(auto) Covariance() const { return kf.get_P(); }

    fastlio::PointCloudXYZI::Ptr RegisteredScan(bool dense) {
      const fastlio::PointCloudXYZI::Ptr& source =
          dense ? feats_undistort : feats_down_body;
      fastlio::PointCloudXYZI::Ptr world(
          new fastlio::PointCloudXYZI(source->size(), 1));
      for (std::size_t index = 0; index < source->size(); ++index) {
        pointBodyToWorld(&source->points[index], &world->points[index]);
      }
      world->width = world->size();
      world->height = 1;
      world->is_dense = source->is_dense;
      return world;
    }
  };

  Impl(const FastLioConfig& config, const std::string& output_root)
      : config_(config) {
    config_.Validate();
    if (output_root.empty()) {
      throw std::invalid_argument("Fast-LIO output_root is empty");
    }
    std::filesystem::create_directories(output_root);

    std::shared_ptr<StaticConfig> static_config(new StaticConfig());
    static_config->lidar_type = config_.lidar_type;
    static_config->point_filter_num = config_.point_filter_num;
    static_config->feature_enabled = false;
    static_config->blind = config_.blind_distance;
    static_config->N_SCANS = config_.scan_line_count;
    static_config->time_unit = NS;
    static_config->SCAN_RATE = 10;
    static_config->NUM_MAX_ITERATIONS = config_.maximum_iterations;
    static_config->filter_size_surf_min = config_.surface_filter_size;
    static_config->filter_size_map_min = config_.map_filter_size;
    static_config->cube_len = config_.cube_side_length;
    static_config->DET_RANGE = config_.detection_range;
    static_config->gyr_cov = config_.gyroscope_covariance;
    static_config->acc_cov = config_.acceleration_covariance;
    static_config->b_gyr_cov = config_.gyroscope_bias_covariance;
    static_config->b_acc_cov = config_.acceleration_bias_covariance;
    static_config->extrinsic_est_en = config_.estimate_extrinsic;

    std::shared_ptr<RuntimeConfig> runtime_config(new RuntimeConfig());
    runtime_config->root_path = output_root;
    runtime_config->file_name = "online";
    runtime_config->b_save_pcd = false;
    runtime_config->b_only_times = false;

    Eigen::Matrix4f lidar_to_imu = Eigen::Matrix4f::Identity();
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        lidar_to_imu(row, column) = static_cast<float>(
            config_.extrinsic_rotation[row * 3 + column]);
      }
      lidar_to_imu(row, 3) =
          static_cast<float>(config_.extrinsic_translation[row]);
    }

    odometry_.reset(new CoreOdometry());
    odometry_->SetGravityImuExtrinsicMatrix(Eigen::Matrix4f::Identity());
    odometry_->SetExtrinsicMatrix(lidar_to_imu);
    odometry_->Init(runtime_config, static_config);
  }

  ~Impl() {
    if (odometry_) {
      odometry_->Close();
    }
  }

  bool IsValidPoint(double x, double y, double z) const {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      return false;
    }
    const double squared_range = x * x + y * y + z * z;
    if (squared_range <= config_.blind_distance * config_.blind_distance ||
        squared_range >= config_.maximum_range * config_.maximum_range) {
      return false;
    }
    const bool inside_robot =
        x > config_.inner_x_min && x < config_.inner_x_max &&
        y > config_.inner_y_min && y < config_.inner_y_max &&
        z > config_.inner_z_min && z < config_.inner_z_max;
    return !inside_robot;
  }

  fastlio::PointCloudXYZI::Ptr ConvertLivoxCloud(
      const FastLioLivoxCloud& cloud) const {
    fastlio::PointCloudXYZI::Ptr converted(
        new fastlio::PointCloudXYZI());
    converted->reserve(cloud.size() /
                       static_cast<std::size_t>(config_.point_filter_num));
    std::size_t valid_count = 0;
    for (std::size_t index = 1; index < cloud.size(); ++index) {
      const FastLioLivoxPoint& source = cloud[index];
      const FastLioLivoxPoint& previous = cloud[index - 1];
      if (source.line >= config_.scan_line_count ||
          ((source.tag & 0x30) != 0x10 &&
           (source.tag & 0x30) != 0x00)) {
        continue;
      }
      ++valid_count;
      if (valid_count %
              static_cast<std::size_t>(config_.point_filter_num) != 0U ||
          !IsValidPoint(source.x, source.y, source.z)) {
        continue;
      }
      if (std::abs(source.x - previous.x) <= 1.0e-7 &&
          std::abs(source.y - previous.y) <= 1.0e-7 &&
          std::abs(source.z - previous.z) <= 1.0e-7) {
        continue;
      }
      fastlio::PointType point;
      point.x = source.x;
      point.y = source.y;
      point.z = source.z;
      point.intensity = source.reflectivity;
      point.normal_x = 0.0F;
      point.normal_y = 0.0F;
      point.normal_z = 0.0F;
      point.curvature = source.offset_time / 1000000.0F;
      converted->push_back(point);
    }
    converted->width = converted->size();
    converted->height = 1;
    converted->is_dense = true;
    return converted;
  }

  void PushLivoxCloud(double timestamp, const FastLioLivoxCloud& cloud) {
    fastlio::PointCloudXYZI::Ptr converted = ConvertLivoxCloud(cloud);
    std::lock_guard<std::mutex> lock(mutex_);
    if (timestamp < last_lidar_timestamp_) {
      lidar_buffer_.clear();
      lidar_time_buffer_.clear();
      lidar_pending_ = false;
    }
    last_lidar_timestamp_ = timestamp;
    lidar_buffer_.push_back(std::move(converted));
    lidar_time_buffer_.push_back(timestamp);
  }

  void PushImu(fastlio::ImuData imu) {
    imu.timestamp -= config_.lidar_to_imu_time_offset;
    std::lock_guard<std::mutex> lock(mutex_);
    if (imu.timestamp < last_imu_timestamp_) {
      imu_buffer_.clear();
    }
    last_imu_timestamp_ = imu.timestamp;
    imu_buffer_.push_back(std::move(imu));
  }

  bool BuildMeasureGroup(fastlio::MeasureGroup* group) {
    if (lidar_buffer_.empty() || imu_buffer_.empty()) {
      return false;
    }
    if (!lidar_pending_) {
      pending_lidar_ = lidar_buffer_.front();
      pending_lidar_begin_time_ = lidar_time_buffer_.front();
      if (pending_lidar_->size() <= 1U) {
        pending_lidar_end_time_ =
            pending_lidar_begin_time_ + lidar_mean_scan_time_;
      } else {
        const double measured_scan_time =
            pending_lidar_->points.back().curvature / 1000.0;
        if (measured_scan_time < 0.5 * lidar_mean_scan_time_) {
          pending_lidar_end_time_ =
              pending_lidar_begin_time_ + lidar_mean_scan_time_;
        } else {
          ++lidar_scan_count_;
          pending_lidar_end_time_ =
              pending_lidar_begin_time_ + measured_scan_time;
          lidar_mean_scan_time_ +=
              (measured_scan_time - lidar_mean_scan_time_) /
              lidar_scan_count_;
        }
      }
      lidar_pending_ = true;
    }
    if (last_imu_timestamp_ < pending_lidar_end_time_) {
      return false;
    }

    group->lidar = pending_lidar_;
    group->lidar_beg_time = pending_lidar_begin_time_;
    group->lidar_end_time = pending_lidar_end_time_;
    group->imu.clear();
    while (!imu_buffer_.empty() &&
           imu_buffer_.front().timestamp <= pending_lidar_end_time_) {
      group->imu.push_back(imu_buffer_.front());
      imu_buffer_.pop_front();
    }
    if (group->imu.empty()) {
      return false;
    }
    lidar_buffer_.pop_front();
    lidar_time_buffer_.pop_front();
    lidar_pending_ = false;
    return true;
  }

  FastLioOutput Step(bool include_registered_scan,
                     bool dense_registered_scan) {
    std::lock_guard<std::mutex> lock(mutex_);
    FastLioOutput output;
    fastlio::MeasureGroup group;
    if (!BuildMeasureGroup(&group)) {
      return output;
    }
    if (!odometry_->run_odometry(group) || !odometry_->HasPose()) {
      output.status = FastLioStepStatus::kInitializing;
      return output;
    }

    output.status = FastLioStepStatus::kPoseUpdated;
    output.pose_ready = true;
    output.timestamp = odometry_->LidarEndTime();
    output.pose = odometry_->Pose();
    output.pose.rot.normalize();
    output.velocity = odometry_->Velocity();
    const auto covariance = odometry_->Covariance();
    for (int row = 0; row < 6; ++row) {
      const int source_row = row < 3 ? row + 3 : row - 3;
      for (int column = 0; column < 6; ++column) {
        const int source_column = column < 3 ? column + 3 : column - 3;
        output.pose_covariance[row * 6 + column] =
            covariance(source_row, source_column);
      }
    }
    if (include_registered_scan) {
      output.registered_scan =
          odometry_->RegisteredScan(dense_registered_scan);
    }
    return output;
  }

  FastLioConfig config_;
  std::shared_ptr<CoreOdometry> odometry_;
  std::mutex mutex_;
  std::deque<fastlio::PointCloudXYZI::Ptr> lidar_buffer_;
  std::deque<double> lidar_time_buffer_;
  std::deque<fastlio::ImuData> imu_buffer_;
  fastlio::PointCloudXYZI::Ptr pending_lidar_;
  double pending_lidar_begin_time_{0.0};
  double pending_lidar_end_time_{0.0};
  double lidar_mean_scan_time_{0.0};
  int lidar_scan_count_{0};
  double last_lidar_timestamp_{0.0};
  double last_imu_timestamp_{-1.0};
  bool lidar_pending_{false};
};

FastLioPipeline::FastLioPipeline(const FastLioConfig& config,
                                 const std::string& output_root)
    : impl_(new Impl(config, output_root)) {}

FastLioPipeline::~FastLioPipeline() = default;

void FastLioPipeline::PushLivoxCloud(
    double timestamp, const FastLioLivoxCloud& cloud) {
  impl_->PushLivoxCloud(timestamp, cloud);
}

void FastLioPipeline::PushImu(const fastlio::ImuData& imu) {
  impl_->PushImu(imu);
}

FastLioOutput FastLioPipeline::Step(bool include_registered_scan,
                                    bool dense_registered_scan) {
  return impl_->Step(include_registered_scan, dense_registered_scan);
}

}  // namespace localization
}  // namespace jojo
