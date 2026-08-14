#ifndef TOOLS_MANUAL_LOOP_SESSION_EXPORTER_H
#define TOOLS_MANUAL_LOOP_SESSION_EXPORTER_H

#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace jojo {
namespace tools {
namespace manual_loop {

struct SessionExporterConfig {
  std::string output_directory = "/tmp/codexopen_manual_loop_session";
  double keyframe_distance = 1.5;
  double keyframe_angle_deg = 0.0;
  double min_keyframe_time = 0.0;
  bool input_cloud_is_global = true;
  double voxel_leaf_size = 0.0;
  double translation_information = 1000.0;
  double rotation_information = 1000.0;
  bool overwrite_existing = false;
};

struct ExportedKeyframe {
  double timestamp = 0.0;
  Eigen::Isometry3d pose_world_sensor = Eigen::Isometry3d::Identity();
};

struct AddFrameResult {
  bool saved = false;
  std::size_t index = 0;
  std::size_t point_count = 0;
};

class SessionExporter {
 public:
  explicit SessionExporter(const SessionExporterConfig& config);
  ~SessionExporter();

  AddFrameResult AddFrame(
      double timestamp, const Eigen::Isometry3d& pose_world_sensor,
      const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud);
  void Finalize() const;

  std::size_t KeyframeCount() const { return keyframes_.size(); }
  const std::string& OutputDirectory() const {
    return config_.output_directory;
  }

 private:
  void ValidateConfig() const;
  void PrepareOutputDirectory();
  bool ShouldCreateKeyframe(const Eigen::Isometry3d& current_pose,
                            double timestamp) const;
  static Eigen::Quaterniond NormalizedQuaternion(
      const Eigen::Matrix3d& rotation);
  void WriteTum() const;
  void WriteG2o() const;

  SessionExporterConfig config_;
  std::vector<ExportedKeyframe> keyframes_;
};

}  // namespace manual_loop
}  // namespace tools
}  // namespace jojo

#endif  // TOOLS_MANUAL_LOOP_SESSION_EXPORTER_H
