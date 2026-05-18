#include "toolz/data_processor/config/runtime_config.h"

namespace jojo {
namespace tools {
namespace cfg = ::jojo::common::config;

void RuntimeConfig::LoadConfig(const std::string& config_path) {
  std::ifstream ifs(config_path);
  if (!ifs.is_open()) {
    std::cerr << "Config open failed: " << config_path << std::endl;
    return;
  }

  nlohmann::json j;
  try {
    ifs >> j;
    *this = j.get<RuntimeConfig>();
  } catch (std::exception& e) {
    std::cerr << "JSON parse error: " << e.what() << std::endl;
    return;
  }

  switch (use_jpg_or_png) {
    case -1:
      compress_params.push_back(cv::IMWRITE_JPEG_QUALITY);
      compress_params.push_back(50);
      break;
    case 0:
      compress_params.push_back(cv::IMWRITE_JPEG_QUALITY);
      // default use jpg in cv is 95
      compress_params.push_back(95);
      break;
    case 1:
      compress_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
      // default use png in cv is 3
      // PNG 是无损压缩格式 0->9 耗时增加 压缩率提升
      compress_params.push_back(9);
      break;
    default:
      break;
  }
}

void from_json(const nlohmann::json& j, RuntimeConfig& c) {
  // clang-format off
  // ========= general =========
  const auto& g = j.at("general");
  c.rosbag_path = cfg::GetOrDefault(g, "rosbag_path", std::string(""));
  c.rosbag_name = cfg::GetOrDefault(g, "rosbag_name", std::string(""));
  c.b_save_data = cfg::GetOrDefault(g, "b_save_data", false);
  c.save_path   = cfg::GetOrDefault(g, "save_path", std::string(""));

  c.prepare_data_num = cfg::GetOrDefault(g, "prepare_data_num", -1);
  c.sample_interval  = cfg::GetOrDefault(g, "sample_interval", 1);
  c.useless_time     = cfg::GetOrDefault(g, "useless_time", 0);

  c.b_compensation_cloud = cfg::GetOrDefault(g, "b_compensation_cloud", false);
  c.b_lt_none_rt         = cfg::GetOrDefault(g, "b_lt_none_rt", 1);

  c.use_bin_or_pcd = cfg::GetOrDefault(g, "b_bin_or_pcd", true);
  c.use_txt_or_pcd = cfg::GetOrDefault(g, "b_txt_or_pcd", false);
  c.use_jpg_or_png = cfg::GetOrDefault(g, "b_jpg_or_png", 0);

  // ========= sensors =========
  const auto& s   = j.at("sensors");
  c.b_local_pose  = cfg::GetOrDefault(s, "LocalPose", false);
  c.b_global_pose = cfg::GetOrDefault(s, "GlobalPose", false);
  c.b_imu_data    = cfg::GetOrDefault(s, "ImuData", false);
  c.b_camera      = cfg::GetOrDefault(s, "Camera", 0);
  c.b_infra       = cfg::GetOrDefault(s, "Infra", 0);
  c.b_star        = cfg::GetOrDefault(s, "Star", 0);
  c.b_radar       = cfg::GetOrDefault(s, "Radar", 0);
  c.b_radar4d     = cfg::GetOrDefault(s, "Radar4D", 0);
  c.b_lidar       = cfg::GetOrDefault(s, "Lidar", false);

  // ========= topics =========
  const auto& t = j.at("topics");

  c.topic_global_pose_sub = cfg::GetOrDefault(t, "global_pose", std::string(""));
  c.topic_local_pose_sub  = cfg::GetOrDefault(t, "local_pose", std::string(""));
  c.topic_imu_data_sub    = cfg::GetOrDefault(t, "imu_data", std::string(""));

  if (t.contains("lidar")) {
    const auto& l           = t.at("lidar");
    c.b_difop               = cfg::GetOrDefault(l, "b_difop", false);
    c.topic_lidar_ori_sub   = cfg::GetOrDefault(l, "ori", std::string(""));
    c.topic_lidar_difop_sub = cfg::GetOrDefault(l, "difop", std::string(""));
    c.topic_lidar_sub       = cfg::GetOrDefault(l, "points", std::string(""));
  }

  c.b_compressed   = cfg::GetOrDefault(t, "b_compressed", true);
  c.b_do_undistort = cfg::GetOrDefault(t, "b_do_undistort", true);

  if (t.contains("camera")) {
    const auto& cam = t.at("camera");
    c.topic_camera_sub =
        cfg::GetOrDefault(cam, "topics", std::vector<std::string>{});
  }

  if (t.contains("infra")) {
    c.topic_infra_sub =
        cfg::GetOrDefault(t.at("infra"), "topics", std::vector<std::string>{});
  }

  if (t.contains("star")) {
    c.topic_star_sub =
        cfg::GetOrDefault(t.at("star"), "topics", std::vector<std::string>{});
  }

  if (t.contains("radar")) {
    c.topic_radar_sub = cfg::GetOrDefault(t.at("radar"), "topic", std::string(""));
  }

  if (t.contains("radar4d")) {
    c.topic_radar4d_sub =
        cfg::GetOrDefault(t.at("radar4d"), "topics", std::vector<std::string>{});
  }

  // ========= calibration =========
  c.calib_file_dir = j.at("calibration").value("calib_file_dir", std::string(""));

  // ========= device =========
  const auto& d = j.at("device_names");

  c.lidar_type   = cfg::GetOrDefault(d, "LidarType", std::string(""));
  c.camera_name  = cfg::GetOrDefault(d, "CameraName", std::vector<std::string>{});
  c.infra_name   = cfg::GetOrDefault(d, "InfraName", std::vector<std::string>{});
  c.star_name    = cfg::GetOrDefault(d, "StarName", std::vector<std::string>{});
  c.radar_type   = cfg::GetOrDefault(d, "RadarType", std::string(""));
  c.radar4d_type = cfg::GetOrDefault(d, "Radar4DType", std::string(""));
  // clang-format on
}

}  // namespace tools
}  // namespace jojo
