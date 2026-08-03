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

  // ========= topics =========
  const auto& t = j.at("topics");

  c.b_do_undistort = cfg::GetOrDefault(t, "b_do_undistort", true);

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
