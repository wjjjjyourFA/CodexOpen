#include "toolz/data_processor/config/interface_config.h"

namespace jojo {
namespace tools {
namespace cfg = ::jojo::common::config;

void InterfaceConfig::LoadConfig(const std::string& config_path) {
  std::ifstream ifs(config_path);
  if (!ifs.is_open()) {
    std::cerr << "Config open failed: " << config_path << std::endl;
    return;
  }

  nlohmann::json j;
  try {
    ifs >> j;
    *this = j.get<InterfaceConfig>();
  } catch (std::exception& e) {
    std::cerr << "JSON parse error: " << e.what() << std::endl;
    return;
  }
}

void from_json(const nlohmann::json& j, InterfaceConfig& c) {
  // clang-format off
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
  // clang-format on
}

}  // namespace tools
}  // namespace jojo
