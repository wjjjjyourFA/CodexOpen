#include "modules/dreamview/map_center_view/config/static_config.h"

namespace jojo {
namespace dreamview {
// using json = nlohmann::json;

void StaticConfig::LoadConfig(const std::string& config_path) {
  try {
    yaml = YAML::LoadFile(config_path);

    map_file_path = yaml["map_file_path"].as<std::string>("");
    // map_center    = Vec3FromYaml(yaml["init"]["map_center"]);

    // init_method = yaml["init"]["init_method"].as<int>(0);
    // this->SwitchMethod(init_method);

    b_use_pose_center = yaml["init"]["b_use_pose_center"].as<bool>(false);

    b_display_roi  = yaml["init"]["b_display_roi"].as<bool>(false);
    b_display_body = yaml["init"]["b_display_body"].as<bool>(false);

    // yaw_offset_euler = yaml["init"]["yaw_offset_euler"].as<double>(0);

  } catch (const std::exception& e) {
    std::cerr << "Fail to open yaml file: " << e.what() << std::endl;
  }
};

/*
void StaticConfig::SwitchMethod(int method) {
  switch (method) {
    case 1:
      map_center       = Vec3FromYaml(yaml["init"]["map_center"]);
      yaw_offset_euler = yaml["init"]["yaw_offset_euler"].as<double>(0);
      break;
    case 3: {
      // read map_info.json
      map_info_file = yaml["init"]["map_info_file"].as<std::string>("map_info");

      // clang-format off
      // 找最后一个路径分隔符（兼容 Linux / Windows）
      auto pos = map_file_path.find_last_of("/\\");
      std::string parent = (pos == std::string::npos) ? "" : map_file_path.substr(0, pos);
      const std::string m_i_path = parent + "/" + map_info_file + ".json";
      // std::cout << "m_i_path: " << m_i_path << std::endl;
      // clang-format on

      this->LoadJson(m_i_path);
      break;
    }
    default:
      break;
  }
}

void StaticConfig::LoadJson(const std::string& config_path) {
  std::ifstream ifs(config_path);
  if (!ifs.is_open()) {
    std::cerr << "Config open failed: " << config_path << std::endl;
    return;
  }

  nlohmann::json j;
  try {
    ifs >> j;

    from_json(j, *this);
  } catch (std::exception& e) {
    std::cerr << "JSON parse error: " << e.what() << std::endl;
    return;
  }
}

void from_json(const nlohmann::json& j, StaticConfig& c) {
  // way 2
  const auto m = j.at("map_center").get<std::vector<double>>();
  c.map_center = Eigen::Map<const Eigen::Vector3d>(m.data());

  const auto& p = j.at("pose_offset");
  c.pose_offset.clear();
  for (int i = 0; i < 6; ++i) {
    c.pose_offset.push_back(p.at(i).get<double>());
  }
  c.yaw_offset_euler = c.pose_offset.at(5);
}
*/

}  // namespace dreamview
}  // namespace jojo
