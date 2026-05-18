#include "modules/mapping/map_visualization/config/static_config.h"

namespace jojo {
namespace mapping {
using json = nlohmann::json;

void StaticConfig::LoadConfig(const std::string& config_path) {
  try {
    yaml = YAML::LoadFile(config_path);

    map_file_path = yaml["map_file_path"].as<std::string>("");

    b_use_pose_center = yaml["init"]["b_use_pose_center"].as<bool>(false);

    // read map_info.json
    map_info_file = yaml["init"]["map_info_file"].as<std::string>("map_info");
    // clang-format off
    auto pos = map_file_path.find_last_of("/\\");
    std::string parent = (pos == std::string::npos) ? "" : map_file_path.substr(0, pos);
    const std::string m_i_path = parent + "/" + map_info_file + ".json";
    // clang-format on
    // 读取 数据集 中的 json
    this->LoadJson(m_i_path);

  } catch (const std::exception& e) {
    std::cerr << "Fail to open yaml file: " << e.what() << std::endl;
  }
};

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
  const auto t = j.at("map2d");
  c.map_min_x  = t.at("min_x").get<double>();
  c.map_min_y  = t.at("min_y").get<double>();

  c.map_resolution  = t.at("resolution").get<float>();

  c.width  = j.at("width").get<int>();
  c.height = j.at("height").get<int>();
}

}  // namespace mapping
}  // namespace jojo
