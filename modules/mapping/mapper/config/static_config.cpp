#include "modules/mapping/mapper/config/static_config.h"

namespace jojo {
namespace mapping {

void StaticConfig::LoadConfig(const std::string& config_path) {
  try {
    yaml = YAML::LoadFile(config_path);

    map_save_path = yaml["map_save_path"].as<std::string>("");

    b_use_pose_center = yaml["init"]["b_use_pose_center"].as<bool>(false);

  } catch (const std::exception& e) {
    std::cerr << "Fail to open yaml file: " << e.what() << std::endl;
  }
};

}  // namespace mapping
}  // namespace jojo
