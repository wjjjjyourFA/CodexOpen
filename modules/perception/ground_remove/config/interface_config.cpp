#include "modules/perception/ground_remove/config/interface_config.h"

namespace jojo {
namespace perception {

void InterfaceConfig::LoadConfig(const std::string& config_path) {
  try {
    // clang-format off
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(config_path, pt);

    rate = pt.get<int>("general.rate", 10);

    pose_topic = pt.get<std::string>("topics.pose_topic", "");
    lidar_topic = pt.get<std::string>("topics.lidar_topic", "");

    map_topic = pt.get<std::string>("topics.map_topic", "");

    // clang-format on
  } catch (const std::exception& e) {
    std::cerr << "Error reading ini file: " << e.what() << std::endl;
  }
}

}  // namespace perception
}  // namespace jojo
