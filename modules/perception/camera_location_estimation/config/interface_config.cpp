#include "modules/perception/camera_location_estimation/config/interface_config.h"

namespace jojo {
namespace perception {
namespace cle {

void InterfaceConfig::LoadConfig(const std::string& config_path) {
  try {
    // clang-format off
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(config_path, pt);

    b_compressed = pt.get<bool>("general.b_compressed", true);

    rate = pt.get<int>("general.rate", 10);

    image_topic = pt.get<std::string>("topics.image_topic", "");
    lidar_topic = pt.get<std::string>("topics.lidar_topic", "");

    // clang-format on
  } catch (const std::exception& e) {
    std::cerr << "Error reading ini file: " << e.what() << std::endl;
  }
}

}  // namespace cle
}  // namespace perception
}  // namespace jojo
