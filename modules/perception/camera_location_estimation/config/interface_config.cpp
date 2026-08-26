#include "modules/perception/camera_location_estimation/config/interface_config.h"

#include <iostream>

namespace jojo {
namespace perception {
namespace cle {

void InterfaceConfig::LoadConfig(const std::string& config_path) {
  valid = false;
  validation_error.clear();
  try {
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(config_path, pt);
    b_compressed = pt.get<bool>("general.b_compressed", true);

    rate = pt.get<int>("general.rate", 10);

    image_topic = pt.get<std::string>("topics.image_topic", "");
    lidar_topic = pt.get<std::string>("topics.lidar_topic", "");

    valid = Validate(&validation_error);
    if (!valid) std::cerr << validation_error << std::endl;
  } catch (const std::exception& e) {
    validation_error = std::string("Error reading ini file: ") + e.what();
    std::cerr << validation_error << std::endl;
  }
}

bool InterfaceConfig::Validate(std::string* error) const {
  if (image_topic.empty() || lidar_topic.empty()) {
    if (error) *error = "image_topic and lidar_topic must not be empty";
    return false;
  }
  if (rate <= 0) {
    if (error) *error = "rate must be greater than zero";
    return false;
  }
  if (error) error->clear();
  return true;
}

}  // namespace cle
}  // namespace perception
}  // namespace jojo
