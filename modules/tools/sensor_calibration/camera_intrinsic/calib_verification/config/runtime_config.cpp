#include "modules/tools/sensor_calibration/camera_intrinsic/calib_verification/config/runtime_config.h"

namespace jojo {
namespace tools {

void RuntimeConfig::LoadConfig(const std::string& config_path) {
  try {
    // clang-format off
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(config_path, pt);
    data_root_path = pt.get<std::string>("general.data_root_path", "");
    
    b_jpg_or_png = pt.get<bool>("general.b_jpg_or_png", 0);

    image_name = pt.get<std::string>("topics.image_name", "");

    // clang-format on
  } catch (const std::exception& e) {
    std::cerr << "Error reading ini file: " << e.what() << std::endl;
  }

  InitPrefixPath();
}

void RuntimeConfig::InitPrefixPath() {
  if (!b_jpg_or_png) {
    image_file = data_root_path + "/" + image_name + ".jpg";
  } else {
    image_file = data_root_path + "/" + image_name + ".png";
  }
}

}  // namespace tools
}  // namespace jojo
