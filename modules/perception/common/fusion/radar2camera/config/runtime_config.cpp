#include "modules/perception/common/fusion/radar2camera/config/runtime_config.h"

namespace jojo {
namespace perception {

void RuntimeConfig::LoadConfig(const std::string& config_path) {
  try {
    // clang-format off
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(config_path, pt);

    b_do_undistort = pt.get<bool>("general.b_do_undistort", 0);

    dist_threshold = pt.get<int>("general.dist_threshold", 1000);

    camera_calib_file_path = pt.get<std::string>("calibration.camera_calib_file_path", "");
    radar_calib_file_path = pt.get<std::string>("calibration.radar_calib_file_path", "");
    // clang-format on
  } catch (const std::exception& e) {
    std::cerr << "Error reading ini file: " << e.what() << std::endl;
  }
}

}  // namespace perception
}  // namespace jojo
