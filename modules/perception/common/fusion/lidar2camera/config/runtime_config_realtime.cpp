#include "modules/perception/common/fusion/lidar2camera/config/runtime_config_realtime.h"

namespace jojo {
namespace perception {

void RuntimeConfig::LoadConfig(const std::string& config_path) {
  try {
    // clang-format off
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(config_path, pt);
    b_compressed = pt.get<bool>("general.b_compressed", true);
    b_undistort = pt.get<bool>("general.b_undistort", false);
    b_lt_none_rt = pt.get<int>("general.b_lt_none_rt", 1);

    rate = pt.get<int>("general.rate", 10);
    dist_threshold = pt.get<int>("general.dist_threshold", 100);

    image_topic = pt.get<std::string>("topics.image_topic", "");
    lidar_topic = pt.get<std::string>("topics.lidar_topic", "");

    calib_file_path = pt.get<std::string>("calibration.calib_file_path", "");
    // clang-format on
  } catch (const std::exception& e) {
    std::cerr << "Error reading ini file: " << e.what() << std::endl;
  }
}

}  // namespace tools
}  // namespace jojo
