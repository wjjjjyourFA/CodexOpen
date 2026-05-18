#include "modules/perception/camera_tracking/config/runtime_config_realtime.h"

namespace jojo {
namespace perception {
namespace ct {

void RuntimeConfig::LoadConfig(const std::string& config_path) {
  try {
    // clang-format off
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(config_path, pt);
    det_wts_file = pt.get<std::string>("general.det_wts_file", "");
    det_engine_file = pt.get<std::string>("general.det_engine_file", "");
    det_onnx_file = pt.get<std::string>("general.det_onnx_file", "");
    sort_wts_file = pt.get<std::string>("general.sort_wts_file", "");
    sort_engine_file = pt.get<std::string>("general.sort_engine_file", "");
    sort_onnx_file = pt.get<std::string>("general.sort_onnx_file", "");

    b_compressed = pt.get<bool>("general.b_compressed", true);
    b_undistort = pt.get<bool>("general.b_undistort", false);

    rate = pt.get<int>("general.rate", 10);

    image_topic = pt.get<std::string>("topics.image_topic", "");

    calib_file_path = pt.get<std::string>("calibration.calib_file_path", "");
    // clang-format on
  } catch (const std::exception& e) {
    std::cerr << "Error reading ini file: " << e.what() << std::endl;
  }
}

}  // namespace ct
}  // namespace perception
}  // namespace jojo
