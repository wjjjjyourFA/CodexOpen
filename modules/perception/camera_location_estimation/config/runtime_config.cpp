#include "modules/perception/camera_location_estimation/config/runtime_config.h"

namespace jojo {
namespace perception {
namespace cle {

void RuntimeConfig::LoadConfig(const std::string& config_path) {
  try {
    // 创建一个 property_tree 对象
    boost::property_tree::ptree pt;

    // 读取 ini 文件
    boost::property_tree::ini_parser::read_ini(config_path, pt);

    // clang-format off
    // 从文件中读取配置项
    wts_file = pt.get<std::string>("general.wts_file", "");
    engine_file = pt.get<std::string>("general.engine_file", "");
    onnx_file = pt.get<std::string>("general.onnx_file", "");

    use_det_or_track = pt.get<int>("general.use_det_or_track", 0);

    calib_file_path = pt.get<std::string>("calibration.calib_file_path", "");

    // 打印其他参数...
    // clang-format on
  } catch (const std::exception& e) {
    std::cerr << "Error reading ini file: " << e.what() << std::endl;
  }
}

}
}  // namespace
}  // namespace jojo

