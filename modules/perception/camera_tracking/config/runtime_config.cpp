#include "modules/perception/camera_tracking/config/runtime_config.h"

namespace jojo {
namespace perception {
namespace ct {

void RuntimeConfig::LoadConfig(const std::string& config_path) {
  try {
    // 创建一个 property_tree 对象
    boost::property_tree::ptree pt;

    // 读取 ini 文件
    boost::property_tree::ini_parser::read_ini(config_path, pt);

    // clang-format off
    // 从文件中读取配置项
    det_wts_file = pt.get<std::string>("general.det_wts_file", "");
    det_engine_file = pt.get<std::string>("general.det_engine_file", "");
    det_onnx_file = pt.get<std::string>("general.det_onnx_file", "");
    sort_wts_file = pt.get<std::string>("general.sort_wts_file", "");
    sort_engine_file = pt.get<std::string>("general.sort_engine_file", "");
    sort_onnx_file = pt.get<std::string>("general.sort_onnx_file", "");
    
    b_undistort = pt.get<bool>("general.b_undistort", false);
    calib_file_path = pt.get<std::string>("calibration.calib_file_path", "");
    // 打印其他参数...
    // clang-format on
  } catch (const std::exception& e) {
    std::cerr << "Error reading ini file: " << e.what() << std::endl;
  }
}

}  // namespace ct
}  // namespace perception
}  // namespace jojo
