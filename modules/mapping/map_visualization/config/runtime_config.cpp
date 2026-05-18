#include "modules/mapping/map_visualization/config/runtime_config.h"

namespace jojo {
namespace mapping {

void RuntimeConfig::LoadConfig(const std::string& config_path) {
  try {
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(config_path, pt);

    // clang-format off
    // 从文件中读取配置项
    root_path = pt.get<std::string>("general.root_path", "");
    file_name = pt.get<std::string>("general.file_name", "");

    b_generate_label_gray = pt.get<bool>("general.b_generate_label_gray", false);
    b_generate_label_color = pt.get<bool>("general.b_generate_label_color", false);

    half_length = pt.get<float>("map.half_length", 1.0);
    max_search_dist = pt.get<float>("map.max_search_dist", 30.0);
    line_width = pt.get<float>("map.line_width", 1.0);

    camera_calib_file_path = pt.get<std::string>("calibration.camera_calib_file_path", "");
    lidar_calib_file_path = pt.get<std::string>("calibration.lidar_calib_file_path", "");

    // 打印其他参数...
    // clang-format on
  } catch (const std::exception& e) {
    std::cerr << "Error reading ini file: " << e.what() << std::endl;
  }
}

}  // namespace mapping
}  // namespace jojo