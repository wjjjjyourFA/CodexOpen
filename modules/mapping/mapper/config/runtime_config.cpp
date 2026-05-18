#include "modules/mapping/mapper/config/runtime_config.h"

namespace jojo {
namespace mapping {

void RuntimeConfig::LoadConfig(const std::string& config_path) {
  try {
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(config_path, pt);

    // clang-format off
    // 从文件中读取配置项
    b_generate_map3d = pt.get<bool>("general.b_generate_map3d", false);
    b_generate_color_map = pt.get<bool>("general.b_generate_color_map", false);

    map_resolution = pt.get<float>("map.map_resolution", 0.2);
    sampling_distance = pt.get<float>("map.sampling_distance", 2.0);

    b_use_time_interpolate = pt.get<bool>("map.b_use_time_interpolate", false);

    b_realtime_show = pt.get<bool>("visualize.b_realtime_show", false);

    camera_calib_file_path = pt.get<std::string>("calibration.camera_calib_file_path", "");

    // 打印其他参数...
    // clang-format on
  } catch (const std::exception& e) {
    std::cerr << "Error reading ini file: " << e.what() << std::endl;
  }
}

}  // namespace mapping
}  // namespace jojo