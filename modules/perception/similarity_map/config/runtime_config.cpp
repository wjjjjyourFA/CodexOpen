#include "modules/perception/similarity_map/config/runtime_config.h"

namespace jojo {
namespace perception {

void RuntimeConfig::LoadConfig(const std::string& config_path) {
  try {
    // 创建一个 property_tree 对象
    boost::property_tree::ptree pt;

    // 读取 ini 文件
    boost::property_tree::ini_parser::read_ini(config_path, pt);

    // clang-format off
    // 从文件中读取配置项
    map_resolution = pt.get<float>("map.map_resolution", 0.2);

    map_rows = pt.get<int32_t>("map.map_rows", 512);
    map_cols = pt.get<int32_t>("map.map_cols", 512);

    min_bound = ReadVec3(pt, "min_bound", Eigen::Vector3f(-5, -3.5, -5));
    max_bound = ReadVec3(pt, "max_bound", Eigen::Vector3f(6, 3.5, 2));

    b_show_color_point = pt.get<bool>("visualize.b_show_color_point", false);

    camera_calib_file_path = pt.get<std::string>("calibration.camera_calib_file_path", "");

    // 打印其他参数...
    // clang-format on
  } catch (const std::exception& e) {
    std::cerr << "Error reading ini file: " << e.what() << std::endl;
  }
}

Eigen::Vector3f ReadVec3(const boost::property_tree::ptree& pt,
                         const std::string& key,
                         const Eigen::Vector3f& default_val) {
  return Eigen::Vector3f(pt.get<float>(key + ".x", default_val.x()),
                         pt.get<float>(key + ".y", default_val.y()),
                         pt.get<float>(key + ".z", default_val.z()));
}

}  // namespace perception
}  // namespace jojo
