#include "modules/perception/ground_remove/config/runtime_config.h"

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
    far_resolution = pt.get<float>("map.far_resolution", 0.5);
    middle_resolution = pt.get<float>("map.middle_resolution", 0.4);
    near_resolution = pt.get<float>("map.near_resolution", 0.2);

    map_resolution = pt.get<float>("map.map_resolution", 0.2);

    map_rows = pt.get<int32_t>("map.map_rows", 512);
    map_cols = pt.get<int32_t>("map.map_cols", 512);

    min_bound = ReadVec3(pt, "min_bound", Eigen::Vector3f(-5, -3.5, -5));
    max_bound = ReadVec3(pt, "max_bound", Eigen::Vector3f(6, 3.5, 2));

    ref_height = pt.get<float>("general.ref_height", 0.0);
    hanging_z = pt.get<float>("general.hanging_z", 2.0);
    mean_z_thresh = pt.get<float>("general.mean_z_thresh", -0.55);
    delta_z_thresh = pt.get<float>("general.delta_z_thresh", 0.1);
    b_use_legacy = pt.get<bool>("general.b_use_legacy", false);
    b_use_gaussian = pt.get<bool>("general.b_use_gaussian", false);
    b_gravity = pt.get<bool>("general.b_gravity", false);

    dist_threshold = pt.get<int>("general.dist_threshold", 100);

    camera_calib_file_path = pt.get<std::string>("calibration.camera_calib_file_path", "");
    gravity_lidar_calib_file_path = pt.get<std::string>("calibration.gravity_lidar_calib_file_path", "");

    b_show_color_point = pt.get<bool>("visualize.b_show_color_point", false);

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
