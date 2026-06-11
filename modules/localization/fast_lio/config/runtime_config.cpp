#include "modules/localization/fast_lio/config/runtime_config.h"

namespace jojo {
namespace localization {

void RuntimeConfig::LoadConfig(const std::string& config_path) {
  try {
    // 创建一个 property_tree 对象
    boost::property_tree::ptree pt;
    // 读取 ini 文件
    boost::property_tree::ini_parser::read_ini(config_path, pt);

    // clang-format off
    // 从文件中读取配置项
    root_path = pt.get<std::string>("general.root_path", "");
    file_name = pt.get<std::string>("general.file_name", "");

    b_save_pcd = pt.get<bool>("general.b_save_pcd", false);
    b_only_times = pt.get<bool>("general.b_only_times", false);

    camera_calib_file_path = pt.get<std::string>("calibration.camera_calib_file_path", "");
    lidar_calib_file_path = pt.get<std::string>("calibration.lidar_calib_file_path", "");
    gravity_imu_calib_file_path = pt.get<std::string>("calibration.gravity_imu_calib_file_path", "");
    vehicle_config_file_path = pt.get<std::string>("calibration.vehicle_config_file_path", "");

    // 打印其他参数...
    // clang-format on
  } catch (const std::exception& e) {
    std::cerr << "Error reading ini file: " << e.what() << std::endl;
  }
}

}  // namespace tools
}  // namespace jojo
