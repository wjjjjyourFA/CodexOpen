#include "tools/data_loader/config/runtime_config_offline.h"

namespace jojo {
namespace tools {

void RuntimeConfigOffline::LoadConfig(const std::string& config_path) {
  try {
    // 创建一个 property_tree 对象
    boost::property_tree::ptree pt;
    // 读取 ini 文件
    boost::property_tree::ini_parser::read_ini(config_path, pt);

    // clang-format off
    // 从文件中读取配置项
    root_path = pt.get<std::string>("general.root_path", "");
    file_name = pt.get<std::string>("general.file_name", "");
    start_time = pt.get<int64_t>("general.start_time", 0);
    end_time = pt.get<int64_t>("general.end_time", 0);
    current_time = pt.get<int64_t>("general.current_time", 0);
    pause_time = pt.get<float>("general.pause_time", 1.0);

    b_local_pose = pt.get<bool>("sensors.LocalPose", false);
    b_global_pose = pt.get<bool>("sensors.GlobalPose", false);
    b_imu_data = pt.get<bool>("sensors.ImuData", false);
    b_radar = pt.get<int>("sensors.Radar", 0);
    b_radar4d = pt.get<int>("sensors.Radar4D", 0);
    b_lidar = pt.get<bool>("sensors.Lidar", false);
    b_camera = pt.get<int>("sensors.Camera", 0);
    b_infra = pt.get<int>("sensors.Infra", 0);
    b_star = pt.get<int>("sensors.Star", 0);

    b_compensation_cloud = pt.get<bool>("general.b_compensation_cloud", false);
    b_lt_none_rt = pt.get<int>("general.b_lt_none_rt", 1);
    use_bin_or_pcd = pt.get<bool>("general.b_bin_or_pcd", false);
    use_jpg_or_png = pt.get<int>("general.b_jpg_or_png", 0);
    use_txt_or_pcd = pt.get<bool>("general.b_txt_or_pcd", false);

    b_imu_vec = pt.get<bool>("general.b_imu_vec", false);
    b_pose_vec = pt.get<bool>("general.b_pose_vec", false);

    calib_file_dir = pt.get<std::string>("calibration.calib_file_dir", "");

    b_undistort = pt.get<bool>("general.b_undistort", false);
    b_do_undistort = pt.get<bool>("topics.b_do_undistort", false);
    camera_name = ParseCommaSeparated(pt, "device_names.CameraName");
    infra_name = ParseCommaSeparated(pt, "device_names.InfraName");
    star_name = ParseCommaSeparated(pt, "device_names.StarName");

    lidar_type = pt.get<std::string>("device_names.LidarType", "");

    radar_type = pt.get<std::string>("device_names.RadarType", "");
    radar4d_type = pt.get<std::string>("device_names.Radar4DType", "");

    // 打印读取的配置
    std::cout << "root_path: " << root_path << std::endl;
    std::cout << "file_path: " << file_name << std::endl;

    // 打印其他参数...
    // clang-format on
  } catch (const std::exception& e) {
    std::cerr << "Error reading ini file: " << e.what() << std::endl;
  }
}

}  // namespace tools
}  // namespace jojo
