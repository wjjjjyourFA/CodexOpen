#include "tools/data_loader/config/runtime_config.h"

namespace jojo {
namespace tools {

void RuntimeConfig::LoadConfig(const std::string& config_path) {
  compress_params.clear();

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

    b_compensation_cloud = pt.get<bool>("general.b_compensation_cloud", false);
    b_lt_none_rt = pt.get<int>("general.b_lt_none_rt", 1);
    use_bin_or_pcd = pt.get<bool>("general.b_bin_or_pcd", false);
    use_jpg_or_png = pt.get<int>("general.b_jpg_or_png", 0);
    use_txt_or_pcd = pt.get<bool>("general.b_txt_or_pcd", false);

    b_imu_vec = pt.get<bool>("general.b_imu_vec", false);
    b_pose_vec = pt.get<bool>("general.b_pose_vec", false);

    calib_file_dir = pt.get<std::string>("calibration.calib_file_dir", "");

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
    return;
  }

  switch (use_jpg_or_png) {
    case -1:
      compress_params.push_back(cv::IMWRITE_JPEG_QUALITY);
      compress_params.push_back(50);
      break;
    case 0:
      compress_params.push_back(cv::IMWRITE_JPEG_QUALITY);
      // default use jpg in cv is 95
      // compress_params.push_back(95);
      break;
    case 1:
      compress_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
      // default use png in cv is 3
      // PNG 是无损压缩格式 0->9 耗时增加 压缩率提升
      compress_params.push_back(9);
      break;
    default:
      break;
  }
}

}  // namespace tools
}  // namespace jojo
