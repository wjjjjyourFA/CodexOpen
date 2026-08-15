#include "tools/data_processor/config/runtime_config.h"

#include <stdexcept>

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
    rosbag_path = pt.get<std::string>("general.rosbag_path", "");
    rosbag_name = pt.get<std::string>("general.rosbag_name", "");
    b_save_data = pt.get<bool>("general.b_save_data", false);
    save_path = pt.get<std::string>("general.save_path", "");
    prepare_data_num = pt.get<int>("general.prepare_data_num", -1);
    sample_interval = pt.get<int>("general.sample_interval", 1);
    useless_time = pt.get<int>("general.useless_time", 0);
    distance_epsilon = pt.get<float>("general.distance_epsilon", 1e-3);
    intensity_epsilon = pt.get<float>("general.intensity_epsilon", 0);

    b_compensation_cloud = pt.get<bool>("general.b_compensation_cloud", false);
    b_lt_none_rt = pt.get<int>("general.b_lt_none_rt", 1);
    use_bin_or_pcd = pt.get<bool>("general.b_bin_or_pcd", false);
    use_jpg_or_png = pt.get<int>("general.b_jpg_or_png", 0);
    use_txt_or_pcd = pt.get<bool>("general.b_txt_or_pcd", false);

    calib_file_dir = pt.get<std::string>("calibration.calib_file_dir", "");

    // 按顺序读取相机参数
    b_do_undistort = pt.get<bool>("topics.b_do_undistort", false);
    camera_name = ParseCommaSeparated(pt, "device_names.CameraName");
    infra_name = ParseCommaSeparated(pt, "device_names.InfraName");
    star_name = ParseCommaSeparated(pt, "device_names.StarName");

    imu_type = pt.get<std::string>("device_names.ImuType", "");

    lidar_type = pt.get<std::string>("device_names.LidarType", "");

    radar_type = pt.get<std::string>("device_names.RadarType", "");
    radar4d_type = pt.get<std::string>("device_names.Radar4DType", "");

    if (sample_interval <= 0) {
      throw std::invalid_argument("general.sample_interval must be positive");
    }
    if (prepare_data_num == 0 || prepare_data_num < -1) {
      throw std::invalid_argument(
          "general.prepare_data_num must be -1 or positive");
    }
    if (useless_time < 0 || distance_epsilon < 0 || intensity_epsilon < 0) {
      throw std::invalid_argument(
          "time and point filtering thresholds must be non-negative");
    }
    if (use_jpg_or_png < -1 || use_jpg_or_png > 1) {
      throw std::invalid_argument("general.b_jpg_or_png must be -1, 0, or 1");
    }

    // 打印读取的配置
    std::cout << "rosbag_path: " << rosbag_path << std::endl;
    std::cout << "rosbag_name: " << rosbag_name << std::endl;
    std::cout << "b_save_data: " << b_save_data << std::endl;
    std::cout << "save_path: " << save_path << std::endl;
    // std::cout << "Lpose: " << b_local_pose << std::endl;

    // 打印其他参数...
    // clang-format on
  } catch (const std::exception& e) {
    throw std::runtime_error("failed to load runtime config '" + config_path +
                             "': " + e.what());
  }

  switch (use_jpg_or_png) {
    case -1:
      compress_params.push_back(cv::IMWRITE_JPEG_QUALITY);
      compress_params.push_back(50);
      break;
    case 0:
      compress_params.push_back(cv::IMWRITE_JPEG_QUALITY);
      // default use jpg in cv is 95
      compress_params.push_back(95);
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
