#include "tools/data_processor/config/runtime_config.h"

namespace jojo {
namespace tools {

void RuntimeConfig::LoadConfig(const std::string& config_path) {
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

    b_local_pose = pt.get<bool>("sensors.LocalPose", false);
    b_global_pose = pt.get<bool>("sensors.GlobalPose", false);
    b_imu_data = pt.get<bool>("sensors.Imu", false);
    b_radar = pt.get<int>("sensors.Radar", 0);
    b_radar_4d = pt.get<int>("sensors.Radar4D", 0);
    b_lidar = pt.get<bool>("sensors.Lidar", false);
    b_camera = pt.get<int>("sensors.Camera", 0);
    b_infra = pt.get<int>("sensors.Infra", 0);
    b_star = pt.get<int>("sensors.Star", 0);

    b_compensation_cloud = pt.get<bool>("general.b_compensation_cloud", false);
    b_lt_none_rt = pt.get<int>("general.b_lt_none_rt", 1);
    use_bin_or_pcd = pt.get<bool>("general.b_bin_or_pcd", false);
    use_jpg_or_png = pt.get<int>("general.b_jpg_or_png", 0);
    use_txt_or_pcd = pt.get<bool>("general.b_txt_or_pcd", false);

    calib_file_path = pt.get<std::string>("calibration.calib_file_path", "");

    // 按顺序读取相机参数
    b_undistort = pt.get<bool>("topics.b_undistort", false);
    camera_name = ParseCommaSeparated(pt, "device_names.CameraName");
    infra_name = ParseCommaSeparated(pt, "device_names.InfraName");
    star_name = ParseCommaSeparated(pt, "device_names.StarName");

    lidar_type = pt.get<std::string>("device_names.LidarType", "");

    radar_type = pt.get<std::string>("device_names.RadarType", "");
    radar_4d_type = pt.get<std::string>("device_names.Radar4DType", "");

    topic_local_pose_sub = pt.get<std::string>("topics.topic_local_pose_sub", "");
    topic_global_pose_sub = pt.get<std::string>("topics.topic_global_pose_sub", "");
    topic_imu_data_sub = pt.get<std::string>("topics.topic_imu_data_sub", "");
    topic_pose_sub = pt.get<std::string>("topics.topic_pose_sub", "");

    b_difop = pt.get<bool>("topics.b_difop", false);
    topic_lidar_sub = pt.get<std::string>("topics.topic_lidar_sub", "");
    topic_lidar_ori_sub = pt.get<std::string>("topics.topic_lidar_ori_sub", "");
    topic_lidar_difop_sub = pt.get<std::string>("topics.topic_lidar_difop_sub", "");

    b_compressed = pt.get<bool>("topics.b_compressed", false);
    topic_camera_sub = ReadStringArray(pt, "topics.topic_camera_sub_", b_camera);
    topic_infra_sub = ReadStringArray(pt, "topics.topic_infra_sub_", b_infra);
    topic_star_sub = ReadStringArray(pt, "topics.topic_star_sub_", b_star);

    topic_radar_sub = pt.get<std::string>("topics.topic_radar_sub_", "");
    topic_radar_4d_sub = ReadStringArray(pt, "topics.topic_radar_4d_sub_", b_radar_4d);

    // 打印读取的配置
    std::cout << "rosbag_path: " << rosbag_path << std::endl;
    std::cout << "rosbag_name: " << rosbag_name << std::endl;
    std::cout << "b_save_data: " << b_save_data << std::endl;
    std::cout << "save_path: " << save_path << std::endl;
    // std::cout << "Lpose: " << b_local_pose << std::endl;

    // 打印其他参数...
    // clang-format on
  } catch (const std::exception& e) {
    std::cerr << "Error reading ini file: " << e.what() << std::endl;
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
