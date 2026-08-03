#include "tools/data_loader/config/interface_config.h"

namespace jojo {
namespace tools {

void InterfaceConfig::LoadConfig(const std::string& config_path) {
  try {
    // 创建一个 property_tree 对象
    boost::property_tree::ptree pt;
    // 读取 ini 文件
    boost::property_tree::ini_parser::read_ini(config_path, pt);

    // clang-format off
    // 从文件中读取配置项
    b_local_pose = pt.get<bool>("sensors.LocalPose", false);
    b_global_pose = pt.get<bool>("sensors.GlobalPose", false);
    b_imu_data = pt.get<bool>("sensors.ImuData", false);
    b_radar = pt.get<int>("sensors.Radar", 0);
    b_radar4d = pt.get<int>("sensors.Radar4D", 0);
    b_lidar = pt.get<bool>("sensors.Lidar", false);
    b_camera = pt.get<int>("sensors.Camera", 0);
    b_infra = pt.get<int>("sensors.Infra", 0);
    b_star = pt.get<int>("sensors.Star", 0);

    b_undistort = pt.get<bool>("general.b_undistort", false);

    topic_local_pose_pub = pt.get<std::string>("topics.topic_local_pose_pub", "");
    topic_global_pose_pub = pt.get<std::string>("topics.topic_global_pose_pub", "");
    topic_imu_data_pub = pt.get<std::string>("topics.topic_imu_data_pub", "");
    topic_pose_pub = pt.get<std::string>("topics.topic_pose_pub", "");

    b_difop = pt.get<bool>("topics.b_difop", false);
    topic_lidar_pub = pt.get<std::string>("topics.topic_lidar_pub", "");
    topic_lidar_ori_pub = pt.get<std::string>("topics.topic_lidar_ori_pub", "");
    topic_lidar_difop_pub = pt.get<std::string>("topics.topic_lidar_difop_pub", "");

    topic_camera_pub = ReadStringArray(pt, "topics.topic_camera_pub_", b_camera);
    topic_infra_pub = ReadStringArray(pt, "topics.topic_infra_pub_", b_infra);
    topic_star_pub = ReadStringArray(pt, "topics.topic_star_pub_", b_star);

    topic_radar_pub = pt.get<std::string>("topics.topic_radar_pub", "");
    topic_radar4d_pub = ReadStringArray(pt, "topics.topic_radar4d_pub_", b_radar4d);

    // 打印其他参数...
    // clang-format on
  } catch (const std::exception& e) {
    std::cerr << "Error reading ini file: " << e.what() << std::endl;
  }
}

}  // namespace tools
}  // namespace jojo
