#include "tools/data_processor/config/interface_config.h"

#include <stdexcept>

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

    topic_radar_sub = pt.get<std::string>("topics.topic_radar_sub", "");
    topic_radar4d_sub = ReadStringArray(pt, "topics.topic_radar4d_sub_", b_radar4d);

    if (b_radar < 0 || b_radar4d < 0 || b_camera < 0 || b_infra < 0 ||
        b_star < 0) {
      throw std::invalid_argument("sensor counts must be non-negative");
    }
    if (topic_camera_sub.size() != static_cast<size_t>(b_camera) ||
        topic_infra_sub.size() != static_cast<size_t>(b_infra) ||
        topic_star_sub.size() != static_cast<size_t>(b_star) ||
        topic_radar4d_sub.size() != static_cast<size_t>(b_radar4d)) {
      throw std::invalid_argument(
          "enabled sensor count does not match configured topic count");
    }

    // 打印其他参数...
    // clang-format on
  } catch (const std::exception& e) {
    throw std::runtime_error("failed to load interface config '" + config_path +
                             "': " + e.what());
  }
}

}  // namespace tools
}  // namespace jojo
