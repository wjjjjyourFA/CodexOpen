#include "vehicle_model_config.h"

void VehicleModelConfig::load_configs(const std::string& yaml_file,
                                      const std::string& prefix) {
  YAML::Node config = YAML::LoadFile(yaml_file);

  if (config["vehicle_model"]["name"]) {
    vehicle_name = config["vehicle_model"]["name"].as<std::string>();
  }

  load_lidar_conf(config, prefix);  // 读取 lidar 配置
};

void VehicleModelConfig::load_lidar_conf(YAML::Node& config,
                                         const std::string& prefix) {
  // 检查 "sensors" -> "lidar" 是否存在
  if (config["sensors"] && config["sensors"]["lidar"]) {
    const YAML::Node& lidars = config["sensors"]["lidar"];

    int num_lidars = lidars["num"].as<int>();

    // 遍历 "lidar" 中的每个项 (如 lidar_1, lidar_2, lidar_3)
    for (int i = 1; i <= num_lidars; ++i) {
      std::string lidar_name = "lidar_" + std::to_string(i);
      if (lidars[lidar_name]) {
        SensorConfig lidar;

        std::string prefix_ = prefix + "/" + vehicle_name + "/";
        // 提取 config_file 和 calibration_file
        if (lidars[lidar_name]["config_file"]) {
          lidar.config_file =
              prefix_ + lidars[lidar_name]["config_file"].as<std::string>();
        }
        // std::cout << "lidar.config_file: " << lidar.config_file
        //           << std::endl;

        // 将读取的配置添加到配置列表中
        lidar_configs.push_back(lidar);
      } else {
        std::cerr << "lidar " << lidar_name << " configuration missing."
                  << std::endl;
      }
    }
  } else {
    std::cerr << "No sensors or lidar section found in the YAML file."
              << std::endl;
  }
};

void update_config(YAML::Node& config, const std::string& ns) {
  if (!config["lidar"] || !config["lidar"].IsSequence()) {
    std::cerr << "Invalid or missing 'lidar' configuration." << std::endl;
    return;
  }

  // 在 yaml-cpp 中，YAML::Node 是引用语义的类（内部使用 shared_ptr 管理），
  // 所以你不需要用 &
  YAML::Node lidar_array = config["lidar"];

  for (std::size_t i = 0; i < lidar_array.size(); ++i) {
    // 修改 lidar_node 时也会反映到 config 上
    YAML::Node lidar_node = lidar_array[i];

    if (!lidar_node.IsMap()) continue;

    // 修改 driver.frame_id 为 ns_lidar{i}
    // if (lidar_node["driver"] && lidar_node["driver"]["frame_id"]) {
    //   lidar_node["driver"]["frame_id"] = ns + "_lidar" + std::to_string(i);
    // }

    // 修改 ros 相关字段
    if (lidar_node["ros"] && lidar_node["ros"].IsMap()) {
      YAML::Node ros_info = lidar_node["ros"];

      // 修改 ros.ros_recv_packet_topic 加上命名空间前缀
      if (ros_info["ros_recv_packet_topic"]) {
        std::string original_topic =
            ros_info["ros_recv_packet_topic"].as<std::string>();
        ros_info["ros_recv_packet_topic"] = "/" + ns + original_topic;
      }

      if (ros_info["ros_send_packet_topic"]) {
        std::string original =
            ros_info["ros_send_packet_topic"].as<std::string>();
        ros_info["ros_send_packet_topic"] = "/" + ns + original;
      }

      if (ros_info["ros_send_point_cloud_topic"]) {
        std::string original =
            ros_info["ros_send_point_cloud_topic"].as<std::string>();
        ros_info["ros_send_point_cloud_topic"] = "/" + ns + original;
      }
    }
  }

  // debug 打印验证结果
  // for (std::size_t i = 0; i < config["lidar"].size(); ++i) {
  //   std::cout << config["lidar"][i]["driver"] << std::endl;
  //   std::cout << "========================== " << std::endl;
  // }
}
