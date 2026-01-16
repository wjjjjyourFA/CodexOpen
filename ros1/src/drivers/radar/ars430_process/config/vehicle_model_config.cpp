#include "vehicle_model_config.h"

void VehicleModelConfig::load_configs(const std::string& yaml_file,
                                      const std::string& prefix) {
  YAML::Node config = YAML::LoadFile(yaml_file);

  if (config["vehicle_model"]["name"]) {
    vehicle_name = config["vehicle_model"]["name"].as<std::string>();
  }

  load_radar_conf(config, prefix);  // 读取 radar 配置
};

void VehicleModelConfig::load_radar_conf(YAML::Node& config,
                                         const std::string& prefix) {
  // 检查 "sensors" -> "radar" 是否存在
  if (config["sensors"] && config["sensors"]["radar"]) {
    const YAML::Node& radars = config["sensors"]["radar"];

    int num_radars = radars["num"].as<int>();

    // 遍历 "radar" 中的每个项 (如 lidar_1, lidar_2, lidar_3)
    for (int i = 1; i <= num_radars; ++i) {
      std::string radar_name = "radar_" + std::to_string(i);
      if (radars[radar_name]) {
        SensorConfig radar;

        std::string prefix_ = prefix + "/" + vehicle_name + "/";
        // 提取 config_file 和 calibration_file
        if (radars[radar_name]["config_file"]) {
          radar.config_file =
              prefix_ + radars[radar_name]["config_file"].as<std::string>();
        }
        if (radars[radar_name]["calibration_file"]) {
          radar.calibration_file =
              prefix_ +
              radars[radar_name]["calibration_file"].as<std::string>();
        }
        // std::cout << "radar.config_file: " << radar.config_file
        //           << std::endl;

        // 将读取的配置添加到配置列表中
        radar_configs.push_back(radar);
      } else {
        std::cerr << "radar " << radar_name << " configuration missing."
                  << std::endl;
      }
    }
  } else {
    std::cerr << "No sensors or radar section found in the YAML file."
              << std::endl;
  }
};