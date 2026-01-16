#include "vehicle_model_config.h"

void VehicleModelConfig::load_configs(const std::string& yaml_file,
                                      const std::string& prefix) {
  YAML::Node config = YAML::LoadFile(yaml_file);

  if (config["vehicle_model"]["name"]) {
    vehicle_name = config["vehicle_model"]["name"].as<std::string>();
  }

  load_gnss_conf(config, prefix);  // 读取 gnss 配置
};

void VehicleModelConfig::load_gnss_conf(YAML::Node& config,
                                        const std::string& prefix) {
  // 检查 "sensors" -> "gnss" 是否存在
  if (config["sensors"] && config["sensors"]["gnss"]) {
    const YAML::Node& gnss_receivers = config["sensors"]["gnss"];

    // 获取相机的数量
    int num_gnss = gnss_receivers["num"].as<int>();

    // 遍历 "gnss" 中的每个项 (如 gnss1, gnss2, gnss3)
    for (int i = 1; i <= num_gnss; ++i) {
      std::string gnss_name = "gnss_" + std::to_string(i);
      if (gnss_receivers[gnss_name]) {
        SensorConfig gnss;

        std::string prefix_ = prefix + "/" + vehicle_name + "/";
        // 提取 config_file 和 calibration_file
        if (gnss_receivers[gnss_name]["config_file"]) {
          gnss.config_file =
              prefix_ +
              gnss_receivers[gnss_name]["config_file"].as<std::string>();
        }
        if (gnss_receivers[gnss_name]["calibration_file"]) {
          gnss.calibration_file =
              prefix_ +
              gnss_receivers[gnss_name]["calibration_file"].as<std::string>();
        }
        // std::cout << "gnss.config_file: " << gnss.config_file
        //           << std::endl;

        // 将读取的配置添加到配置列表中
        gnss_configs.push_back(gnss);
      } else {
        std::cerr << "gnss " << gnss_name << " configuration missing."
                  << std::endl;
      }
    }
  } else {
    std::cerr << "No sensors or gnss section found in the YAML file."
              << std::endl;
  }
};