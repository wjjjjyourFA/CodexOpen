#include "modules/drivers/camera/config/vehicle_model_config.h"

void VehicleModelConfig::load_configs(const std::string& yaml_file,
                                      const std::string& prefix) {
  YAML::Node config = YAML::LoadFile(yaml_file);

  if (config["vehicle_model"]["name"]) {
    vehicle_name = config["vehicle_model"]["name"].as<std::string>();
  }

  load_camera_conf(config, prefix);  // 读取 camera 配置
}

void VehicleModelConfig::load_camera_conf(YAML::Node& config,
                                          const std::string& prefix) {
  // 检查 "sensors" -> "camera" 是否存在
  if (config["sensors"] && config["sensors"]["camera"]) {
    const YAML::Node& cameras = config["sensors"]["camera"];

    // 获取图像的压缩比
    default_compress_ratio = cameras["compress_ratio"].as<int>();
    int num_cameras = cameras["num"].as<int>();  // 获取相机的数量

    // 遍历 "camera" 中的每个项 (如 camera_1, camera_2, camera_3)
    /* 键值法
      for (YAML::const_iterator it = cameras.begin(); it != cameras.end();
           ++it) {
        // 检查是否是一个 map
        if (it->second.IsMap()) {
          SensorConfig camera;
          // 提取 config_file 和 calibration_file
          if (it->second["config_file"]) {
            camera.config_file = it->second["config_file"].as<std::string>();
          }
          if (it->second["calibration_file"]) {
            camera.calibration_file =
                it->second["calibration_file"].as<std::string>();
          }
          // 实际路径还需要加上 prefix_root

          // 将读取的配置添加到配置列表中
          camera_configs.push_back(camera);
        } else {
          std::cerr << "Invalid configuration for "
                    << it->first.as<std::string>() << std::endl;
        }
      } */

    for (int i = 1; i <= num_cameras; ++i) {
      std::string camera_name = "camera_" + std::to_string(i);
      if (cameras[camera_name]) {
        SensorConfig camera;

        // 确保 resize 数组有两个元素
        if (cameras[camera_name]["resize"].size() == 2) {
          // 第一个值为 width
          camera.width = cameras[camera_name]["resize"][0].as<int>();
          // 第二个值为 height
          camera.height = cameras[camera_name]["resize"][1].as<int>();
          // std::cout << "Width: " << camera.width
          //           << ", Height: " << camera.height << std::endl;
          // 获取图像的压缩比
          // camera.compress_ratio = cameras["compress_ratio"].as<int>();
        } else {
          std::cerr << "Invalid resize array!" << std::endl;
        }

        std::string prefix_ = prefix + "/" + vehicle_name + "/";
        // 提取 config_file 和 calibration_file
        if (cameras[camera_name]["config_file"]) {
          camera.config_file =
              prefix_ + cameras[camera_name]["config_file"].as<std::string>();
        }
        if (cameras[camera_name]["calibration_file"]) {
          camera.calibration_file =
              prefix_ +
              cameras[camera_name]["calibration_file"].as<std::string>();
        }
        // std::cout << "camera.config_file: " << camera.config_file
        //           << std::endl;
        // std::cout << "camera.calibration_file: " << camera.calibration_file
        //           << std::endl;

        // 将读取的配置添加到配置列表中
        camera_configs.push_back(camera);
      } else {
        std::cerr << "Camera " << camera_name << " configuration missing."
                  << std::endl;
      }
    }
  } else {
    std::cerr << "No sensors or camera section found in the YAML file."
              << std::endl;
  }
}
