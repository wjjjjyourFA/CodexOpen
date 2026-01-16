#ifndef VEHICLE_MODEL_CONFIG_H
#define VEHICLE_MODEL_CONFIG_H

#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>

class SensorConfig {
 public:
  std::string config_file;
  std::string calibration_file;
};

class VehicleModelConfig {
 public:
  VehicleModelConfig() = default;
  ~VehicleModelConfig() = default;

  std::string vehicle_name = "";
  std::vector<SensorConfig> lidar_configs;

  void load_configs(const std::string& yaml_file, const std::string& prefix);

  void load_lidar_conf(YAML::Node& config, const std::string& prefix);
};

void update_config(YAML::Node& config, const std::string& ns);

#endif  // VEHICLE_MODEL_CONFIG_H
