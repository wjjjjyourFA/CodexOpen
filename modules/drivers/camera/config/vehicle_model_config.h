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
  int width;
  int height;
  std::string config_file;
  std::string calibration_file;
  int compress_ratio = 95;
};

class VehicleModelConfig {
 public:
  std::string vehicle_name = "";
  std::vector<SensorConfig> camera_configs;

  void load_configs(const std::string& yaml_file, const std::string& prefix);

  void load_camera_conf(YAML::Node& config, const std::string& prefix);

  int default_compress_ratio = 95;
};

#endif  // VEHICLE_MODEL_CONFIG_H
