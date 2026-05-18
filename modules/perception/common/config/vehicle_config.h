#ifndef READ_VEHICLE_PARAMS_H
#define READ_VEHICLE_PARAMS_H

#pragma once

#include <iostream>
#include <fstream>  // 用于 std::ifstream
#include <memory>
#include <vector>

namespace jojo {
namespace perception {
namespace config {

class VehicleConfig {
 public:
  VehicleConfig() {};
  virtual ~VehicleConfig() = default;

  void SetLoadPath(const std::string& load_path);

  void LoadFromFile(const std::string& config_file);

  float RONI_min_x;
  float RONI_max_x;
  float RONI_min_y;
  float RONI_max_y;
  float RONI_min_z;
  float RONI_max_z;

 private:
  std::string LoadPath = "";
};

}  // namespace config
}  // namespace perception
}  // namespace jojo

#endif