#ifndef READ_VEHICLE_PARAMS_H
#define READ_VEHICLE_PARAMS_H

#pragma once

#include <fstream>  // 用于 std::ifstream
#include <iostream>
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

  bool LoadFromFile(const std::string& config_file);

  float RONI_min_x = 0.0f;
  float RONI_max_x = 0.0f;
  float RONI_min_y = 0.0f;
  float RONI_max_y = 0.0f;
  float RONI_min_z = 0.0f;
  float RONI_max_z = 0.0f;

 private:
  std::string LoadPath = "";
};

}  // namespace config
}  // namespace perception
}  // namespace jojo

#endif
