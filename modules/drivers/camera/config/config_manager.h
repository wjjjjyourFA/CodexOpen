#ifndef GMSL_CAMERA_CONFIG_H
#define GMSL_CAMERA_CONFIG_H

#pragma once

#include <iostream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/optional.hpp>

#include "modules/common/config/config_file_base.h"
#include "modules/drivers/camera/config/vehicle_model_config.h"

namespace jojo {
namespace drivers {
namespace config = jojo::common::config;

class ConfigManager : public config::ConfigFileBase {
 public:
  using config::ConfigFileBase::ConfigFileBase;

  void LoadConfig(const std::string& config_path) override;

 public:
  VehicleModelConfig vehicle_model_config;

  std::string GetVehicleName() const {
    return vehicle_model_config.vehicle_name;
  }

  int GetCompressRatio() const {
    return vehicle_model_config.default_compress_ratio;
  }
};

}  // namespace drivers
}  // namespace jojo

#endif  // GMSL_CAMERA_CONFIG_H
