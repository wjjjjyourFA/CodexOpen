#ifndef GNSS_PARAMS_H
#define GNSS_PARAMS_H

#pragma once

#include <iostream>

#include "modules/common/config/config_file_base.h"

#include "vehicle_model_config.h"

namespace jojo {
namespace drivers {

class ConfigManager : public jojo::common::config::ConfigFileBase {
 public:
  using jojo::common::config::ConfigFileBase::ConfigFileBase;

  void LoadConfig(const std::string& config_path) override;

 public:
  VehicleModelConfig vehicle_model_config;

  std::string GetVehicleName() const {
    return vehicle_model_config.vehicle_name;
  };
};

}  // namespace drivers
}  // namespace jojo

#endif  // GNSS_PARAMS_H
