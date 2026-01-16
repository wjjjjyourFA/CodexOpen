#pragma once

#include <iostream>

#include "modules/common/config/config_file_base.h"

namespace jojo {
namespace drivers {

class RuntimeConfig : public jojo::common::config::ConfigFileBase {
 public:
  using jojo::common::config::ConfigFileBase::ConfigFileBase;

  void LoadConfig(const std::string& config_path) override;

  std::string GetVehicleName() const { return vehicle_name; };

 public:
  std::string vehicle_name;

  std::string type;
  std::string group_ip;
  std::string radar_ip;
  std::string local_ip;
  int group_port;
  int radar_port;
  int local_port;

  std::string frame_id;
  std::string channel_name;
};

}  // namespace drivers
}  // namespace jojo
