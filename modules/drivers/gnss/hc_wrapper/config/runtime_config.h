#pragma once

#include "modules/common/config/config_file_base.h"

namespace jojo {
namespace drivers {

class RuntimeConfig : public jojo::common::config::ConfigFileBase {
 public:
  using jojo::common::config::ConfigFileBase::ConfigFileBase;

  void LoadConfig(const std::string& config_path) override;

  std::string GetVehicleName() const { return vehicle_name; }

 public:
  std::string vehicle_name;

  int id = -1;
  std::string type;
  std::string dev_addr;
  uint32_t baudrate;
  std::string host;
  int port;

  int frame_rate;
  std::string frame_id;
  std::string channel_name;

  bool pub_imu;
  bool save;
};

}  // namespace drivers
}  // namespace jojo