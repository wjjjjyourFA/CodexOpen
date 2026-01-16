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
  // Which network interface (combination of port & IP) are we reading from?
  char interface[16] = "eth0";
  // What simple packet filters are we applying?
  char filter[256] = "";
  // Radar Default
  int packets = 0, id = 0, port = 31122;
  // Based on if we're using live capture or from offline pcap doc
  int capture_live = 1;
  // File path to pcap doc if using offline capture
  char capture_path[256] = "";

  std::string frame_id;
  std::string channel_name;
};

}  // namespace drivers
}  // namespace jojo