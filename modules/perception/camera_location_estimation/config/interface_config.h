#ifndef CAMERA_LOCATION_ESTIMATION_INTERFACE_CONFIG_H
#define CAMERA_LOCATION_ESTIMATION_INTERFACE_CONFIG_H

#include <string>

#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include "modules/common/config/config_file_base.h"

namespace jojo {
namespace perception {
namespace cle {

class InterfaceConfig : public jojo::common::config::ConfigFileBase {
 public:
  using jojo::common::config::ConfigFileBase::ConfigFileBase;
  void LoadConfig(const std::string& config_path) override;
  bool Validate(std::string* error = nullptr) const;

  bool b_compressed = true;
  std::string image_topic;
  std::string lidar_topic;
  std::string gnss_topic;
  std::string odom_topic;

  int rate = 10;

  bool valid = false;
  std::string validation_error;
};

}  // namespace cle
}  // namespace perception
}  // namespace jojo
#endif
