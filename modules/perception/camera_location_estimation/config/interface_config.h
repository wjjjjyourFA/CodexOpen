#ifndef CAMERA_LOCATION_ESTIMATION_INTERFACE_CONFIG_H
#define CAMERA_LOCATION_ESTIMATION_INTERFACE_CONFIG_H

#pragma once

#include <iostream>

#include <boost/optional.hpp>
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

 public:
  bool b_compressed = true;

  std::string image_topic = "";
  std::string lidar_topic = "";

  int rate = 10;
};

}  // namespace cle
}  // namespace perception
}  // namespace jojo

#endif
