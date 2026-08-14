#ifndef LIDAR2CAMERA_CONFIG_INTERFACE_CONFIG_H
#define LIDAR2CAMERA_CONFIG_INTERFACE_CONFIG_H

#pragma once

#include <iostream>

#include "modules/common/config/config_file_base.h"

namespace jojo {
namespace perception {

class InterfaceConfig : public jojo::common::config::ConfigFileBase {
 public:
  using jojo::common::config::ConfigFileBase::ConfigFileBase;

  void LoadConfig(const std::string& config_path) override;

  void InitPrefixPath();

 public:
  std::string data_root_path = "./../data/PerceptionFuse";

  bool b_matched = 0;

  bool b_bin_or_pcd = 0;
  bool b_jpg_or_png = 0;

  std::string lidar_file = "";
  std::string image_file = "";

 private:
  std::string lidar_name = "";
  std::string image_name = "";

 public:
  bool b_compressed = true;

  std::string image_topic = "";
  std::string lidar_topic = "";

  int rate = 10;
};

}  // namespace perception
}  // namespace jojo

#endif
