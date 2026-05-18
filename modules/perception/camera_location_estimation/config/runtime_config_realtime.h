#ifndef CAMERA_DETECTION_ESTIMATION_CONFIG_REALTIME_H
#define CAMERA_DETECTION_ESTIMATION_CONFIG_REALTIME_H

#pragma once

#include <iostream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/optional.hpp>

#include "modules/common/config/config_file_base.h"

namespace jojo {
namespace perception {
namespace cle {

class RuntimeConfig : public jojo::common::config::ConfigFileBase {
 public:
  using jojo::common::config::ConfigFileBase::ConfigFileBase;

  void LoadConfig(const std::string& config_path) override;

 public:
  std::string wts_file;
  std::string engine_file;
  std::string onnx_file;

  bool b_compressed = true;
  bool b_undistort;
  std::string calib_file_path = "";

  std::string image_topic = "";
  std::string lidar_topic = "";

  int rate = 10;

  int dist_threshold = 100;

  int use_det_or_track;
};

}  // namespace cle
}  // namespace perception
}  // namespace jojo

#endif
