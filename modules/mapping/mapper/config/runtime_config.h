#ifndef MAPPER_PARAMS_H
#define MAPPER_PARAMS_H

#pragma once

#include "modules/common/config/config_file_base.h"

namespace jojo {
namespace mapping {

class RuntimeConfig : public jojo::common::config::ConfigFileBase {
 public:
  using jojo::common::config::ConfigFileBase::ConfigFileBase;

  void LoadConfig(const std::string& config_path) override;

  bool b_generate_map3d     = false;
  bool b_generate_color_map = false;

  // 单位 m
  float map_resolution    = 0.2;
  float sampling_distance = 2.0;

  bool b_use_time_interpolate = false;

  bool b_realtime_show = false;

  std::string camera_calib_file_path = "";
};

}  // namespace mapping
}  // namespace jojo

#endif
