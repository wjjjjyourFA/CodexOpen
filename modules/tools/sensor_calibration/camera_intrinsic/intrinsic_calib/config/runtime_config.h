#ifndef PARAMS_H
#define PARAMS_H

#pragma once

#include <iostream>

#include "modules/common/config/config_file_base.h"

namespace jojo {
namespace tools {

class RuntimeConfig : public jojo::common::config::ConfigFileBase {
 public:
  using jojo::common::config::ConfigFileBase::ConfigFileBase;

  void LoadConfig(const std::string& config_path) override;
  
 public:
  std::string data_root_path =
      "./../../../data/SensorCalibration/IntrinsicCalib";

  int grid_size     = 50;
  int corner_width  = 15;
  int corner_height = 17;

  float IMAGE_MARGIN_PERCENT                      = 0.1;
  float CHESSBOARD_MIN_AREA_PORTION               = 0.04;
  float CHESSBOARD_MIN_ANGLE                      = 40.0;
  float CHESSBOARD_MIN_MOVE_THRESH                = 0.08;
  float CHESSBOARD_MIN_ANGLE_CHANGE_THRESH        = 5.0;
  float CHESSBOARD_MIN_AREA_CHANGE_PARTION_THRESH = 0.005;
  float MAX_SELECTED_SAMPLE_NUM                   = 45;
  float REGION_MAX_SELECTED_SAMPLE_NUM            = 15;
};

}  // namespace tools
}  // namespace jojo

#endif
