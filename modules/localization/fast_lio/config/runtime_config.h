#ifndef FAST_LIO_PARAMS_H
#define FAST_LIO_PARAMS_H

#pragma once

#include "modules/common/config/config_file_base.h"

namespace jojo {
namespace localization {

class RuntimeConfig : public jojo::common::config::ConfigFileBase {
 public:
  using jojo::common::config::ConfigFileBase::ConfigFileBase;

  void LoadConfig(const std::string& config_path) override;

 public:
  std::string root_path;
  std::string file_name;

  std::string lidar_calib_file_path;
  std::string camera_calib_file_path;
  std::string vehicle_config_file_path;

  bool b_save_pcd = false;
  bool b_only_times;  // 是否只输出时间戳
};

}  // namespace tools
}  // namespace jojo

#endif
