#ifndef MAP_CENTER_VIEW_PARAMS_H
#define MAP_CENTER_VIEW_PARAMS_H

#pragma once

#include "modules/common/config/config_file_base.h"

namespace jojo {
namespace dreamview {

class RuntimeConfig : public jojo::common::config::ConfigFileBase {
 public:
  using jojo::common::config::ConfigFileBase::ConfigFileBase;

  void LoadConfig(const std::string& config_path) override;
};

}  // namespace dreamview
}  // namespace jojo

#endif
