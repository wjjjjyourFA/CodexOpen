#ifndef READ_SENSOR_PARAMS_JSON_H
#define READ_SENSOR_PARAMS_JSON_H

#pragma once

#include <Eigen/Dense>

#include "modules/common/config/config_file_json.h"
#include "modules/perception/common/config/sensor_extrinsics.h"
#include "modules/perception/common/config/utils.h"

namespace jojo {
namespace perception {
namespace config {

class SensorExtrinsicsJson : public SensorExtrinsics {
 public:
  SensorExtrinsicsJson() {};
  virtual ~SensorExtrinsicsJson() = default;

 protected:
  bool LoadFromFileBase(const std::string& filename,
                        Eigen::Matrix4f& extrinsic_matrix) override;
};

}  // namespace config
}  // namespace perception
}  // namespace jojo

#endif  // READ_BASE_PARAMS_H