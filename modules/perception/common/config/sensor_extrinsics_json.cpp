#include "modules/perception/common/config/sensor_extrinsics_json.h"

#include <iostream>

namespace jojo {
namespace perception {
namespace config {

bool SensorExtrinsicsJson::LoadFromFileBase(const std::string& filename,
                                            Eigen::Matrix4f& extrinsic_matrix) {
  try {
    nlohmann::json root;
    if (!ReadJsonFile(filename, &root)) {
      return false;
    }
    if (!root.contains("extrinsic")) {
      std::cerr << "Missing key 'extrinsic' in " << filename << std::endl;
      return false;
    }

    Eigen::Matrix4f parsed;
    if (!ParseExtrinsicMatrix(root.at("extrinsic"), filename, &parsed)) {
      return false;
    }

    extrinsic_matrix = parsed;
    return true;
  } catch (const nlohmann::json::exception& error) {
    std::cerr << "Invalid extrinsic json '" << filename << "': " << error.what()
              << std::endl;
    return false;
  }
}

}  // namespace config
}  // namespace perception
}  // namespace jojo
