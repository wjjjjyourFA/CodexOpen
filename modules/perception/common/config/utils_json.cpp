#include "modules/perception/common/config/utils_json.h"

#include <fstream>
#include <iostream>

namespace jojo {
namespace perception {
namespace config {

bool ReadJsonFile(const std::string& filename, nlohmann::json* root) {
  if (root == nullptr) {
    return false;
  }
  std::ifstream input(filename);
  if (!input) {
    std::cerr << "Cannot open json file: " << filename << std::endl;
    return false;
  }
  input >> *root;
  return true;
}

bool ReadMatrix3f(const nlohmann::json& value, Eigen::Matrix3f* matrix) {
  if (matrix == nullptr || !value.is_array() || value.size() != 3) {
    return false;
  }
  Eigen::Matrix3f parsed;
  for (int row = 0; row < 3; ++row) {
    if (!value.at(row).is_array() || value.at(row).size() != 3) {
      return false;
    }
    for (int col = 0; col < 3; ++col) {
      parsed(row, col) = value.at(row).at(col).get<float>();
    }
  }
  *matrix = parsed;
  return true;
}

bool ParseExtrinsicMatrix(const nlohmann::json& extrinsic,
                          const std::string& filename,
                          Eigen::Matrix4f* extrinsic_matrix) {
  if (extrinsic_matrix == nullptr) {
    return false;
  }

  // 读取 R
  Eigen::Matrix3f rotation;
  if (!extrinsic.contains("rotation") ||
      !ReadMatrix3f(extrinsic.at("rotation"), &rotation)) {
    std::cerr << "Invalid rotation format in " << filename << std::endl;
    return false;
  }

  // 读取 T
  // 原始单位是 millimeter，这里统一转成 millimeter
  // 如果后续代码一直用米，可去掉 *1000.0
  if (!extrinsic.contains("translation")) {
    std::cerr << "Missing translation in " << filename << std::endl;
    return false;
  }
  const auto& translation_json = extrinsic.at("translation");
  if (!translation_json.is_array() || translation_json.size() != 3) {
    std::cerr << "Invalid translation format in " << filename << std::endl;
    return false;
  }

  // 转换单位
  const std::string unit = extrinsic.value("unit", "meter");
  float unit_scale = 1.0f;
  if (unit == "millimeter") {
    unit_scale = 1.0f / 1000.0f;
  } else if (unit != "meter") {
    std::cerr << "Unsupported translation unit: " << unit << std::endl;
    return false;
  }

  Eigen::Matrix4f parsed = Eigen::Matrix4f::Identity();
  parsed.block<3, 3>(0, 0) = rotation;
  for (int row = 0; row < 3; ++row) {
    parsed(row, 3) = translation_json.at(row).get<float>() * unit_scale;
  }
  *extrinsic_matrix = parsed;
  return true;
}

}  // namespace config
}  // namespace perception
}  // namespace jojo
