#ifndef PERCEPTION_COMMON_CONFIG_UTILS_JSON_H
#define PERCEPTION_COMMON_CONFIG_UTILS_JSON_H

#pragma once

#include <string>

#include <Eigen/Dense>
#include <nlohmann/json.hpp>

namespace jojo {
namespace perception {
namespace config {

bool ReadJsonFile(const std::string& filename, nlohmann::json* root);

bool ReadMatrix3f(const nlohmann::json& value, Eigen::Matrix3f* matrix);

bool ParseExtrinsicMatrix(const nlohmann::json& extrinsic,
                          const std::string& filename,
                          Eigen::Matrix4f* extrinsic_matrix);

}  // namespace config
}  // namespace perception
}  // namespace jojo

#endif  // PERCEPTION_COMMON_CONFIG_UTILS_JSON_H
