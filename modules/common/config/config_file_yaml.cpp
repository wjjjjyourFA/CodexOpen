#include "modules/common/config/config_file_yaml.h"

namespace jojo {
namespace common {
namespace config {

Eigen::Vector3d ConfigFileYaml::Vec3FromYaml(const YAML::Node& node) {
  std::vector<double> v = node.as<std::vector<double>>();
  if (v.size() != 3) {
    throw std::runtime_error("Expected 3 elements for Vector3d");
  }

  // v.size() == 3
  // v 是连续内存（vector满足）
  // auto v = node.as<std::vector<double>>();
  // auto r = Eigen::Map<Eigen::Vector3d>(v.data());

  return Eigen::Vector3d(v[0], v[1], v[2]);
}

}  // namespace config
}  // namespace common
}  // namespace jojo