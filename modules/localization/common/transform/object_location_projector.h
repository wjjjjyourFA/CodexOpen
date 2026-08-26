#ifndef GLOBAL_LOCATION_PROJECTOR_H
#define GLOBAL_LOCATION_PROJECTOR_H

#include <string>

#include <Eigen/Core>

namespace jojo {
namespace localization {
namespace common {

struct GlobalLocatedObject {
  std::size_t source_index = 0U;

  Eigen::Vector3f center_sensor = Eigen::Vector3f::Zero();
  Eigen::Vector3d center_global = Eigen::Vector3d::Zero();

  base::BBox3DRotated box_global;

  bool global_valid = false;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct GlobalLocationResult {
  std::vector<GlobalLocatedObject> objects;

  std::uint64_t timestamp_ns = 0U;
  std::string source_frame;
  std::string target_frame;
  std::string error;
};

class GlobalLocationProjector {};

}  // namespace common
}  // namespace localization
}  // namespace jojo

#endif