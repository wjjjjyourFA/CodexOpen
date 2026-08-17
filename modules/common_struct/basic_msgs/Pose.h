#pragma once

#include "modules/common_struct/basic_msgs/Quaternion.h"
#include "modules/common_struct/basic_msgs/VectorPoint.h"

namespace jojo {
namespace common_struct {

struct Pose {
  Vector3d position;
  Quaternion orientation;
};

}  // namespace common_struct
}  // namespace jojo
