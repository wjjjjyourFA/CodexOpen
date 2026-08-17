#pragma once

#include <vector>

#include "modules/common_struct/basic_msgs/Header.h"
#include "modules/common_struct/basic_msgs/VectorPoint.h"

namespace jojo {
namespace common_struct {

struct Twist {
  Vector3d linear;
  Vector3d angular;
};

struct PointStamped {
  Header header;
  Vector3d point;
};

struct PolygonStamped {
  Header header;
  std::vector<Vector3f> points;
};

}  // namespace common_struct
}  // namespace jojo
