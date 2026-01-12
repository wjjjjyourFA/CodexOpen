// # This represents a vector in free space. 

// Pose6DInt pose

// int32 speed

#pragma once

#include "modules/common_struct/basic_msgs/Pose6D.h"

namespace jojo {
namespace common_struct {
  
struct Pose7D {
  Pose6D pose;

  double speed;
};

}  // namespace common_struct
}  // namespace jojo