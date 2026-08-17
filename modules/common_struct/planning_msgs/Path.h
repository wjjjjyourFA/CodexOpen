#pragma once

#include <vector>

#include "modules/common_struct/basic_msgs/Header.h"
#include "modules/common_struct/localization_msgs/PoseStamp.h"

namespace jojo {
namespace common_struct {

struct Path {
  Header header;
  std::vector<PoseStamped> poses;
};

}  // namespace common_struct
}  // namespace jojo
