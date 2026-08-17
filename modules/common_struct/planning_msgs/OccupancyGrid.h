#pragma once

#include <cstdint>
#include <vector>

#include "modules/common_struct/basic_msgs/Header.h"
#include "modules/common_struct/basic_msgs/Pose.h"

namespace jojo {
namespace common_struct {

struct MapMetaData {
  std::uint64_t map_load_time{0};
  float resolution{0.0F};
  std::uint32_t width{0};
  std::uint32_t height{0};
  Pose origin;
};

struct OccupancyGrid {
  Header header;
  MapMetaData info;
  std::vector<std::int8_t> data;
};

}  // namespace common_struct
}  // namespace jojo
