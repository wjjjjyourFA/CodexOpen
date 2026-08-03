#ifndef ROSBAG_UTILS_H
#define ROSBAG_UTILS_H

#include <algorithm>
#include <cstring>

#include "string"

namespace jojo {
namespace tools {

std::string FormatRosbagTime(const std::string& rosbag_name);

}  // namespace tools
}  // namespace jojo

#endif  // ROSBAG_UTILS_H