#include "tools/common/utils/rosbag_utils.h"

namespace jojo {
namespace tools {

std::string FormatRosbagTime(const std::string &rosbag_name) {
  /* way 1
  char day[10];
  memset(day, '\0', sizeof(day));
  strncpy(day, rosbag_name.c_str(), 10);

  char time[20];
  memset(time, '\0', sizeof(time));
  strcpy(time, rosbag_name.c_str());
  */

  // way 2
  std::string time_str;
  size_t pos = rosbag_name.find("rosbag2_");
  if (pos != std::string::npos) {
    time_str = rosbag_name.substr(pos + 8);  // 去掉 "rosbag2_"
  } else {
    time_str = rosbag_name;  // 已经是 2025-03-20-10-27-52 这种
  }

  // 把 '_' 统一替换成 '-'
  std::replace(time_str.begin(), time_str.end(), '_', '-');

  return time_str;
}

}  // namespace tools
}  // namespace jojo