#ifndef ROS1_SRC_TOOLS_FAST_CALIB_INCLUDE_FAST_CALIB_ROS1_BAG_LOADER_H_
#define ROS1_SRC_TOOLS_FAST_CALIB_INCLUDE_FAST_CALIB_ROS1_BAG_LOADER_H_

#include <cstddef>
#include <string>

#include <opencv2/core.hpp>

#include "fast_calib/fast_calib.h"

namespace jojo {
namespace tools {
namespace fast_calib {
namespace ros1 {

struct InputData {
  cv::Mat image;
  pcl::PointCloud<PointXYZRing>::Ptr cloud{
      new pcl::PointCloud<PointXYZRing>};
  LidarType lidar_type = LidarType::kUnknown;
  std::string message_type;
  std::size_t message_count = 0;
};

bool LoadInputData(const Params& params,
                   InputData* data,
                   std::string* error);

}  // namespace ros1
}  // namespace fast_calib
}  // namespace tools
}  // namespace jojo

#endif  // ROS1_SRC_TOOLS_FAST_CALIB_INCLUDE_FAST_CALIB_ROS1_BAG_LOADER_H_
