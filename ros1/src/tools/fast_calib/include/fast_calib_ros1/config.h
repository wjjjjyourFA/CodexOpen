#ifndef ROS1_SRC_TOOLS_FAST_CALIB_INCLUDE_FAST_CALIB_ROS1_CONFIG_H_
#define ROS1_SRC_TOOLS_FAST_CALIB_INCLUDE_FAST_CALIB_ROS1_CONFIG_H_

#include <string>

#include <ros/node_handle.h>

#include "fast_calib/fast_calib.h"

namespace jojo {
namespace tools {
namespace fast_calib {
namespace ros1 {

struct InterfaceParams {
  bool debug_enabled = true;
  std::string debug_frame_id = "map";
  double debug_publish_rate_hz = 1.0;
  int cloud_queue_size = 1;
  int center_queue_size = 10;
  std::string qr_cloud_topic = "qr_cloud";
  std::string center_cloud_topic = "center_cloud";
  std::string filtered_cloud_topic = "filtered_cloud";
  std::string plane_cloud_topic = "plane_cloud";
  std::string aligned_cloud_topic = "aligned_cloud";
  std::string edge_cloud_topic = "edge_cloud";
  std::string center_z0_cloud_topic = "center_z0_cloud";
  std::string aligned_lidar_centers_topic = "aligned_lidar_centers";
  std::string colored_cloud_topic = "colored_cloud";
};

bool ReadParameters(const ros::NodeHandle& private_node,
                    Params* params,
                    InterfaceParams* interface,
                    std::string* error);

}  // namespace ros1
}  // namespace fast_calib
}  // namespace tools
}  // namespace jojo

#endif  // ROS1_SRC_TOOLS_FAST_CALIB_INCLUDE_FAST_CALIB_ROS1_CONFIG_H_
