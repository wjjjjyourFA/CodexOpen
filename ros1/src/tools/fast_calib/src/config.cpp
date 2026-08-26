#include "fast_calib_ros1/config.h"

#include <sstream>

namespace jojo {
namespace tools {
namespace fast_calib {
namespace ros1 {

bool ReadParameters(const ros::NodeHandle& private_node,
                    Params* params,
                    InterfaceParams* interface,
                    std::string* error) {
  if (params == nullptr || interface == nullptr) {
    if (error != nullptr) {
      *error = "parameter destination is null";
    }
    return false;
  }

  private_node.param("fx", params->fx, params->fx);
  private_node.param("fy", params->fy, params->fy);
  private_node.param("cx", params->cx, params->cx);
  private_node.param("cy", params->cy, params->cy);
  private_node.param("k1", params->k1, params->k1);
  private_node.param("k2", params->k2, params->k2);
  private_node.param("p1", params->p1, params->p1);
  private_node.param("p2", params->p2, params->p2);
  private_node.param("marker_size", params->marker_size, params->marker_size);
  private_node.param("delta_width_qr_center",
                     params->delta_width_qr_center,
                     params->delta_width_qr_center);
  private_node.param("delta_height_qr_center",
                     params->delta_height_qr_center,
                     params->delta_height_qr_center);
  private_node.param("delta_width_circles", params->delta_width_circles,
                     params->delta_width_circles);
  private_node.param("delta_height_circles", params->delta_height_circles,
                     params->delta_height_circles);
  private_node.param("circle_radius", params->circle_radius,
                     params->circle_radius);
  private_node.param("min_detected_markers", params->min_detected_markers,
                     params->min_detected_markers);
  private_node.param("x_min", params->x_min, params->x_min);
  private_node.param("x_max", params->x_max, params->x_max);
  private_node.param("y_min", params->y_min, params->y_min);
  private_node.param("y_max", params->y_max, params->y_max);
  private_node.param("z_min", params->z_min, params->z_min);
  private_node.param("z_max", params->z_max, params->z_max);
  private_node.param("image_path", params->image_path, params->image_path);
  private_node.param("bag_path", params->bag_path, params->bag_path);
  private_node.param("lidar_topic", params->lidar_topic,
                     params->lidar_topic);
  private_node.param("output_path", params->output_path,
                     params->output_path);

  private_node.param("debug/enabled", interface->debug_enabled,
                     interface->debug_enabled);
  private_node.param("debug/frame_id", interface->debug_frame_id,
                     interface->debug_frame_id);
  private_node.param("debug/publish_rate_hz",
                     interface->debug_publish_rate_hz,
                     interface->debug_publish_rate_hz);
  private_node.param("debug/cloud_queue_size", interface->cloud_queue_size,
                     interface->cloud_queue_size);
  private_node.param("debug/center_queue_size", interface->center_queue_size,
                     interface->center_queue_size);
  private_node.param("topics/qr_cloud", interface->qr_cloud_topic,
                     interface->qr_cloud_topic);
  private_node.param("topics/center_cloud", interface->center_cloud_topic,
                     interface->center_cloud_topic);
  private_node.param("topics/filtered_cloud",
                     interface->filtered_cloud_topic,
                     interface->filtered_cloud_topic);
  private_node.param("topics/plane_cloud", interface->plane_cloud_topic,
                     interface->plane_cloud_topic);
  private_node.param("topics/aligned_cloud", interface->aligned_cloud_topic,
                     interface->aligned_cloud_topic);
  private_node.param("topics/edge_cloud", interface->edge_cloud_topic,
                     interface->edge_cloud_topic);
  private_node.param("topics/center_z0_cloud",
                     interface->center_z0_cloud_topic,
                     interface->center_z0_cloud_topic);
  private_node.param("topics/aligned_lidar_centers",
                     interface->aligned_lidar_centers_topic,
                     interface->aligned_lidar_centers_topic);
  private_node.param("topics/colored_cloud", interface->colored_cloud_topic,
                     interface->colored_cloud_topic);

  if (params->fx <= 0.0 || params->fy <= 0.0 || params->marker_size <= 0.0 ||
      params->delta_width_circles <= 0.0 ||
      params->delta_height_circles <= 0.0 || params->circle_radius <= 0.0 ||
      params->min_detected_markers < 1 ||
      params->min_detected_markers > kTargetCircleCount ||
      params->x_min >= params->x_max || params->y_min >= params->y_max ||
      params->z_min >= params->z_max || params->image_path.empty() ||
      params->bag_path.empty() || params->lidar_topic.empty() ||
      params->output_path.empty()) {
    if (error != nullptr) {
      *error = "calibration parameters are invalid or incomplete";
    }
    return false;
  }
  if (interface->debug_enabled &&
      (interface->debug_publish_rate_hz <= 0.0 ||
       interface->cloud_queue_size <= 0 ||
       interface->center_queue_size <= 0 ||
       interface->debug_frame_id.empty())) {
    if (error != nullptr) {
      *error = "debug publisher parameters are invalid";
    }
    return false;
  }
  return true;
}

}  // namespace ros1
}  // namespace fast_calib
}  // namespace tools
}  // namespace jojo
