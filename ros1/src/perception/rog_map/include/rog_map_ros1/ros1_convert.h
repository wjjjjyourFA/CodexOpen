#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

#include <rog_map_ros1/VizConfig.h>
#include <rog_map/rog_map.h>

namespace jojo {
namespace perception {
namespace ros1 {

class RogMapRos1Convert {
 public:
  RogMapRos1Convert(::ros::NodeHandle& node,
                    ::ros::NodeHandle& private_node);

  bool Init();

 private:
  using DynamicServer =
      dynamic_reconfigure::Server<rog_map_ros1::VizConfig>;

  bool LoadCoreConfig(rog_map::Config* config);
  void OdometryCallback(const nav_msgs::OdometryConstPtr& message);
  void CloudCallback(const sensor_msgs::PointCloud2ConstPtr& message);
  void UpdateCallback(const ::ros::TimerEvent& event);
  void VisualizationCallback(const ::ros::TimerEvent& event);
  void DynamicVisualizationCallback(
      rog_map_ros1::VizConfig& config, uint32_t level);
  visualization_msgs::MarkerArray MakeBoundsMarkers(
      const rog_map::VisualizationOutput& output,
      const ::ros::Time& stamp) const;

  ::ros::NodeHandle node_;
  ::ros::NodeHandle private_node_;
  std::unique_ptr<rog_map::ROGMap> core_;
  mutable std::mutex core_mutex_;

  ::ros::Subscriber odometry_subscriber_;
  ::ros::Subscriber cloud_subscriber_;
  ::ros::Timer update_timer_;
  ::ros::Timer visualization_timer_;

  ::ros::Publisher point_cloud_publisher_;
  ::ros::Publisher occupied_publisher_;
  ::ros::Publisher unknown_publisher_;
  ::ros::Publisher inflated_occupied_publisher_;
  ::ros::Publisher inflated_unknown_publisher_;
  ::ros::Publisher frontier_publisher_;
  ::ros::Publisher esdf_publisher_;
  ::ros::Publisher negative_esdf_publisher_;
  ::ros::Publisher occupied_esdf_publisher_;
  ::ros::Publisher bounds_publisher_;
  tf2_ros::TransformBroadcaster transform_broadcaster_;

  std::unique_ptr<DynamicServer> dynamic_server_;
  DynamicServer::CallbackType dynamic_callback_;
  rog_map::VisualizationRequest visualization_request_;

  rog_map::PointCloud pending_cloud_;
  ::ros::Time pending_cloud_stamp_;
  bool has_pending_cloud_{false};

  bool ros_callback_enabled_{true};
  bool visualization_enabled_{true};
  bool use_dynamic_reconfigure_{false};
  bool publish_unknown_map_{false};
  bool invert_odometry_z_{true};
  bool broadcast_tf_{true};
  int mode_{2};
  double odometry_timeout_{2.0};
  double update_frequency_{1000.0};
  double visualization_frequency_{10.0};
  int visualization_frame_rate_{0};
  double cloud_z_scale_{1.0};
  int odometry_queue_size_{20};
  int cloud_queue_size_{2};
  int output_queue_size_{2};

  std::string cloud_topic_{"/registered_scan"};
  std::string odometry_topic_{"/state_estimation"};
  std::string map_frame_{"map"};
  std::string child_frame_{"sensor"};
  std::string visualization_frame_{"map"};
  std::string point_cloud_topic_{"rog_map/pc"};
  std::string occupied_topic_{"rog_map/occ"};
  std::string unknown_topic_{"rog_map/unk"};
  std::string inflated_occupied_topic_{"rog_map/inf_occ"};
  std::string inflated_unknown_topic_{"rog_map/inf_unk"};
  std::string frontier_topic_{"rog_map/frontier"};
  std::string esdf_topic_{"rog_map/esdf"};
  std::string negative_esdf_topic_{"rog_map/esdf_neg"};
  std::string occupied_esdf_topic_{"rog_map/esdf_occ"};
  std::string bounds_topic_{"rog_map/map_bound"};
};

}  // namespace ros1
}  // namespace perception
}  // namespace jojo
