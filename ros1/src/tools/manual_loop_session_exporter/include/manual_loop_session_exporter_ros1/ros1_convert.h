#ifndef MANUAL_LOOP_SESSION_EXPORTER_ROS1_CONVERT_H
#define MANUAL_LOOP_SESSION_EXPORTER_ROS1_CONVERT_H

#include <memory>
#include <string>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>

#include "tools/manual_loop_session_exporter/session_exporter.h"

namespace jojo {
namespace tools {
namespace manual_loop {
namespace ros1 {

class Ros1Convert {
 public:
  Ros1Convert(ros::NodeHandle& node, ros::NodeHandle& private_node);
  bool Init();

 private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      nav_msgs::Odometry, sensor_msgs::PointCloud2>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  static Eigen::Isometry3d PoseToIsometry(const geometry_msgs::Pose& pose);
  void SynchronizedCallback(const nav_msgs::OdometryConstPtr& odometry,
                            const sensor_msgs::PointCloud2ConstPtr& cloud);
  bool FinalizeService(std_srvs::Trigger::Request& request,
                       std_srvs::Trigger::Response& response);

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  message_filters::Subscriber<nav_msgs::Odometry> odometry_subscriber_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_subscriber_;
  std::unique_ptr<Synchronizer> synchronizer_;
  ros::ServiceServer finalize_service_;
  std::unique_ptr<SessionExporter> exporter_;

  std::string odometry_topic_ = "/state_estimation";
  std::string cloud_topic_ = "/registered_scan";
  std::string finalize_service_name_ = "finalize";
  int sync_queue_size_ = 100;
  double sync_slop_ = 0.05;
};

}  // namespace ros1
}  // namespace manual_loop
}  // namespace tools
}  // namespace jojo

#endif  // MANUAL_LOOP_SESSION_EXPORTER_ROS1_CONVERT_H
