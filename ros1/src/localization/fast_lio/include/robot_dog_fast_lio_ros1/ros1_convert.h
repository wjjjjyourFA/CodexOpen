#ifndef ROBOT_DOG_FAST_LIO_ROS1_CONVERT_H
#define ROBOT_DOG_FAST_LIO_ROS1_CONVERT_H

#include <memory>
#include <string>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>

#if __has_include(<livox_ros_driver2/CustomMsg.h>)
#include <livox_ros_driver2/CustomMsg.h>
namespace robot_dog_livox = livox_ros_driver2;
#elif __has_include(<livox_ros_driver2-1.2.4/CustomMsg.h>)
#include <livox_ros_driver2-1.2.4/CustomMsg.h>
namespace robot_dog_livox = livox_ros_driver2;
#elif __has_include(<livox_ros_driver/CustomMsg.h>)
#include <livox_ros_driver/CustomMsg.h>
namespace robot_dog_livox = livox_ros_driver;
#else
#error "A Livox CustomMsg header is required"
#endif

#include "modules/localization/fast_lio/fast_lio_pipeline.h"

namespace jojo {
namespace localization {
namespace ros1 {

class Ros1Convert {
 public:
  Ros1Convert(::ros::NodeHandle& node, ::ros::NodeHandle& private_node);
  ~Ros1Convert() = default;

  bool Init();
  void Run();

 private:
  using LivoxMessage = robot_dog_livox::CustomMsg;

  void LidarCallback(const typename LivoxMessage::ConstPtr& message);
  void ImuCallback(const sensor_msgs::ImuConstPtr& message);
  void PublishOutput(const FastLioOutput& output);
  nav_msgs::Odometry MakeOdometry(
      const FastLioOutput& output, const std::string& frame_id,
      const std::string& child_frame_id) const;

  ::ros::NodeHandle node_;
  ::ros::NodeHandle private_node_;
  ::ros::Subscriber lidar_subscriber_;
  ::ros::Subscriber imu_subscriber_;
  ::ros::Publisher odometry_publisher_;
  ::ros::Publisher state_estimation_publisher_;
  ::ros::Publisher path_publisher_;
  ::ros::Publisher registered_scan_publisher_;
  tf::TransformBroadcaster transform_broadcaster_;
  std::unique_ptr<FastLioPipeline> core_;

  std::string lidar_topic_{"/livox/lidar"};
  std::string imu_topic_{"/imu/data"};
  std::string odometry_topic_{"/lio/odom"};
  std::string state_estimation_topic_{"/state_estimation"};
  std::string path_topic_{"/lio/path"};
  std::string registered_scan_topic_{"/registered_scan"};
  std::string world_frame_{"map"};
  std::string odometry_frame_{"world"};
  std::string sensor_frame_{"sensor"};
  std::string output_root_{"/tmp/codexopen_robot_dog_fast_lio"};
  int lidar_queue_size_{1};
  int imu_queue_size_{200000};
  int output_queue_size_{100};
  bool path_enabled_{true};
  bool scan_publish_enabled_{true};
  bool dense_publish_enabled_{true};
  double processing_rate_{20.0};
  double path_min_distance_{0.1};
  Eigen::Vector3d last_path_position_{0.0, 0.0, -100.0};
  nav_msgs::Path path_message_;
};

}  // namespace ros1
}  // namespace localization
}  // namespace jojo

#endif  // ROBOT_DOG_FAST_LIO_ROS1_CONVERT_H
