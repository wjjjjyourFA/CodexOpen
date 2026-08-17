#pragma once

#include <memory>
#include <string>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <self_state/GlobalPose.h>
#include <self_state/LidarGlobalPose.h>
#include <tf/transform_broadcaster.h>

#include "modules/localization/prior_map_localization/prior_map_localization.h"

namespace jojo {
namespace localization {
namespace ros1 {

class PriorMapLocalizationRos1Convert {
 public:
  PriorMapLocalizationRos1Convert(::ros::NodeHandle& node,
                                  ::ros::NodeHandle& private_node,
                                  std::string runtime_config,
                                  std::string map_path,
                                  std::string log_directory);

  bool Init();
  void Run();

 private:
  void StandardCloudCallback(
      const sensor_msgs::PointCloud2ConstPtr& message);
  void LivoxCloudCallback(
      const livox_ros_driver2::CustomMsgConstPtr& message);
  void StandardImuCallback(const sensor_msgs::ImuConstPtr& message);
  void LivoxImuCallback(const sensor_msgs::ImuConstPtr& message);
  void InitialPoseCallback(
      const geometry_msgs::PoseWithCovarianceStampedConstPtr& message);
  void GlobalPoseCallback(const self_state::GlobalPoseConstPtr& message);

  ImuDataConstPtr ConvertStandardImu(
      const sensor_msgs::Imu& message) const;
  static ImuDataConstPtr ConvertLivoxImu(const sensor_msgs::Imu& message);
  void PublishMap();
  void PublishOutput(const PriorMapLocalizationOutput& output);
  nav_msgs::Odometry MakeOdometry(
      const PriorMapLocalizationOutput& output,
      const std::string& frame_id,
      const std::string& child_frame_id) const;

  ::ros::NodeHandle node_;
  ::ros::NodeHandle private_node_;
  std::string runtime_config_;
  std::string map_path_;
  std::string log_directory_;
  std::unique_ptr<PriorMapLocalization> core_;

  ::ros::Subscriber lidar_subscriber_;
  ::ros::Subscriber imu_subscriber_;
  ::ros::Subscriber initial_pose_subscriber_;
  ::ros::Subscriber global_pose_subscriber_;
  ::ros::Publisher map_publisher_;
  ::ros::Publisher path_publisher_;
  ::ros::Publisher lio_odometry_publisher_;
  ::ros::Publisher lio_path_publisher_;
  ::ros::Publisher state_estimation_publisher_;
  ::ros::Publisher registered_scan_publisher_;
  ::ros::Publisher lidar_global_pose_publisher_;
  tf::TransformBroadcaster transform_broadcaster_;

  nav_msgs::Path path_;
  nav_msgs::Path lio_path_;
  Eigen::Vector3d last_lio_path_position_{Eigen::Vector3d::Zero()};
  bool has_last_lio_path_position_{false};

  std::string lidar_livox_topic_{"/livox/lidar"};
  std::string imu_livox_topic_{"/imu/data"};
  std::string lidar_standard_topic_{"/rslidar_points/main"};
  std::string imu_standard_topic_{"SensorMsgsIMU"};
  std::string global_pose_topic_{"/self_state/GlobalPose_ugv"};
  std::string initial_pose_topic_{"/initialpose"};
  std::string map_topic_{"/world_state/cloud_map"};
  std::string path_topic_{"/world_state/path"};
  std::string lio_odometry_topic_{"/lio/odom"};
  std::string lio_path_topic_{"/lio/path"};
  std::string state_estimation_topic_{"/state_estimation"};
  std::string registered_scan_topic_{"/registered_scan"};
  std::string lidar_global_pose_topic_{"/self_state/LidarGlobalPose"};
  std::string map_frame_{"map"};
  std::string world_frame_{"world"};
  std::string sensor_frame_{"sensor"};

  int lidar_queue_size_{5};
  int imu_queue_size_{200000};
  int initial_pose_queue_size_{1};
  int global_pose_queue_size_{20};
  int map_queue_size_{10};
  int path_queue_size_{5};
  int odometry_queue_size_{100};
  int lio_path_queue_size_{1};
  int registered_scan_queue_size_{10};
  bool map_latched_{true};
  double processing_frequency_{5000.0};
  double path_min_distance_{0.1};
  double standard_lidar_time_offset_{-0.1};
  double standard_imu_gyro_scale_{10.0};
  bool standard_imu_swap_xy_{true};
  bool standard_imu_invert_y_{true};
};

}  // namespace ros1
}  // namespace localization
}  // namespace jojo
