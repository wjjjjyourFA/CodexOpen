#ifndef ROBOT_DOG_FAST_LIO_ROS1_CONVERT_H
#define ROBOT_DOG_FAST_LIO_ROS1_CONVERT_H

#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

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

#include "modules/localization/fast_lio/lidar_odometry.h"

namespace jojo {
namespace localization {
namespace ros1 {

class RobotDogLidarOdometry final : public fastlio::LidarOdometry {
 public:
  bool HasPose() const { return pose_inited; }
  double LidarEndTime() const { return lidar_end_time; }
  const fastlio::OdomData& Pose() const { return o_pose; }
  const fastlio::V3D& Velocity() const { return state_point.vel; }
  decltype(auto) Covariance() const { return kf.get_P(); }

  fastlio::PointCloudXYZI::Ptr RegisteredScan(bool dense) {
    const fastlio::PointCloudXYZI::Ptr& source =
        dense ? feats_undistort : feats_down_body;
    fastlio::PointCloudXYZI::Ptr world(new fastlio::PointCloudXYZI(source->size(), 1));
    for (std::size_t i = 0; i < source->size(); ++i) {
      pointBodyToWorld(&source->points[i], &world->points[i]);
    }
    return world;
  }
};

class Ros1Convert {
 public:
  Ros1Convert(ros::NodeHandle& node, ros::NodeHandle& private_node);
  ~Ros1Convert();

  bool Init();
  void Run();

 private:
  using LivoxMessage = robot_dog_livox::CustomMsg;

  void LidarCallback(const typename LivoxMessage::ConstPtr& message);
  void ImuCallback(const sensor_msgs::ImuConstPtr& message);
  bool BuildMeasureGroup(fastlio::MeasureGroup& group);
  fastlio::PointCloudXYZI::Ptr ConvertLivoxMessage(
      const typename LivoxMessage::ConstPtr& message) const;
  bool IsValidPoint(double x, double y, double z) const;

  void PublishOdometry();
  void PublishPath();
  void PublishRegisteredScan();

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::Subscriber lidar_subscriber_;
  ros::Subscriber imu_subscriber_;
  ros::Publisher odometry_publisher_;
  ros::Publisher state_estimation_publisher_;
  ros::Publisher path_publisher_;
  ros::Publisher registered_scan_publisher_;
  tf::TransformBroadcaster transform_broadcaster_;

  std::shared_ptr<RobotDogLidarOdometry> odometry_;
  std::shared_ptr<jojo::localization::RuntimeConfig> runtime_config_;
  std::shared_ptr<jojo::localization::StaticConfig> static_config_;

  mutable std::mutex buffer_mutex_;
  std::deque<fastlio::PointCloudXYZI::Ptr> lidar_buffer_;
  std::deque<double> lidar_time_buffer_;
  std::deque<fastlio::ImuData> imu_buffer_;

  fastlio::PointCloudXYZI::Ptr pending_lidar_;
  double pending_lidar_begin_time_ = 0.0;
  double pending_lidar_end_time_ = 0.0;
  double lidar_mean_scan_time_ = 0.0;
  int lidar_scan_count_ = 0;
  double last_lidar_timestamp_ = 0.0;
  double last_imu_timestamp_ = -1.0;
  bool lidar_pending_ = false;

  std::string lidar_topic_ = "/livox/lidar";
  std::string imu_topic_ = "/imu/data";
  std::string odometry_topic_ = "/lio/odom";
  std::string state_estimation_topic_ = "/state_estimation";
  std::string path_topic_ = "/lio/path";
  std::string registered_scan_topic_ = "/registered_scan";
  std::string world_frame_ = "map";
  std::string sensor_frame_ = "sensor";
  std::string output_root_ = "/tmp/codexopen_robot_dog_fast_lio";

  bool time_sync_enabled_ = false;
  double lidar_to_imu_time_offset_ = 0.0;
  int point_filter_num_ = 3;
  int scan_line_count_ = 4;
  double blind_distance_ = 0.5;
  double inner_x_min_ = -0.7;
  double inner_x_max_ = 0.7;
  double inner_y_min_ = -0.4;
  double inner_y_max_ = 0.4;
  double inner_z_min_ = -0.6;
  double inner_z_max_ = 0.5;
  double max_range_ = 15.0;

  bool path_enabled_ = true;
  bool scan_publish_enabled_ = true;
  bool dense_publish_enabled_ = true;
  double processing_rate_ = 20.0;
  Eigen::Vector3d last_path_position_ = Eigen::Vector3d(0.0, 0.0, -100.0);
  nav_msgs::Path path_message_;
};

}  // namespace ros1
}  // namespace localization
}  // namespace jojo

#endif  // ROBOT_DOG_FAST_LIO_ROS1_CONVERT_H
