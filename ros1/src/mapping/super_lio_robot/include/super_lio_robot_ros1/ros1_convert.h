#ifndef SUPER_LIO_ROBOT_ROS1_CONVERT_H_
#define SUPER_LIO_ROBOT_ROS1_CONVERT_H_

#include <cstdint>
#include <deque>
#include <memory>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>

#if __has_include(<livox_ros_driver2/CustomMsg.h>)
#include <livox_ros_driver2/CustomMsg.h>
namespace super_lio_robot_livox = livox_ros_driver2;
#elif __has_include(<livox_ros_driver2-1.2.4/CustomMsg.h>)
#include <livox_ros_driver2-1.2.4/CustomMsg.h>
namespace super_lio_robot_livox = livox_ros_driver2;
#elif __has_include(<livox_ros_driver/CustomMsg.h>)
#include <livox_ros_driver/CustomMsg.h>
namespace super_lio_robot_livox = livox_ros_driver;
#else
#error "A Livox CustomMsg header is required"
#endif

#include "lio/ESKF.h"
#include "lio/data_interface.h"

namespace rsm1_ros {
struct EIGEN_ALIGN16 Point {
  PCL_ADD_POINT4D;
  std::uint8_t intensity;
  std::uint16_t ring;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace rsm1_ros

POINT_CLOUD_REGISTER_POINT_STRUCT(
    rsm1_ros::Point,
    (float, x, x)(float, y, y)(float, z, z)
    (std::uint8_t, intensity, intensity)(std::uint16_t, ring, ring)
    (double, timestamp, timestamp))

namespace LI2Sup {
namespace ros1 {

class Ros1Convert final : public DataInterface {
 public:
  using Ptr = std::shared_ptr<Ros1Convert>;
  using LivoxMessage = super_lio_robot_livox::CustomMsg;

  Ros1Convert(::ros::NodeHandle& node, ::ros::NodeHandle& private_node);
  ~Ros1Convert() override = default;

  bool Init(const std::string& map_root);
  void SpinOnce();
  double processing_rate() const { return processing_rate_; }

  bool sync_measure(MeasureGroup& measurements) override;
  void setESKF(std::shared_ptr<ESKF>& eskf) override;
  void pub_odom(const NavState& state) override;
  void pub_cloud_world(const BASIC::CloudPtr& cloud,
                       double timestamp) override;
  void pub_registered_scan(const BASIC::CloudPtr& cloud,
                           double timestamp) override;
  void set_global_map(const BASIC::CloudPtr& global_map) override;
  void set_initial_data(BASIC::SE3& initial_pose,
                        bool& has_initial_guess,
                        bool initialization_finished = false) override;

 private:
  bool LoadAlgorithmParameters(const std::string& map_root);
  bool LoadInterfaceParameters();
  bool ValidateParameters() const;
  void ConfigureRosIo();

  void ImuCallback(const sensor_msgs::ImuConstPtr& message);
  void LivoxCallback(const typename LivoxMessage::ConstPtr& message);
  void StandardCloudCallback(const sensor_msgs::PointCloud2ConstPtr& message);
  void InitialPoseCallback(
      const geometry_msgs::PoseWithCovarianceStampedConstPtr& message);
  void GlobalMapTimerCallback(const ::ros::TimerEvent& event);

  ::ros::NodeHandle node_;
  ::ros::NodeHandle private_node_;
  ::ros::CallbackQueue callback_queue_;
  ::ros::Subscriber lidar_subscriber_;
  ::ros::Subscriber imu_subscriber_;
  ::ros::Subscriber initial_pose_subscriber_;
  ::ros::Publisher odometry_publisher_;
  ::ros::Publisher state_estimation_publisher_;
  ::ros::Publisher path_publisher_;
  ::ros::Publisher registered_scan_publisher_;
  ::ros::Publisher world_cloud_publisher_;
  ::ros::Publisher imu_odometry_publisher_;
  ::ros::Publisher robot_odometry_publisher_;
  ::ros::Publisher robot_pose_publisher_;
  ::ros::Publisher global_map_publisher_;
  ::ros::Timer global_map_timer_;
  tf::TransformBroadcaster transform_broadcaster_;

  std::deque<IMUData> imu_buffer_;
  std::deque<LidarData> lidar_buffer_;
  bool lidar_pushed_{false};
  double last_imu_timestamp_{-1.0};
  double last_lidar_timestamp_{-1.0};
  ESKF::Ptr eskf_;

  nav_msgs::Path path_;
  BASIC::V3 last_path_point_{0.0, 0.0, -100.0};
  sensor_msgs::PointCloud2 global_map_message_;
  BASIC::SE3* initial_pose_{nullptr};
  bool* has_initial_guess_{nullptr};
  int global_map_timer_count_{0};
  int global_map_publish_interval_{1};

  std::string lidar_topic_{"/livox/lidar"};
  std::string imu_topic_{"/imu/data"};
  std::string odometry_topic_{"/lio/odom"};
  std::string state_estimation_topic_{"/state_estimation"};
  std::string path_topic_{"/lio/path"};
  std::string registered_scan_topic_{"/registered_scan"};
  std::string world_cloud_topic_{"/lio/cloud_world"};
  std::string imu_odometry_topic_{"/lio/imu/odom"};
  std::string robot_odometry_topic_{"/lio/robo/odom"};
  std::string robot_pose_topic_{"/mavros/vision_pose/pose"};
  std::string global_map_topic_{"/lio/global_map"};
  std::string initial_pose_topic_{"/initialpose"};
  std::string world_frame_{"world"};
  std::string map_frame_{"map"};
  std::string sensor_frame_{"sensor"};
  int lidar_queue_size_{1000};
  int imu_queue_size_{10000};
  int odometry_queue_size_{100};
  int path_queue_size_{1};
  int cloud_queue_size_{10};
  int initial_pose_queue_size_{1};
  double processing_rate_{500.0};
  double path_min_distance_{0.1};
  bool global_map_latched_{true};
};

}  // namespace ros1
}  // namespace LI2Sup

#endif  // SUPER_LIO_ROBOT_ROS1_CONVERT_H_
