#include "tools/data_processor/ros1_convert_rs_legacy.h"

#include <cerrno>
#include <stdexcept>

namespace jojo {
namespace tools {

Ros1ConvertRs::Ros1ConvertRs(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
    : Ros1Convert(nh, private_nh), nh_(nh) {}

Ros1ConvertRs::~Ros1ConvertRs() {
  if (semaphore_initialized_) {
    sem_destroy(&point_cloud_ready_sem_);
  }
}

void Ros1ConvertRs::InitRslidar() {
  if (sem_init(&point_cloud_ready_sem_, 0, 0) != 0) {
    throw std::runtime_error("failed to initialize RoboSense legacy semaphore");
  }
  semaphore_initialized_ = true;

  const auto& interface = interface_config();

  // clang-format off
  point_cloud_subscriber_ = nh_.subscribe(interface->topic_lidar_sub, 10, &Ros1ConvertRs::RecvLidarHandler, this);

  // rslidar_sdk-1.3.2
  scan_publisher_ = nh_.advertise<rslidar_msgs::rslidarScan>(interface->topic_lidar_ori_sub, 10);
  difop_publisher_ = nh_.advertise<rslidar_msgs::rslidarPacket>(interface->topic_lidar_difop_sub, 10);
  // rslidar_sdk-1.5.19 
  // difop_publisher_ = nh_.advertise<rslidar_msg::RslidarPacket>(interface->topic_lidar_difop_sub, 10);
  // clang-format on

  first_difop_pending_ = true;
  ROS_INFO("----> use legacy RoboSense ROS1 driver relay for lidar type %s.",
           runtime_config()->lidar_type.c_str());
}

void Ros1ConvertRs::SendLidarHandler(const rosbag::MessageInstance& m,
                                     const std::string& topic) {
  const auto& interface = interface_config();

  // The old driver must receive a DIFOP packet before MSOP scans are sent.
  if (first_difop_pending_) {
    if (topic != interface->topic_lidar_difop_sub) return;

    auto difop = m.instantiate<rslidar_msgs::rslidarPacket>();
    if (!difop) {
      ROS_WARN("Cannot instantiate legacy rslidarPacket from topic %s",
               topic.c_str());
      return;
    }

    difop_publisher_.publish(*difop);
    first_difop_pending_ = false;
    ROS_INFO("----> published first RoboSense DIFOP packet.");
    return;
  }

  if (topic == interface->topic_lidar_ori_sub) {
    auto scan = m.instantiate<rslidar_msgs::rslidarScan>();
    if (!scan) {
      ROS_WARN("Cannot instantiate legacy rslidarScan from topic %s",
               topic.c_str());
      return;
    }

    waiting_for_point_cloud_ = true;
    scan_publisher_.publish(*scan);
    IncrementLidarSendCount();

    WaitForPointCloud();
    waiting_for_point_cloud_ = false;
    return;
  }

  if (topic == interface->topic_lidar_difop_sub) {
    auto difop = m.instantiate<rslidar_msgs::rslidarPacket>();
    if (!difop) {
      ROS_WARN("Cannot instantiate legacy rslidarPacket from topic %s",
               topic.c_str());
      return;
    }
    difop_publisher_.publish(*difop);
  }
}

void Ros1ConvertRs::RecvLidarHandler(
    const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg) {
  // Strict offline parsing: only release the bag-reading thread after this
  // point cloud has completed the common conversion and saving path.
  LidarHandler(point_cloud_msg);
  if (waiting_for_point_cloud_) {
    sem_post(&point_cloud_ready_sem_);
  }
}

void Ros1ConvertRs::WaitForPointCloud() {
  // Keep a strict one-scan/one-cloud sequence. The next bag message is not
  // consumed until the external RoboSense driver has returned and the callback
  // has completely processed the current scan's point cloud.
  while (sem_wait(&point_cloud_ready_sem_) != 0) {
    if (errno != EINTR) {
      throw std::runtime_error(
          "failed while waiting for legacy RoboSense point cloud");
    }
  }
}

void Ros1ConvertRs::FinishLidar() { waiting_for_point_cloud_ = false; }

}  // namespace tools
}  // namespace jojo
