#ifndef DATA_PROCESSOR_ROS1_CONVERT_RS_LEGACY_H
#define DATA_PROCESSOR_ROS1_CONVERT_RS_LEGACY_H

#include <semaphore.h>

#include <atomic>
#include <string>

#include "tools/data_processor/ros1_convert.h"

// Message definitions used by the RoboSense ROS1 driver 1.3.x.
#include "rslidar_sdk-1.3.2/lidar_packet_ros.h"
#include "rslidar_sdk-1.3.2/lidar_scan_ros.h"
// #include "rslidar_sdk-1.5.19/rslidar_packet.hpp"

namespace jojo {
namespace tools {

// Compatibility path for the legacy RoboSense ROS1 driver.
//
// Raw scan/difop messages are republished to the external driver and the
// resulting PointCloud2 message is fed back into Ros1Convert's common lidar
// processing path. This keeps image/pose/radar and point-cloud saving behavior
// in the base class while isolating the legacy ROS relay state in this subclass.
class Ros1ConvertRs : public Ros1Convert {
 public:
  Ros1ConvertRs(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
  ~Ros1ConvertRs() override;

 protected:
  void InitRslidar() override;
  void SendLidarHandler(const rosbag::MessageInstance& m,
                        const std::string& topic) override;
  void FinishLidar() override;

 private:
  void RecvLidarHandler(
      const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg);
  void WaitForPointCloud();

  ros::NodeHandle nh_;
  ros::Publisher scan_publisher_;
  ros::Publisher difop_publisher_;
  ros::Subscriber point_cloud_subscriber_;

  sem_t point_cloud_ready_sem_;

  bool semaphore_initialized_ = false;
  std::atomic<bool> waiting_for_point_cloud_{false};
  bool first_difop_pending_ = true;
};

}  // namespace tools
}  // namespace jojo

#endif  // DATA_PROCESSOR_ROS1_CONVERT_RS_LEGACY_H
