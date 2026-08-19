#ifndef DATA_PROCESSOR_ROS1_CONVERT_RS_H
#define DATA_PROCESSOR_ROS1_CONVERT_RS_H

#include <cstdint>
#include <memory>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rs_driver/common/error_code.hpp>
#include <rs_driver/driver/decoder/decoder.hpp>
#include <rs_driver/msg/pcl_point_cloud_msg.hpp>

#include "tools/data_processor/ros1_convert.h"

// CODEXOPEN
#include "rslidar_sdk-1.5.19/rslidar_packet.hpp"

namespace jojo {
namespace tools {

// ROS1 converter that decodes RoboSense raw packets in-process.  All common
// bag/image/pose/radar handling remains in Ros1Convert; only the packet LiDAR
// path is replaced by rs_driver's decoder.
class Ros1ConvertRs : public Ros1Convert {
 public:
  Ros1ConvertRs(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
  ~Ros1ConvertRs() override = default;

 protected:
  void InitRslidar() override;
  void SendLidarHandler(const rosbag::MessageInstance& m,
                        const std::string& topic) override;
  void FinishLidar() override;

 private:
  using RsPointCloud = PointCloudT<pcl::PointXYZI>;
  using RsDecoder    = robosense::lidar::Decoder<RsPointCloud>;

  std::shared_ptr<RsDecoder> CreateDecoder(const std::string& lidar_type) const;
  void HandleDecoderError(const robosense::lidar::Error& error) const;
  void HandleSplitFrame(uint16_t height, double cloud_timestamp_sec);
  void SaveDecodedFrame(const std::shared_ptr<RsPointCloud>& cloud,
                        double cloud_timestamp_sec);

  std::shared_ptr<RsDecoder> decoder_;
  double last_msop_timestamp_sec_ = 0.0;
};

}  // namespace tools
}  // namespace jojo

#endif  // DATA_PROCESSOR_ROS1_CONVERT_RS_H
