#include "tools/data_processor/ros1_convert_rs.h"

#include <cmath>
#include <functional>
#include <stdexcept>

#include <pcl/common/transforms.h>
#include <rs_driver/driver/decoder/decoder_factory.hpp>

namespace jojo {
namespace tools {

Ros1ConvertRs::Ros1ConvertRs(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
    : Ros1Convert(nh, private_nh) {}

std::shared_ptr<Ros1ConvertRs::RsDecoder> Ros1ConvertRs::CreateDecoder(
    const std::string& lidar_type) const {
  robosense::lidar::LidarType rs_type;

  switch (SensorRegistry::Instance().GetLidarType(lidar_type)) {
    case ::LidarType::RS128:
      rs_type = robosense::lidar::LidarType::RS128;
      break;
    case ::LidarType::M1P:
      rs_type = robosense::lidar::LidarType::RSM1;
      break;
    case ::LidarType::RSEMX:
      rs_type = robosense::lidar::LidarType::RSEMX;
      break;
    case ::LidarType::RSE1:
      rs_type = robosense::lidar::LidarType::RSE1;
      break;
    default:
      throw std::invalid_argument(
          "RoboSense decoder does not support lidar_type='" + lidar_type + "'");
  }

  robosense::lidar::RSDecoderParam decoder_param;
  // A rosbag may be replayed much faster/slower than real time. 
  // Use packet timestamps so saved point-cloud timestamps do not depend on replay speed.
  decoder_param.use_lidar_clock = true;
  decoder_param.dense_points    = true;

  return robosense::lidar::DecoderFactory<RsPointCloud>::createDecoder(
      rs_type, decoder_param);
}

void Ros1ConvertRs::InitRslidar() {
  decoder_ = CreateDecoder(runtime_config()->lidar_type);
  decoder_->point_cloud_.reset(new RsPointCloud);
  decoder_->regCallback(
      std::bind(&Ros1ConvertRs::HandleDecoderError, this,
                std::placeholders::_1),
      std::bind(&Ros1ConvertRs::HandleSplitFrame, this, std::placeholders::_1,
                std::placeholders::_2));

  ROS_INFO("----> use rs_driver decoder for lidar type %s.",
           runtime_config()->lidar_type.c_str());
}

void Ros1ConvertRs::SendLidarHandler(const rosbag::MessageInstance& m,
                                     const std::string& topic) {
  if (!decoder_) {
    throw std::runtime_error("RoboSense decoder is not initialized");
  }

  auto packet = m.instantiate<rslidar_msg::RslidarPacket>();
  if (!packet) {
    ROS_WARN("Cannot instantiate rslidar_msg/RslidarPacket from topic %s",
             topic.c_str());
    return;
  }
  if (packet->data.empty()) {
    ROS_WARN("Ignore empty RoboSense packet from topic %s", topic.c_str());
    return;
  }

  IncrementLidarSendCount();

  const bool is_difop = packet->is_difop != 0;
  if (is_difop) {
    decoder_->processDifopPkt(packet->data.data(), packet->data.size());
    return;
  }

  last_msop_timestamp_sec_ = packet->header.stamp.toSec();
  decoder_->processMsopPkt(packet->data.data(), packet->data.size());
}

void Ros1ConvertRs::HandleDecoderError(
    const robosense::lidar::Error& error) const {
  ROS_WARN("rs_driver decoder: %s", error.toString().c_str());
}

void Ros1ConvertRs::HandleSplitFrame(uint16_t height,
                                     double cloud_timestamp_sec) {
  if (!decoder_ || !decoder_->point_cloud_) return;

  std::shared_ptr<RsPointCloud> completed_cloud = decoder_->point_cloud_;
  decoder_->point_cloud_.reset(new RsPointCloud);

  if (completed_cloud->empty()) return;
  (void)height;
  completed_cloud->height   = 1;
  completed_cloud->width    = static_cast<uint32_t>(completed_cloud->size());
  completed_cloud->is_dense = true;

  SaveDecodedFrame(completed_cloud, cloud_timestamp_sec);
}

void Ros1ConvertRs::SaveDecodedFrame(const std::shared_ptr<RsPointCloud>& cloud,
                                     double cloud_timestamp_sec) {
  if (!cloud || cloud->empty()) return;

  if (!std::isfinite(cloud_timestamp_sec) || cloud_timestamp_sec <= 0.0) {
    cloud_timestamp_sec = last_msop_timestamp_sec_;
  }
  if (!std::isfinite(cloud_timestamp_sec) || cloud_timestamp_sec <= 0.0) {
    ROS_WARN("Drop RoboSense cloud because no valid timestamp is available");
    return;
  }

  const uint64_t msg_time =
      static_cast<uint64_t>(std::llround(cloud_timestamp_sec * 1000.0));

  if (!data_processor->PushSampledTime(msg_time)) return;

  RsPointCloud::Ptr output_cloud(new RsPointCloud);
  output_cloud->swap(*cloud);

  pcl::transformPointCloud(*output_cloud, *output_cloud,
                           GetTransMatrix(runtime_config()->b_lt_none_rt));

  if (runtime_config()->b_save_data) {
    data_processor->SaveLidarData(output_cloud, msg_time);
  }

  IncrementLidarRecvCount();
}

void Ros1ConvertRs::FinishLidar() {
  if (!decoder_ || !decoder_->point_cloud_ || decoder_->point_cloud_->empty()) {
    return;
  }

  std::shared_ptr<RsPointCloud> final_cloud = decoder_->point_cloud_;
  decoder_->point_cloud_.reset(new RsPointCloud);
  final_cloud->height   = 1;
  final_cloud->width    = static_cast<uint32_t>(final_cloud->size());
  final_cloud->is_dense = true;

  double timestamp_sec = decoder_->prevPktTs();
  if (!std::isfinite(timestamp_sec) || timestamp_sec <= 0.0) {
    timestamp_sec = last_msop_timestamp_sec_;
  }
  SaveDecodedFrame(final_cloud, timestamp_sec);
}

}  // namespace tools
}  // namespace jojo
