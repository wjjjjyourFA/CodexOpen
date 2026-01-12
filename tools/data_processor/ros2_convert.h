#ifndef ROS2_CONVERT_H
#define ROS2_CONVERT_H

// humble
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/storage_options.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_filter.hpp>

// foxy
// #include <rosbag2_cpp/rosbag2_storage/serialized_bag_message.hpp>
// #include <rosbag2_cpp/rosbag2_storage/storage_filter.hpp>

#include "rclcpp/rclcpp.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

// UGV_2025
#include "rslidar_msg-1.5.9/msg/rslidar_packet.hpp"
#include "self_state/msg/global_pose.hpp"
#include "self_state/msg/local_pose.hpp"
#include "sensor/msg/esr_radar_object.hpp"
#include "sensor/msg/esr_radar_info.hpp"
#include "ars548_interface/msg/detection_list.hpp"
#include "ars548_interface/msg/detection.hpp"

// CODEXOPEN

// #include <boost/foreach.hpp> // C++11 之前
#include <iostream>
#include <thread>
#include <unordered_set>
#include <vector>

#include <boost/asio/post.hpp>
#include <boost/asio/thread_pool.hpp>

#include "cyber/common/environment_conf.h"
#include "modules/common/math/math_utils_extra.h"
#include "modules/perception/common/lidar/third_party/convert/robosense.h"
#include "modules/perception/common/radar/third_party/convert/ars408.h"
#include "modules/perception/common/radar/third_party/convert/ars548.h"
#include "modules/perception/common/radar/third_party/convert/hugin.h"

#include "tools/data_processor/data_processor.h"

namespace jojo {
namespace tools {
using namespace std;
using namespace jojo::common::math;
using namespace jojo::perception::camera;

struct DataStatistic {
  int num              = 0;
  size_t sampled_index = 0;
};

template <typename MsgT>
std::shared_ptr<MsgT> DeserializeMsg(
    const std::shared_ptr<rosbag2_storage::SerializedBagMessage> &m) {
  auto msg = std::make_shared<MsgT>();
  rclcpp::Serialization<MsgT> serializer;
  rclcpp::SerializedMessage serialized_msg(*m->serialized_data);
  serializer.deserialize_message(&serialized_msg, msg.get());
  return msg;
}

class Ros2Convert {
 public:
  Ros2Convert();  // Constructor
  virtual ~Ros2Convert();

  void Init(std::shared_ptr<rclcpp::Node> nh,
            std::shared_ptr<ParamSimple> param);

  void Run();

  void Ros2bagParseBase(
      const std::shared_ptr<rosbag2_storage::SerializedBagMessage> m);

  void Ros2bagParseRadar(
      const std::shared_ptr<rosbag2_storage::SerializedBagMessage> m);

  void Ros2bagParseImageWrapper(
      const std::shared_ptr<rosbag2_storage::SerializedBagMessage> &m,
      int sensor_count, const std::vector<std::string> &topics,
      std::vector<DataStatistic> &ds, const std::string &parser_name);

  void Ros2bagParseRadar4DWrapper(
      const std::shared_ptr<rosbag2_storage::SerializedBagMessage> &m,
      int sensor_count, const std::vector<std::string> &topics,
      std::vector<DataStatistic> &ds, const std::string &parser_name);

  std::shared_ptr<DataProcessor> data_processor;

 protected:
  // clang-format off
  void LidarHandler(sensor_msgs::msg::PointCloud2::ConstPtr msg_ptr);

  void LocalPoseHandler(self_state::msg::LocalPose::ConstPtr msg_ptr);

  void GlobalPoseHandler(self_state::msg::GlobalPose::ConstPtr msg_ptr);

  void CameraCompressedHandler(sensor_msgs::msg::CompressedImage::ConstPtr msg_ptr, int id, 
                               int mode, size_t &sampled_index, int &num_image);

  void CameraHandler(sensor_msgs::msg::Image::ConstPtr msg_ptr, int id, 
                     int mode, size_t &sampled_index, int &num_image);

  void RadarHandler(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> m);

  void Radar4DHandler(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> m, int id,
                      size_t &sampled_index, int &num_radar);
  // clang-format on

 private:
  std::shared_ptr<ParamSimple> param_ /*param_simple*/;

  std::shared_ptr<rclcpp::Node> node;

  int num_global_pose = 0;
  int num_local_pose  = 0;
  int num_imu_data    = 0;

  std::vector<DataStatistic> ds_camera;
  std::vector<DataStatistic> ds_infra;
  std::vector<DataStatistic> ds_star;
  DataStatistic ds_radar;
  std::vector<DataStatistic> ds_radar_4d;

  void PrintParserCount(const std::vector<DataStatistic> &counts,
                        const std::string &name);

  //
  std::mutex mtx;

  // RsLidarDifopWrapper
  rclcpp::Publisher<rslidar_msg::msg::RslidarPacket>::SharedPtr pub_difop;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud;

  void InitRs();

  void RecvLidarHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg_ptr);
  void SendLidarHandler(
      const std::shared_ptr<rosbag2_storage::SerializedBagMessage> m);

  sem_t sem_a, sem_b;
  bool b_first_pub_difop;

  int num_lidar_send = 0;
  int num_lidar_recv = 0;
  // RsLidarDifopWrapper
};

}  // namespace tools
}  // namespace jojo

#endif  // Ros2Convert
