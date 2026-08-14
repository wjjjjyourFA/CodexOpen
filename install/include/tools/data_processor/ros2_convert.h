#ifndef DATA_PROCESSOR_ROS2_CONVERT_H
#define DATA_PROCESSOR_ROS2_CONVERT_H

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

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

// UGV_2025
#include "version_1.1/self_state/msg/global_pose.hpp"
#include "version_1.1/self_state/msg/local_pose.hpp"
#include "version_1.1/sensor/msg/esr_radar_info.hpp"
#include "version_1.1/sensor/msg/esr_radar_object.hpp"

// CODEXOPEN
#include "ars548_interface/msg/detection.hpp"
#include "ars548_interface/msg/detection_list.hpp"
#include "rslidar_msg-1.5.9/msg/rslidar_packet.hpp"

// #include <boost/foreach.hpp> // C++11 之前
#include <iostream>
#include <thread>
#include <unordered_set>
#include <vector>

#include <boost/asio/post.hpp>
#include <boost/asio/thread_pool.hpp>

#include "modules/common/environment_conf.h"
#include "modules/common/math/math_utils_extra.h"
#include "modules/perception/common/lidar/convert/robosense.h"
#include "modules/perception/common/radar/convert/ars408.h"
#include "modules/perception/common/radar/convert/ars548.h"
#include "modules/perception/common/radar/convert/hugin.h"
#include "tools/common/utils/rclcpp_utils.h"
#include "tools/data_processor/data_processor.h"

namespace jojo {
namespace tools {
using namespace std;
using namespace jojo::common::math;
using namespace jojo::perception::camera;

// FAST_ROUND ==> llround()
#define F_ROUND(x) ((int)((x) + ((x) >= 0 ? 0.5 : -0.5)))

template <typename dataType>
struct DataStatistic {
  explicit DataStatistic(std::string n = "", int m = 0) : name(n), mode(m) {}
  int num              = 0;
  size_t sampled_index = 0;
  int mode             = 0;  // for 1-camera 2-infra 3-star

  // last frame data for nearest
  int64_t diff      = INT64_MAX;
  uint64_t msg_time = 0;
  dataType data;

  std::string name = "";
  bool is_end      = false;
};

struct ImageTask {
  enum class ImageType { RAW, COMPRESSED };

  ImageType type;

  sensor_msgs::msg::Image::ConstSharedPtr raw_msg;
  sensor_msgs::msg::CompressedImage::ConstSharedPtr compressed_msg;

  int channel_idx;
  DataStatistic<cv::Mat>* ds;
  int sensor_mode;
};

template <typename MsgT>
std::shared_ptr<MsgT> DeserializeMsg(
    const std::shared_ptr<rosbag2_storage::SerializedBagMessage>& m) {
  auto msg = std::make_shared<MsgT>();
  rclcpp::Serialization<MsgT> serializer;
  rclcpp::SerializedMessage serialized_msg(*m->serialized_data);
  serializer.deserialize_message(&serialized_msg, msg.get());
  return msg;
}

class Ros2Convert {
 public:
  Ros2Convert(std::shared_ptr<rclcpp::Node> nh);  // Constructor
  virtual ~Ros2Convert();

  bool Init(std::shared_ptr<jojo::tools::RuntimeConfig> param,
            std::shared_ptr<jojo::tools::InterfaceConfig> interface);

  void Run();

  void Ros2bagParseBase(
      const std::shared_ptr<rosbag2_storage::SerializedBagMessage> m);

  void Ros2bagParseImageWrapper(
      const std::shared_ptr<rosbag2_storage::SerializedBagMessage>& m,
      const std::string& topic,
      const std::unordered_map<std::string, int>& topic_map,
      std::vector<DataStatistic<cv::Mat>>& ds_vec);

  std::shared_ptr<DataProcessor> data_processor;

 protected:
  // clang-format off
  void LidarHandler(sensor_msgs::msg::PointCloud2::ConstPtr msg_ptr);

  void LocalPoseHandler(self_state::msg::LocalPose::ConstPtr msg_ptr);

  void GlobalPoseHandler(self_state::msg::GlobalPose::ConstPtr msg_ptr);

  void ImuDataHandler(const sensor_msgs::msg::Imu::ConstPtr msg_ptr);

  void ImageWorkerFunc(const ImageTask& task, DataProcessor* proc, RuntimeConfig* param);

  void RadarHandler(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> m);

  void Radar4DHandler(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> m, int idx);
  // clang-format on

 private:
  std::shared_ptr<jojo::tools::RuntimeConfig> rparam_;
  std::shared_ptr<jojo::tools::InterfaceConfig> iparam_;

  std::shared_ptr<rclcpp::Node> node;

  int num_global_pose = 0;
  int num_local_pose  = 0;
  int num_imu_data    = 0;

  std::unordered_map<std::string, int> camera_topic_map;
  std::unordered_map<std::string, int> infra_topic_map;
  std::unordered_map<std::string, int> star_topic_map;
  std::unordered_map<std::string, int> radar4d_topic_map;

  std::vector<DataStatistic<cv::Mat>> ds_camera;
  std::vector<DataStatistic<cv::Mat>> ds_infra;
  std::vector<DataStatistic<cv::Mat>> ds_star;
  DataStatistic<uint> ds_radar;
  std::vector<DataStatistic<uint>> ds_radar4d;

  template <typename DataType>
  void PrintParserCount(const std::vector<DataStatistic<DataType>>& counts,
                        const std::string& name);

  // RsLidarDifopWrapper
  rclcpp::Publisher<rslidar_msg::msg::RslidarPacket>::SharedPtr pub_difop;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud;

  void InitRslidar();

  void RecvLidarHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg_ptr);
  void SendLidarHandler(
      const std::shared_ptr<rosbag2_storage::SerializedBagMessage> m,
      const std::string& topic);

  sem_t sem_a, sem_b;
  bool b_first_pub_difop;

  int num_lidar_send = 0;
  int num_lidar_recv = 0;
  // RsLidarDifopWrapper
};

template <typename DataType>
void Ros2Convert::PrintParserCount(
    const std::vector<DataStatistic<DataType>>& counts,
    const std::string& name) {
  size_t size = counts.size();
  if (size == 0) {
    RCLCPP_INFO(node->get_logger(), "----> message %s num %d", name.c_str(), 0);
  } else if (size == 1) {
    RCLCPP_INFO(node->get_logger(), "----> message %s num %d", name.c_str(),
                counts.at(0).num);
  } else {
    for (size_t i = 0; i < size; ++i) {
      RCLCPP_INFO(node->get_logger(), "----> message %s_%zu num %d",
                  name.c_str(), i + 1, counts.at(i).num);
    }
  }
}

}  // namespace tools
}  // namespace jojo

#endif  // DATA_PROCESSOR_ROS2_CONVERT_H
