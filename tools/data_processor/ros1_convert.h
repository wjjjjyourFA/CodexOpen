#ifndef ROS1_CONVERT_H
#define ROS1_CONVERT_H

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>

#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"

// UGV_2024
#include "version_1.1/self_state/GlobalPose.h"
#include "version_1.1/self_state/LocalPose.h"
#include "version_1.1/sensor/ESR_Radar_Info.h"
#include "version_1.1/sensor/ESR_Radar_Object.h"

// CODEXOPEN
#include "rslidar_sdk-1.3.2/lidar_packet_ros.h"
#include "rslidar_sdk-1.3.2/lidar_scan_ros.h"
#include "ars548_msg/DetectionList.h"
#include "ars548_msg/detections.h"
#include "ars_40X/Cluster.h"
#include "ars_40X/ClusterList.h"

// #include <boost/foreach.hpp> // C++11 之前
#include <iostream>
#include <thread>
#include <unordered_set>
#include <vector>
#include <stdint.h>
#include <math.h>  // for llround

#include <boost/asio/post.hpp>
#include <boost/asio/thread_pool.hpp>

#include "modules/common/environment_conf.h"
#include "modules/common/math/math_utils_extra.h"
#include "modules/perception/common/lidar/convert/robosense.h"
#include "modules/perception/common/lidar/convert/velodyne.h"
#include "modules/perception/common/radar/convert/ars408.h"
#include "modules/perception/common/radar/convert/ars548.h"
#include "modules/perception/common/radar/convert/hugin.h"

#include "tools/data_processor/data_processor.h"

namespace jojo {
namespace tools {
using namespace std;
using namespace jojo::common::math;
using namespace jojo::perception::camera;

template <typename dataType>
struct DataStatistic {
  explicit DataStatistic(int m = 0) : mode(m) {}
  int num              = 0;
  size_t sampled_index = 0;
  int mode             = 0;  // for 1-camera 2-infra 3-star

  // last frame data for nearest
  int64_t diff      = INT64_MAX;
  uint64_t msg_time = 0;
  dataType data;
};

class Ros1Convert {
 public:
  Ros1Convert();  // Constructor
  virtual ~Ros1Convert();

  void Init(ros::NodeHandle& nh, ros::NodeHandle& private_nh,
            std::shared_ptr<RuntimeConfig> param);

  void Run();

  void Ros1bagParseBase(const rosbag::MessageInstance& m);

  void Ros1bagParseImageWrapper(const rosbag::Bag& bag, int sensor_count,
                                const std::vector<std::string>& topics,
                                std::vector<DataStatistic<cv::Mat>>& ds,
                                const std::string& parser_name);

  std::shared_ptr<DataProcessor> data_processor;

 protected:
  // clang-format off
  void LidarHandler(const sensor_msgs::PointCloud2::ConstPtr msg_ptr);

  void LocalPoseHandler(self_state::LocalPose::ConstPtr msg_ptr);

  void GlobalPoseHandler(self_state::GlobalPose::ConstPtr msg_ptr);

  void ImuDataHandler(sensor_msgs::Imu::ConstPtr msg_ptr);

  void CameraCompressedHandler(sensor_msgs::CompressedImage::ConstPtr msg_ptr, int& id, DataStatistic<cv::Mat>& ds);

  void CameraHandler(sensor_msgs::Image::ConstPtr msg, int& id, DataStatistic<cv::Mat>& ds);

  void RadarHandler(const rosbag::MessageInstance& m);

  void Radar4DHandler(const rosbag::MessageInstance& m, int id);
  // clang-format on

 private:
  std::shared_ptr<RuntimeConfig> param_ /*param_simple*/;

  ros::NodeHandle nh_, private_nh_;

  int num_global_pose = 0;
  int num_local_pose  = 0;
  int num_imu_data    = 0;

  std::vector<DataStatistic<cv::Mat>> ds_camera;
  std::vector<DataStatistic<cv::Mat>> ds_infra;
  std::vector<DataStatistic<cv::Mat>> ds_star;
  DataStatistic<uint> ds_radar;
  std::vector<DataStatistic<uint>> ds_radar_4d;

  template <typename DataType>
  void PrintParserCount(const std::vector<DataStatistic<DataType>>& counts,
                        const std::string& name);

  //
  std::mutex mtx;

  // RsLidarDifopWrapper
  ros::Publisher pub_ori;
  ros::Publisher pub_difop;
  ros::Subscriber sub_cloud;

  void InitRs();

  void RecvLidarHandler(const sensor_msgs::PointCloud2& msg);
  void SendLidarHandler(const rosbag::MessageInstance& m, std::string& topic);

  sem_t sem_a, sem_b;
  bool b_first_pub_difop;

  int num_lidar_send = 0;
  int num_lidar_recv = 0;
  // RsLidarDifopWrapper
};

// 实现函数模板（通常放在头文件里）
template <typename DataType>
void Ros1Convert::PrintParserCount(
    const std::vector<DataStatistic<DataType>>& counts,
    const std::string& name) {
  size_t size = counts.size();
  if (size == 0) {
    ROS_INFO("----> message %s num %d", name.c_str(), 0);
  } else if (size == 1) {
    ROS_INFO("----> message %s num %d", name.c_str(), counts[0].num);
  } else {
    for (size_t i = 0; i < size; ++i) {
      ROS_INFO("----> message %s_%zu num %d", name.c_str(), i + 1,
               counts[i].num);
    }
  }
}

}  // namespace tools
}  // namespace jojo

#endif  // Ros1Convert
