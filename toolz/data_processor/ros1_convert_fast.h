#ifndef DATA_PROCESSOR_ROS1_CONVERT_H
#define DATA_PROCESSOR_ROS1_CONVERT_H

#include <ros/ros.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"

// 2023
#include "version_1.0/sensor/ESR_Radar_Info.h"
#include "version_1.0/sensor/ESR_Radar_Object.h"

// UGV_2024
#include "version_1.1/self_state/GlobalPose.h"
#include "version_1.1/self_state/LocalPose.h"
// #include "version_1.1/sensor/ESR_Radar_Info.h"
// #include "version_1.1/sensor/ESR_Radar_Object.h"

// CODEXOPEN
// #include "ars548_msg/DetectionList.h"
// #include "ars548_msg/detections.h"
#include "ars548_process/Detection.h"
#include "ars548_process/DetectionList.h"
#include "ars_40X/Cluster.h"
#include "ars_40X/ClusterList.h"
#include "livox_ros_driver2-1.2.4/CustomMsg.h"
#include "livox_ros_driver2-1.2.4/CustomPoint.h"
#include "rslidar_sdk-1.3.2/lidar_packet_ros.h"
#include "rslidar_sdk-1.3.2/lidar_scan_ros.h"

// #include <boost/foreach.hpp> // C++11 之前
#include <math.h>  // for llround
#include <stdint.h>

#include <iostream>
#include <thread>
#include <unordered_set>
#include <vector>

#include <boost/asio/post.hpp>
#include <boost/asio/thread_pool.hpp>

#include "cyber/base/thread_pool.h"
#include "modules/common/environment_conf.h"
#include "modules/common/math/math_utils_extra.h"
#include "modules/perception/common/lidar/convert/livox.h"
#include "modules/perception/common/lidar/convert/robosense.h"
#include "modules/perception/common/lidar/convert/velodyne.h"
#include "modules/perception/common/radar/convert/ars408.h"
#include "modules/perception/common/radar/convert/ars548.h"
#include "modules/perception/common/radar/convert/hugin.h"
#include "tools/data_processor/async_writer.h"
#include "tools/data_processor/block_queue.h"
#include "toolz/data_processor/data_processor.h"

namespace jojo {
namespace tools {
using namespace std;
using namespace apollo::cyber::base;
using namespace jojo::common::math;
using namespace jojo::perception::camera;

// FAST_ROUND ==> llround()
#define F_ROUND(x) ((int)((x) + ((x) >= 0 ? 0.5 : -0.5)))

// #define M_PI (3.14159265358)
#define DEG_2_RAD 0.017453293  // M_PI / 180.0
#define RAD_2_DEG 57.29578  // 180.0 / M_PI

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

/*
 * @brief 图像任务包，由 Reader 主线程填充，Worker 线程消费
 * 
 * 重要：raw_data 是已经从 bag 中 instantiate 出来的 shared_ptr，
 * 生命周期独立于 bag 迭代器，Worker 可以安全持有。
 *
 * 使用 std::variant 避免虚函数开销，同时支持压缩/非压缩两种消息类型。
 */
struct ImageTask {
  enum class ImageType { RAW, COMPRESSED };

  ImageType type;

  // 两种消息类型之一（主线程已完成 instantiate）
  sensor_msgs::ImageConstPtr raw_msg;  // type == RAW
  sensor_msgs::CompressedImageConstPtr compressed_msg;  // type == COMPRESSED

  int channel_idx;  // 对应 topic_camera_sub[i] 等的索引
  DataStatistic<cv::Mat>* ds;  // 指向对应的 DataStatistic，Worker 独占访问
  int sensor_mode;  // DataStatistic::mode (camera=1, infra=2, star=3)
};

class Ros1Convert {
 public:
  Ros1Convert(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
  virtual ~Ros1Convert();

  bool Init(std::shared_ptr<jojo::tools::RuntimeConfig> param,
            std::shared_ptr<jojo::tools::InterfaceConfig> interface);

  void CreateImageWorker();

  void Run();

  void Ros1bagParseBase(const rosbag::MessageInstance& m);

  void TryDispatchImageTask(
      const rosbag::MessageInstance& m, const std::string& topic,
      const std::unordered_map<std::string, int>& topic_map,
      std::vector<std::unique_ptr<BlockingQueue<ImageTask>>>& queues,
      std::vector<DataStatistic<cv::Mat>>& ds_vec);

  std::shared_ptr<DataProcessor> data_processor;

 protected:
  // clang-format off
  void LidarHandler(const sensor_msgs::PointCloud2::ConstPtr& msg_ptr);
  void LivoxLidarHandler(const livox_ros_driver2::CustomMsg::ConstPtr& msg_ptr);
  void NormalLidarHandler(const sensor_msgs::PointCloud2::ConstPtr& msg_ptr);

  void LocalPoseHandler(const self_state::LocalPose::ConstPtr& msg_ptr);

  void GlobalPoseHandler(const self_state::GlobalPose::ConstPtr& msg_ptr);

  void ImuDataHandler(const sensor_msgs::Imu::ConstPtr& msg_ptr);

  void RadarHandler(const rosbag::MessageInstance& m);

  void Radar4DHandler(const rosbag::MessageInstance& m, int idx);
  // clang-format on

 private:
  std::shared_ptr<jojo::tools::RuntimeConfig> rparam_;
  std::shared_ptr<jojo::tools::InterfaceConfig> iparam_;

  ros::NodeHandle nh_, pnh_;

  int num_global_pose = 0;
  int num_local_pose  = 0;
  int num_imu_data    = 0;

  AsyncWriter* gpose_writer_ = nullptr;
  AsyncWriter* lpose_writer_ = nullptr;
  AsyncWriter* imu_writer_   = nullptr;

  std::unordered_map<std::string, int> camera_topic_map;
  std::unordered_map<std::string, int> infra_topic_map;
  std::unordered_map<std::string, int> star_topic_map;
  std::unordered_map<std::string, int> radar4d_topic_map;

  std::vector<DataStatistic<cv::Mat>> ds_camera;
  std::vector<DataStatistic<cv::Mat>> ds_infra;
  std::vector<DataStatistic<cv::Mat>> ds_star;
  DataStatistic<uint> ds_radar;
  std::vector<DataStatistic<uint>> ds_radar4d;

  std::vector<std::unique_ptr<BlockingQueue<ImageTask>>> camera_queues;
  std::vector<std::unique_ptr<BlockingQueue<ImageTask>>> infra_queues;
  std::vector<std::unique_ptr<BlockingQueue<ImageTask>>> star_queues;

  std::vector<std::thread> camera_workers;
  std::vector<std::thread> infra_workers;
  std::vector<std::thread> star_workers;

  bool CheckSampledDataIsEnd();

  template <typename DataType>
  void PrintParserCount(const std::vector<DataStatistic<DataType>>& counts,
                        const std::string& name);

  // RsLidarDifopWrapper
  ros::Publisher pub_ori;
  ros::Publisher pub_difop;
  ros::Subscriber sub_cloud;

  void InitRslidar();
#if defined(RSLIDAR_OLD)
  // ros1 for intensity int8
  pcl::PointCloud<robosense_ros::PointII>::Ptr cloud_buffer_;
#elif defined(RSLIDAR_NEW)
  pcl::PointCloud<robosense_ros::PointIF>::Ptr cloud_buffer_;
#elif defined(VELODYNE)
  pcl::PointCloud<velodyne_ros::PointXYZIR>::Ptr cloud_buffer_;
#elif defined(LIVOX_NEW)
  pcl::PointCloud<livox_ros::PointXYZIRT>::Ptr cloud_buffer_;
#else
  pcl::PointCloud<robosense_ros::Point>::Ptr cloud_buffer_;
#endif
#if defined(LIDAR_DATASET)
  pcl::PointCloud<pcl::PointXYZIRT>::Ptr cloud_;
#else
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_;
#endif

  void RecvLidarHandler(const sensor_msgs::PointCloud2& msg);
  void SendLidarHandler(const rosbag::MessageInstance& m,
                        const std::string& topic);

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

#endif  // DATA_PROCESSOR_ROS1_CONVERT_H
