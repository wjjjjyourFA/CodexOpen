#ifndef ROS1_CONVERT_H
#define ROS1_CONVERT_H

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

#include "modules/common/environment_conf.h"
#include "modules/common/math/math_utils_extra.h"
#include "tools/data_processor/config/sensor_config.h"

#include "tools/data_loader/config/runtime_config_realtime.h"
#include "tools/data_loader/data_loader_realtime.h"
#include "tools/data_loader/data_container_ros1.h"

namespace jojo {
namespace tools {
using namespace std;

class Ros1Convert {
 public:
  Ros1Convert();
  ~Ros1Convert();

  void Init(ros::NodeHandle &nh, ros::NodeHandle &private_nh,
            std::shared_ptr<RuntimeConfigRealtime> param);
  void InitRos1();

  void Run();
  bool RunRosTimer();

  bool LoadGlobalPose(const std::string &path, const std::string &data_file);

  bool LoadLocalPose(const std::string &path, const std::string &data_file);

  std::shared_ptr<DataLoaderRealtime> data_loader;

 protected:
  bool is_running_ = false;

 private:
  std::shared_ptr<RuntimeConfigRealtime> param_;

  ros::NodeHandle node;
  std::shared_ptr<image_transport::ImageTransport> it;
  std::string ns;

  DataContainerRos1<self_state::LocalPose> dc_local_pose;
  DataContainerRos1<self_state::GlobalPose> dc_global_pose;

  // 模板特化
  std::vector<DataContainerRos1<sensor_msgs::ImagePtr>> dc_camera;
  std::vector<DataContainerRos1<sensor_msgs::ImagePtr>> dc_infra;
  std::vector<DataContainerRos1<sensor_msgs::ImagePtr>> dc_star;

  DataContainerRos1<uint64_t /*sensor_msgs::PointCloud*/> dc_radar;
  std::vector<DataContainerRos1<uint64_t /*sensor_msgs::PointCloud2*/>> dc_radar_4d;
  DataContainerRos1<uint64_t> dc_lidar;

  // clang-format off
  void Ros1PublishBase(const ros::TimerEvent &, DataContainerBase *tmp);
  // void PublishGlobalPose(const ros::TimerEvent &);
  // void PublishLocalPose(const ros::TimerEvent &);

  bool PubLidarBase(DataContainerRos1<uint64_t> &data_c,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr cur_cloud_ptr);
  void PublishLidar(const ros::TimerEvent &);

  void PublishRadar(const ros::TimerEvent &);

  bool PubRadar4DBase(DataContainerRos1<uint64_t> &data_c, int id);
  void PublishRadar4D(const ros::TimerEvent &, int id);

  bool PubImageBase(DataContainerRos1<sensor_msgs::ImagePtr> &data_c, int id, int mode);
  void PublishImage(const ros::TimerEvent &, int id, int mode);
  // clang-format on
};

}  // namespace tools
}  // namespace jojo

#endif
