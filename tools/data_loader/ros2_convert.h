#ifndef DATA_LOADER_ROS2_CONVERT_H
#define DATA_LOADER_ROS2_CONVERT_H

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pcl_conversions/pcl_conversions.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

// UGV_2024
#include "ars548_interface/msg/detection.hpp"
#include "ars548_interface/msg/detection_list.hpp"
#include "rslidar_msg-1.5.9/msg/rslidar_packet.hpp"
#include "self_state/msg/global_pose.hpp"
#include "self_state/msg/local_pose.hpp"
#include "sensor/msg/esr_radar_info.hpp"
#include "sensor/msg/esr_radar_object.hpp"

// CODEXOPEN
#include "modules/common/math/math_utils_extra.h"
#include "tools/data_loader/config/runtime_config_realtime.h"
#include "tools/data_loader/data_container_ros2.h"
#include "tools/data_loader/data_loader.h"
// #include "tools/data_loader/data_loader_realtime.h"
#include "tools/data_processor/config/sensor_config.h"

namespace jojo {
namespace tools {
using namespace std;

class Ros2Convert {
 public:
  Ros2Convert(std::shared_ptr<rclcpp::Node> nh);
  ~Ros2Convert();

  bool Init(std::shared_ptr<jojo::tools::RuntimeConfig> param,
            std::shared_ptr<jojo::tools::InterfaceConfig> interface);
  void InitRos2();

  void Run();
  bool RunRosTimer();

  bool LoadGlobalPose(const std::string& file_path);
  bool LoadGlobalPose(const std::string& path, const std::string& data_file);

  bool LoadLocalPose(const std::string& file_path);
  bool LoadLocalPose(const std::string& path, const std::string& data_file);

  std::shared_ptr<DataLoader> data_loader;
  // std::shared_ptr<DataLoaderRealtime> data_loader;

 protected:
  bool is_running_ = false;

 private:
  std::shared_ptr<jojo::tools::RuntimeConfig> rparam_;
  std::shared_ptr<jojo::tools::InterfaceConfig> iparam_;

  std::shared_ptr<rclcpp::Node> node;
  std::shared_ptr<image_transport::ImageTransport> it;
  std::string ns;

  DataContainerRos2<self_state::msg::LocalPose> dc_local_pose;
  DataContainerRos2<self_state::msg::GlobalPose> dc_global_pose;

  // 模板特化
  std::vector<DataContainerRos2<sensor_msgs::msg::Image::SharedPtr>> dc_camera;
  std::vector<DataContainerRos2<sensor_msgs::msg::Image::SharedPtr>> dc_infra;
  std::vector<DataContainerRos2<sensor_msgs::msg::Image::SharedPtr>> dc_star;

  // DataContainerRos2<sensor_msgs::msg::PointCloud> dc_radar;
  // std::vector<DataContainerRos2<sensor_msgs::msg::PointCloud2>> dc_radar4d;
  // DataContainerRos2<sensor_msgs::msg::PointCloud2> dc_lidar;
  DataContainerRos2<uint64_t /*sensor_msgs::PointCloud*/> dc_radar;
  std::vector<DataContainerRos2<uint64_t /*sensor_msgs::PointCloud2*/>>
      dc_radar4d;
  DataContainerRos2<uint64_t> dc_lidar;

  // clang-format off
  void Ros2PublishBase(DataContainerRos2Base *tmp);
  // void PublishGlobalPose();
  // void PublishLocalPose();

  bool PubLidarBase(DataContainerRos2<uint64_t> &data_c,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr cur_cloud_ptr);
  void PublishLidar();

  void PublishRadar();

  bool PubRadar4DBase(DataContainerRos2<uint64_t> &data_c, int id);
  void PublishRadar4D(int id);

  bool PubImageBase(DataContainerRos2<sensor_msgs::msg::Image::SharedPtr> &data_c, int id, int mode);
  void PublishImage(int id, int mode);
  // clang-format on
};

}  // namespace tools
}  // namespace jojo

#endif
