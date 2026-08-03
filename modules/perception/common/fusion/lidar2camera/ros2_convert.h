#ifndef LIDAR_CAMERA_ROS2_CONVERT_H
#define LIDAR_CAMERA_ROS2_CONVERT_H

#include <chrono>
#include <thread>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio/videoio.hpp>
//#include <opencv2/imgcodecs/legacy/constants_c.h>

#include "rclcpp/rclcpp.hpp"
// 用image_transport软件包发布和订阅ROS中的图像
#include <image_transport/image_transport.h>
// 这两个头文件包含了CvBridge类以及与图像编码相关的函数
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>

#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>

#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

// #include "modules/common/environment_conf.h"
#include "modules/common/math/math_utils_extra.h"
// #include "modules/perception/common/camera/common/undistortion_handler.h"
#include "modules/perception/common/camera/common/undistortion_handler_cv.h"
#include "modules/perception/common/camera/params/camera_params.h"
#include "modules/perception/common/fusion/lidar2camera/config/interface_config.h"
#include "modules/perception/common/fusion/lidar2camera/config/runtime_config.h"
#include "modules/perception/common/fusion/lidar2camera/lidar_camera_fusion.h"
#include "modules/perception/common/lidar/convert/robosense.h"
#include "modules/perception/common/lidar/convert/velodyne.h"
#include "modules/perception/tools/common/show_data_2d.h"

namespace math       = jojo::common::math;
namespace perception = jojo::perception;
namespace base       = jojo::perception::base;
namespace cfg        = jojo::perception::config;
namespace camera     = jojo::perception::camera;
namespace fusion     = jojo::perception::fusion;

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> CloudT;

class Ros2Convert {
 public:
  Ros2Convert(std::shared_ptr<rclcpp::Node> nh);
  ~Ros2Convert();

  bool Init(std::shared_ptr<jojo::perception::RuntimeConfig> param,
            std::shared_ptr<jojo::perception::InterfaceConfig> interface);

  void Run();

 protected:
  void ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  void ImageCompressedCallback(
      const sensor_msgs::msg::CompressedImage::SharedPtr msg);

  void PointCloud2Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void PointCloudCallback(const sensor_msgs::msg::PointCloud::SharedPtr msg);

  std::shared_ptr<camera::CameraParams> camera_params;
  std::shared_ptr<camera::UndistortionHandler> camera_undistort;
  std::shared_ptr<fusion::LidarCameraFusion> fusion;

 private:
  std::shared_ptr<perception::RuntimeConfig> rparam_;
  std::shared_ptr<perception::InterfaceConfig> iparam_;

  std::shared_ptr<rclcpp::Node> nh_;
  rclcpp::SubscriptionBase::SharedPtr image_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
  std::string ns;

  // 因为这里是严格只读取一个图像和一个点云，所以使用了 mutex 来保护数据
  std::mutex mutex_;
  cv::Mat recvImg;
  pcl::PointCloud<robosense_ros::PointIF>::Ptr rs_cloud_ptr;
  CloudT::Ptr raw_cloud_ptr;
  bool image_recv_ = false, point_recv_ = false;
  // output
  cv::Mat dst_img;
  CloudT::Ptr dst_cloud_ptr;

  Eigen::Matrix4f trans_matrix;
};

#endif  // LIDAR_CAMERA_ROS2_CONVERT_H