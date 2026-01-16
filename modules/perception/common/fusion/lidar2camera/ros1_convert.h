#ifndef ROS1_CONVERT_H
#define ROS1_CONVERT_H

#include <chrono>
#include <thread>

#include <opencv2/opencv.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/imgcodecs/legacy/constants_c.h>

#include <ros/ros.h>
// 用image_transport软件包发布和订阅ROS中的图像
#include <image_transport/image_transport.h>
// 这两个头文件包含了CvBridge类以及与图像编码相关的函数
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/CompressedImage.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>

#include "modules/common/environment_conf.h"
#include "modules/common/math/math_utils_extra.h"
// #include "modules/perception/common/camera/common/undistortion_handler_legacy.h"
#include "modules/perception/common/camera/common/undistortion_handler_cv.h"
#include "modules/perception/common/camera/params/camera_params.h"
#include "modules/perception/common/lidar/convert/robosense.h"
#include "modules/perception/common/lidar/convert/velodyne.h"
#include "modules/perception/common/fusion/lidar2camera/lidar_camera_fusion.h"
#include "modules/perception/tools/common/show_data_2d.h"
#include "modules/perception/common/fusion/lidar2camera/config/runtime_config_realtime.h"

namespace math = jojo::common::math;
namespace perception = jojo::perception;
namespace base = jojo::perception::base;
namespace cfg = jojo::perception::config;
namespace camera = jojo::perception::camera;
namespace fusion = jojo::perception::fusion;

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> CloudT;

class Ros1Convert {
 public:
  Ros1Convert();
  ~Ros1Convert();

  bool Init(ros::NodeHandle& nh, ros::NodeHandle& private_nh,
            std::shared_ptr<perception::RuntimeConfig> param);
  void Run();

 protected:
  void ImageCallback(const sensor_msgs::ImageConstPtr& msg);

  void ImageCompressedCallback(const sensor_msgs::CompressedImageConstPtr& msg);

  void PointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& msg);

  void PointCloudCallback(const sensor_msgs::PointCloudConstPtr& msg);

  std::shared_ptr<camera::CameraParams> camera_params;
  std::shared_ptr<camera::UndistortionHandler> camera_undistort;
  // std::shared_ptr<camera::UndistortionHandlerCv> camera_undistort;
  std::shared_ptr<fusion::LidarCameraFusion> fusion;

 private:
  std::shared_ptr<perception::RuntimeConfig> param_ /*param_manager_*/;

  ros::NodeHandle node;
  ros::Subscriber image_sub, cloud_sub;
  std::string ns;

  // 因为这里是严格只读取一个图像和一个点云，所以使用了 mutex 来保护数据
  std::mutex mutex_;
  cv::Mat recvImg;
#if defined(RSLIDAR_OLD)
  pcl::PointCloud<robosense_ros::PointII>::Ptr rs_cloud_ptr;
#elif defined(RSLIDAR_NEW)
  pcl::PointCloud<robosense_ros::PointIF>::Ptr rs_cloud_ptr;
#elif defined(VELODYNE)
  pcl::PointCloud<velodyne_ros::PointXYZIR>::Ptr vd_cloud_ptr;
#else 
  pcl::PointCloud<robosense_ros::Point>::Ptr rs_cloud_ptr;
#endif
  CloudT::Ptr raw_cloud_ptr;
  bool image_recv_ = false, point_recv_ = false;
  // output
  cv::Mat dst_img;
  CloudT::Ptr dst_cloud_ptr;

  Eigen::Matrix4f trans_matrix;
};

#endif  // Ros1Convert
