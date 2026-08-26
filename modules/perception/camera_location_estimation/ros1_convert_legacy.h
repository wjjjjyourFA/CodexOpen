#ifndef CAMERA_LOCATION_ESTIMATION_ROS1_CONVERT_H
#define CAMERA_LOCATION_ESTIMATION_ROS1_CONVERT_H

#include <atomic>
#include <memory>
#include <mutex>

#include <ros/ros.h>

#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include "modules/common/math/math_utils_extra.h"
#include "modules/perception/camera_location_estimation/camera_location_estimation.h"
#include "modules/perception/camera_location_estimation/config/interface_config.h"
#include "modules/perception/camera_location_estimation/config/runtime_config.h"
#include "modules/perception/common/camera/common/undistortion_handler_cv.h"
#include "modules/perception/common/camera/params/camera_params.h"
#include "modules/perception/common/fusion/lidar2camera/lidar_camera_fusion.h"
#include "modules/perception/common/lidar/convert/robosense.h"
#include "modules/perception/common/lidar/convert/velodyne.h"

namespace cfg    = jojo::perception::config;
namespace camera = jojo::perception::camera;
namespace fusion = jojo::perception::fusion;
namespace cle    = jojo::perception::cle;

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> CloudT;

class Ros1Convert {
 public:
  Ros1Convert(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
  ~Ros1Convert();

  bool Init(std::shared_ptr<cle::RuntimeConfig> param,
            std::shared_ptr<cle::InterfaceConfig> interface);
  void Run();
  void Stop();

 protected:
  void ImageCallback(const sensor_msgs::ImageConstPtr& msg);
  void ImageCompressedCallback(const sensor_msgs::CompressedImageConstPtr& msg);

  void PointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& msg);
  void PointCloudCallback(const sensor_msgs::PointCloudConstPtr& msg);

  std::shared_ptr<camera::CameraParams> camera_params_;
  std::shared_ptr<camera::UndistortionHandler> camera_undistort_;
  std::shared_ptr<fusion::LidarCameraFusion> fusion_;

 private:
  void Process(cv::Mat image, const CloudT::Ptr& cloud);

  std::shared_ptr<cle::RuntimeConfig> rparam_;
  std::shared_ptr<cle::InterfaceConfig> iparam_;

  std::unique_ptr<cle::CameraLocationEstimation> image_locator_;

  ros::NodeHandle node_;
  ros::Subscriber image_sub_;
  ros::Subscriber cloud_sub_;

  std::mutex data_mutex_;
  cv::Mat latest_image_;
  CloudT::Ptr latest_cloud_;
  bool has_image_             = false;
  bool has_cloud_             = false;
  bool undistortion_ready_    = true;
  std::atomic_bool stopping_{false};
};

#endif
