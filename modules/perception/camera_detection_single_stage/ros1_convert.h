#ifndef CAMERA_DETECTION_SINGLE_STAGE_ROS1_CONVERT_H
#define CAMERA_DETECTION_SINGLE_STAGE_ROS1_CONVERT_H

#include <chrono>
#include <thread>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio/videoio.hpp>
//#include <opencv2/imgcodecs/legacy/constants_c.h>

#include <ros/ros.h>
// 用image_transport软件包发布和订阅ROS中的图像
#include <image_transport/image_transport.h>
// 这两个头文件包含了CvBridge类以及与图像编码相关的函数
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>

#include "sensor_msgs/CompressedImage.h"

//
#include "modules/perception/camera_detection_single_stage/config/interface_config.h"
#include "modules/perception/camera_detection_single_stage/config/runtime_config.h"
#include "modules/perception/camera_detection_single_stage/detector/yolo_obstacle_detector.h"
#include "modules/perception/common/camera/common/undistortion_handler_cv.h"
#include "modules/perception/common/camera/params/camera_params.h"
#include "modules/perception/tools/common/show_data_2d.h"

namespace base       = jojo::perception::base;
namespace cfg        = jojo::perception::config;
namespace camera     = jojo::perception::camera;
namespace perception = jojo::perception;
namespace cdss       = jojo::perception::cdss;

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> CloudT;

class Ros1Convert {
 public:
  Ros1Convert(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
  ~Ros1Convert();

  bool Init(std::shared_ptr<cdss::RuntimeConfig> param,
            std::shared_ptr<cdss::InterfaceConfig> interface);
  void Run();

  std::shared_ptr<cdss::YoloObstacleDetector> image_detector;

 protected:
  void ImageCallback(const sensor_msgs::ImageConstPtr& msg);

  void ImageCompressedCallback(const sensor_msgs::CompressedImageConstPtr& msg);

  std::shared_ptr<camera::CameraParams> camera_params;
  std::shared_ptr<camera::UndistortionHandler> camera_undistort;

 private:
  std::shared_ptr<cdss::RuntimeConfig> rparam_;
  std::shared_ptr<cdss::InterfaceConfig> iparam_;

  ros::NodeHandle node;
  ros::Subscriber image_sub;
  std::string ns;

  std::mutex mutex_;
  cv::Mat recvImg;
  uint64_t msg_time;
  bool image_recv_ = false;
};

#endif  // CAMERA_DETECTION_SINGLE_STAGE_ROS1_CONVERT_H
