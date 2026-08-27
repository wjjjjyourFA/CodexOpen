#ifndef ROS1_CONVERT_H
#define ROS1_CONVERT_H

#include <chrono>
#include <thread>

#include <ros/ros.h>
// 用image_transport软件包发布和订阅ROS中的图像
#include <image_transport/image_transport.h>
// 这两个头文件包含了CvBridge类以及与图像编码相关的函数
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio/videoio.hpp>

#include "sensor_msgs/CompressedImage.h"
//#include <opencv2/imgcodecs/legacy/constants_c.h>

#include "cyber/common/file.h"
#include "modules/common/environment_conf.h"
#include "modules/drivers/camera_tztek/driver_wrapper.h"
// #include "modules/drivers/camera/driver_wrapper_tztek.h"
#include "modules/drivers/camera/config/config_manager.h"

namespace drivers = jojo::drivers;
// namespace camera  = jojo::drivers::camera;
using apollo::drivers::camera::config::Config;

class DriverWrapper : public DriverWrapperBase {
 public:
  DriverWrapper() {};
  ~DriverWrapper() {};
};

class Ros1Convert {
 public:
  Ros1Convert();
  ~Ros1Convert();

  bool Init(ros::NodeHandle& nh, ros::NodeHandle& private_nh,
            std::shared_ptr<drivers::ConfigManager> param);
  void Run();

 protected:
  void SingleChannel(int index);
  void MultiChannel();

 private:
  std::shared_ptr<drivers::ConfigManager> param_ /*param_manager_*/;

  ros::NodeHandle node;
  std::shared_ptr<image_transport::ImageTransport> it = nullptr;
  std::string ns;

  std::vector<DriverWrapper> driver_vector;

  std::vector<std::thread> threads_;  // 可选管理线程
};

#endif  // ROS1_CONVERT_H