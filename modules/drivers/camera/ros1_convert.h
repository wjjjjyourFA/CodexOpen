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
//#include <opencv2/imgcodecs/legacy/constants_c.h>

#include "sensor_msgs/CompressedImage.h"

//
#include "cyber/common/file.h"
#include "modules/common/environment_conf.h"
#include "modules/drivers/camera/driver_wrapper.h"
// #include "modules/drivers/camera/driver_wrapper_tztek.h"
#include "modules/drivers/camera/config/config_manager.h"

namespace drivers = jojo::drivers;
namespace camera  = jojo::drivers::camera;
using apollo::drivers::camera::config::Config;

class DriverWrapper : public DriverWrapperBase {
 public:
  DriverWrapper() {};
  ~DriverWrapper() {};

#if defined(ONLY_COMPRESSED_IMAGE)
  ros::Publisher pub;
  // sensor_msgs::CompressedImagePtr ros1_msg;

  static constexpr size_t BSIZE = 3;
  std::array<boost::shared_ptr<sensor_msgs::CompressedImage>, BSIZE> pool;
  int idx = 0;

  void Init() {
    for (auto& msg : pool) {
      msg = boost::make_shared<sensor_msgs::CompressedImage>();
      msg->data.reserve(MAX_JPEG_SIZE);
    }
  }
#else
  image_transport::Publisher pub;
  sensor_msgs::ImagePtr ros1_msg;

  void Init() {}
#endif

  int MAX_JPEG_SIZE;
};

class Ros1Convert {
 public:
  Ros1Convert(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
  ~Ros1Convert();

  bool Init(std::shared_ptr<drivers::ConfigManager> param);
  void Run();

  void image_pub(DriverWrapper& driver);
  void image_compressed_pub(DriverWrapper& driver, const uint8_t* pData,
                            int nDatalen);

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
  // std::mutex mutex_;
};

#endif  // ROS1_CONVERT_H