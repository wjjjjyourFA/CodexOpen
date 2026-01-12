#ifndef ROS2_CONVERT_H
#define ROS2_CONVERT_H

#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
// 用image_transport软件包发布和订阅ROS中的图像
#include <image_transport/image_transport.hpp>
// 这两个头文件包含了CvBridge类以及与图像编码相关的函数
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include "sensor_msgs/msg/compressed_image.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/imgcodecs/legacy/constants_c.h>

#include "cyber/common/file.h"

#include "modules/drivers/camera/driver_wrapper.h"
#include "modules/drivers/camera/config/config_manager.h"

namespace drivers = jojo::drivers;
namespace camera = jojo::drivers::camera;
using apollo::drivers::camera::config::Config;

class DrvierWrapper : public DrvierWrapperBase {
 public:
  DrvierWrapper() {};
  ~DrvierWrapper() {};

  image_transport::Publisher pub;

  sensor_msgs::msg::Image::SharedPtr ros2_msg;
};

class Ros2Convert {
 public:
  Ros2Convert();
  ~Ros2Convert();

  bool Init(std::shared_ptr<rclcpp::Node> nh,
            std::shared_ptr<drivers::ConfigManager> param);
  void Run();

  void image_pub(SensorConfig& config, DrvierWrapper& driver);
  void image_compressed_pub();

 protected:
  void SingleChannel(int index);
  void MultiChannel();

 private:
  std::shared_ptr<drivers::ConfigManager> param_ /*param_manager_*/;

  std::shared_ptr<rclcpp::Node> node;
  std::shared_ptr<image_transport::ImageTransport> it;
  std::string ns;

  std::vector<DrvierWrapper> driver_vector;

  std::vector<std::thread> threads_;  // 可选管理线程
  // std::mutex mutex_;
};

#endif
