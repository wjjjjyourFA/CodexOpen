#ifndef DRIVER_WRAPPER_H
#define DRIVER_WRAPPER_H

#include <opencv2/opencv.hpp>

#include "modules/drivers/camera/usb_cam_cv.h"
 
namespace camera = jojo::drivers::camera;
using apollo::drivers::camera::config::Config;

class DrvierWrapperBase {
 public:
  DrvierWrapperBase() {};
  ~DrvierWrapperBase() {};

  int index = -1;
  std::shared_ptr<camera::UsbCamCv> camera_device;
  std::shared_ptr<Config> conf;

  camera::CameraImagePtr raw_image = nullptr;
  cv::Mat raw_image_for_resize;

  std::string topic = "";

  bool resize_enabled = false;
};

#endif