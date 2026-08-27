#ifndef DRIVER_WRAPPER_H
#define DRIVER_WRAPPER_H

#include <opencv2/opencv.hpp>

#include "modules/drivers/camera/usb_cam_cv.h"
#if defined(MJEPG)
#include "modules/drivers/camera/usb_cam_cv_jpeg.h"
#endif

using apollo::drivers::camera::config::Config;

class DriverWrapperBase {
 public:
  DriverWrapperBase() {};
  ~DriverWrapperBase() {};

  int index = -1;
  std::shared_ptr<jojo::drivers::camera::UsbCamCv> camera_device;
  std::shared_ptr<Config> conf;

  jojo::drivers::camera::CameraImagePtr raw_image = nullptr;
  cv::Mat raw_image_for_resize;

  std::string topic = "";

  bool resize_enabled = false;
};

#endif