#ifndef USB_CAM_CV_JPEG_H
#define USB_CAM_CV_JPEG_H

#pragma once

#include <memory>

#include "modules/drivers/camera/jpeg_encode.h"
#include "modules/drivers/camera/usb_cam_cv.h"

namespace jojo {
namespace drivers {
namespace camera {

// typedef void (*JPEG_CALLBACK)(int nChan, struct timespec stTime, int nWidth,
//                               int nHeight, unsigned char* pData, int nDatalen,
//                               void* pUserData);

// UsbCamCv 负责公共的 V4L2 采集流程，本类只处理 JPEG 输出差异。
class UsbCamCvJpeg : public UsbCamCv {
 public:
  UsbCamCvJpeg() = default;
  ~UsbCamCvJpeg() override;

 protected:
  bool init_image_processor() override;
  bool process_image(void* src, int len, CameraImagePtr dest,
                     bool show = false) override;
  bool process_image_mjpeg(void* src, int len, CameraImagePtr dest) override;

 private:
  // JPEG Encoder
  std::unique_ptr<JpegEnc> m_jpegEnc = nullptr;
};

}  // namespace camera
}  // namespace drivers
}  // namespace jojo

#endif
