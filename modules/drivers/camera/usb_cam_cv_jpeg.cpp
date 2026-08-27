#include "modules/drivers/camera/usb_cam_cv_jpeg.h"

namespace jojo {
namespace drivers {
namespace camera {

UsbCamCvJpeg::~UsbCamCvJpeg() {
  if (m_jpegEnc) {
    m_jpegEnc->Release();
  }
}

bool UsbCamCvJpeg::init_image_processor() {
  if (pixel_format() == V4L2_PIX_FMT_MJPEG) {
    /* 不再执行 mjpeg ==> rgb 的转换
    if (init_mjpeg_decoder(camera_config()->width(),
                           camera_config()->height()) != 1) {
      return false;
    }
    */
  }

#if defined(ONLY_COMPRESSED_IMAGE)
  if (!m_jpegEnc) {
    m_jpegEnc = std::make_unique<JpegEnc>(0, camera_config()->width(),
                                         camera_config()->height(), 90);
  }
  if (!m_jpegEnc->Init()) return false;
#endif

  return true;
}

bool UsbCamCvJpeg::process_image(void* src, int len, CameraImagePtr dest,
                                 bool show) {
#ifndef ONLY_COMPRESSED_IMAGE
  return UsbCamCv::process_image(src, len, dest, show);
#else
  if (src == nullptr || dest == nullptr) {
    AERROR << "process image error. src or dest is null";
    return false;
  }

  uint8_t* data = static_cast<uint8_t*>(src);

  // 转 JPEG
  uint8_t* jpegPtr = nullptr;
  size_t jpegSize  = 0;
  m_jpegEnc->Encode(data, jpegPtr, jpegSize);
  if (jpegPtr && jpegSize > 0) {
    // 回调触发
    InvokeMJPEGCallback(jpegPtr, static_cast<int>(jpegSize));
  }
  // 控制帧率
  //  std::this_thread::sleep_for(std::chrono::milliseconds(1000 / m_fps));

  return true;
#endif
}

bool UsbCamCvJpeg::process_image_mjpeg(void* src, int len,
                                       CameraImagePtr dest) {
  if (src == nullptr || dest == nullptr) {
    AERROR << "process image error. src or dest is null";
    return false;
  }

  // MJPEG → BGR
  // way 1
  // cv::Mat jpeg(1, len, CV_8UC1, src);

  /* way 2
  std::vector<uint8_t> mjpeg_buf(len);
  memcpy(mjpeg_buf.data(), src, len);
  cv::Mat jpeg(1, len, CV_8UC1, mjpeg_buf.data());
  cv::Mat bgr = cv::imdecode(jpeg, cv::IMREAD_COLOR);

  if (bgr.empty()) {
    AERROR << "MJPEG decode failed";
    return false;
  }
  dest->image = bgr;
  */

  /* way 3
  // std::cout << "config_->height():" << config_->height() << "config_->width():" << config_->width() << std::endl;
  cv::Mat bgr(camera_config()->height(), camera_config()->width(), CV_8UC3);
  char* mjpeg_data = reinterpret_cast<char*>(src);
  this->mjpeg2rgb(mjpeg_data, len, bgr);
  dest->image = bgr;
  */

  // way 4
  InvokeMJPEGCallback(static_cast<const uint8_t*>(src), len);

  return true;
}

}  // namespace camera
}  // namespace drivers
}  // namespace jojo
