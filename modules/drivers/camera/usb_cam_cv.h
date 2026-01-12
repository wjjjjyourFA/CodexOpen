#ifndef USB_CAM_SIMPLE_H
#define USB_CAM_SIMPLE_H

#pragma once

#include <malloc.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>  // 解决 O_RDWR、O_NONBLOCK 未定义问题
#include <sys/types.h>  // open() 可能还需要这个
#include <sys/stat.h>  // stat 结构体
#include <unistd.h>  // close() 等函数

// FFmpeg
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/mem.h>
#include <libswscale/swscale.h>
#include <linux/videodev2.h>
#include <libavutil/log.h>
#if LIBAVCODEC_VERSION_INT >= AV_VERSION_INT(58, 0, 0)
#include <libavutil/imgutils.h>
#endif
}

// FFmpeg
#include <libavcodec/version.h>
#if LIBAVCODEC_VERSION_MAJOR < 55
#define AV_CODEC_ID_MJPEG CODEC_ID_MJPEG
#endif

#include <memory>
#include <sstream>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/imgcodecs/legacy/constants_c.h>

// #include "cyber/cyber.h"
#include "cyber/common/log.h"

#include "modules/drivers/camera/proto/config.pb.h"

// default is v4l2

namespace jojo {
namespace drivers {
namespace camera {
using apollo::drivers::camera::config::Config;
using apollo::drivers::camera::config::IO_METHOD_MMAP;
using apollo::drivers::camera::config::IO_METHOD_READ;
using apollo::drivers::camera::config::IO_METHOD_UNKNOWN;
using apollo::drivers::camera::config::IO_METHOD_USERPTR;
// using apollo::drivers::camera::config::RGB;
// using apollo::drivers::camera::config::YUYV;

// camera raw image struct
struct CameraImage {
  int width;
  int height;
  int bytes_per_pixel;
  int image_size;
  int is_new;
  int tv_sec;
  int tv_usec;
  cv::Mat yuv_image;
  cv::Mat image;

  ~CameraImage() {
    // cv::Mat 自动释放
    // yuv_image.release();
    // image.release();
  }
};

typedef std::shared_ptr<CameraImage> CameraImagePtr;

struct buffer {
  void* start;
  size_t length;
};

class UsbCamCv {
 public:
  UsbCamCv();
  virtual ~UsbCamCv();

  virtual bool init(const std::shared_ptr<Config>& camera_config);
  // user use this function to get camera frame data
  virtual bool poll(const CameraImagePtr& raw_image);

  bool is_capturing();
  bool wait_for_device(void);

  void ReleaseDevice();

  void DebugInfo();

 protected:
  cv::Mat frame;

 private:
  int xioctl(int fd, int request, void* arg);
  bool init_device(void);
  bool uninit_device(void);

  void set_device_config();
  // enables/disable auto focus
  void set_auto_focus(int value);
  // set video device parameters
  void set_v4l_parameter(const std::string& param, int value);
  void set_v4l_parameter(const std::string& param, const std::string& value);

  int init_mjpeg_decoder(int image_width, int image_height);
  bool init_mjpeg_sws();
  // MJPEG → cv::Mat
  // void mjpeg2rgb(char* mjepg_buffer, int len, char* rgb_buffer, int pixels);
  void mjpeg2rgb(char* mjpeg_buffer, int len, cv::Mat& output);
  bool init_read(unsigned int buffer_size);
  bool init_mmap(void);
  bool init_userp(unsigned int buffer_size);
  bool close_device(void);
  bool open_device(void);
  bool read_frame(CameraImagePtr raw_image);
  bool process_image(void* src, int len, CameraImagePtr dest);
  bool process_image_mjpeg(void* src, int len, CameraImagePtr dest);
  bool start_capturing(void);
  bool stop_capturing(void);
  void reconnect();
  void reset_device();

  std::shared_ptr<Config> config_;
  int pixel_format_;
  int fd_;
  buffer* buffers_;
  unsigned int n_buffers_;
  bool is_capturing_;

  AVFrame* avframe_camera_ = nullptr;
  AVFrame* avframe_rgb_ = nullptr;
  AVCodec* avcodec_ = nullptr;
  AVDictionary* avoptions_ = nullptr;
  AVCodecContext* avcodec_context_ = nullptr;
  int avframe_camera_size_;
  int avframe_rgb_size_;
  struct SwsContext* video_sws_ = nullptr;

  float frame_warning_interval_ = 0.0;
  float frame_drop_interval_ = 0.0;
};

}  // namespace camera
}  // namespace drivers
}  // namespace jojo

#endif
