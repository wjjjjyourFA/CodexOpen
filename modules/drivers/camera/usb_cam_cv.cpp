#include <cmath>
#include <string>

#ifndef __aarch64__
// #include "adv_trigger.h"
// #include "modules/drivers/camera/util.h"
#endif

#include "modules/drivers/camera/usb_cam_cv.h"

#define __STDC_CONSTANT_MACROS

#define CLEAR(x) memset(&(x), 0, sizeof(x))

namespace jojo {
namespace drivers {
namespace camera {

UsbCamCv::UsbCamCv()
    : fd_(-1),
      buffers_(NULL),
      n_buffers_(0),
      is_capturing_(false),
      frame_drop_interval_(0.0) {}

UsbCamCv::~UsbCamCv() {
  stop_capturing();
  uninit_device();
  close_device();

  if (pixel_format_ == V4L2_PIX_FMT_MJPEG) {
    if (video_sws_) sws_freeContext(video_sws_);
    if (avframe_camera_) av_frame_free(&avframe_camera_);
    if (avframe_rgb_) {
      if (avframe_rgb_->data[0]) {
        av_free(avframe_rgb_->data[0]);
      }
      av_frame_free(&avframe_rgb_);
    }
    if (avcodec_context_) avcodec_free_context(&avcodec_context_);
  }
}

// const std::shared_ptr<Config>& cameraconfig
// const 修饰引用 &，意思是 在函数内部不能让 cameraconfig 指向其他 shared_ptr 对象。
// 但是不影响指针指向的对象，可以通过 cameraconfig 修改内容
// 如果你想让对象内容不可改，需要加 const 在对象类型上
bool UsbCamCv::init(const std::shared_ptr<Config>& cameraconfig) {
  config_ = cameraconfig;

  if (config_->pixel_format() == "yuyv") {
    pixel_format_ = V4L2_PIX_FMT_YUYV;
  } else if (config_->pixel_format() == "uyvy") {
    pixel_format_ = V4L2_PIX_FMT_UYVY;
  } else if (config_->pixel_format() == "mjpeg") {
    pixel_format_ = V4L2_PIX_FMT_MJPEG;
  } else {
    pixel_format_ = V4L2_PIX_FMT_YUYV;
    AERROR << "Wrong pixel fromat:" << config_->pixel_format()
           << ",must be yuyv | uyvy | mjpeg ";
    return false;
  }
  if (pixel_format_ == V4L2_PIX_FMT_MJPEG) {
    if (init_mjpeg_decoder(config_->width(), config_->height()) != 1) {
      return false;
    }
  }

  // 检测帧率异常, 丢弃帧率异常的图像
  // Warning when diff with last > 1.5* interval
  frame_warning_interval_ = static_cast<float>(1.5 / config_->frame_rate());
  // now max fps 30, we use an appox time 0.9 to drop image.
  frame_drop_interval_ = static_cast<float>(0.9 / config_->frame_rate());

  return true;
}

int UsbCamCv::init_mjpeg_decoder(int image_width, int image_height) {
  // 设置日志级别
  // 所有 FFmpeg 日志都会被屏蔽，包括警告和错误
  av_log_set_level(AV_LOG_QUIET);

  // 旧版 FFmpeg（< 4.0）需要手动注册
  avcodec_register_all();

  // 找到 MJPEG 解码器
  avcodec_ = avcodec_find_decoder(AV_CODEC_ID_MJPEG);
  if (!avcodec_) {
    AERROR << "Could not find MJPEG decoder";
    return 0;
  }

  // 分配解码器上下文 AVCodecContext，后续配置参数、打开解码器都需要这个结构体
  avcodec_context_ = avcodec_alloc_context3(avcodec_);
  if (!avcodec_context_) {
    AERROR << "Decoder not initialized";
    return 0;
}

#if LIBAVCODEC_VERSION_INT >= AV_VERSION_INT(58, 0, 0)
  // 分配解码帧
  // avframe_camera_：原始摄像头帧（通常是 YUV）
  // avframe_rgb_：解码后的 RGB 图像，用于显示或 OpenCV
  // avpicture_alloc 分配 RGB 图像内存
  avframe_camera_ = av_frame_alloc();
  avframe_rgb_ = av_frame_alloc();

  // way 2
  avframe_camera_size_ =
      av_image_get_buffer_size(AV_PIX_FMT_YUV422P, image_width, image_height, 1);
  avframe_rgb_size_ = 
      av_image_get_buffer_size(AV_PIX_FMT_BGR24, image_width, image_height, 1);
  uint8_t* rgb_buffer = (uint8_t*)av_malloc(avframe_rgb_size_);
  av_image_fill_arrays(avframe_rgb_->data, avframe_rgb_->linesize, rgb_buffer,
                        AV_PIX_FMT_BGR24, image_width, image_height, 1);
#elif LIBAVCODEC_VERSION_INT >= AV_VERSION_INT(57,0,0)
  avframe_camera_ = av_frame_alloc();
  avframe_rgb_ = av_frame_alloc();

  // way 1
  avpicture_alloc(reinterpret_cast<AVPicture*>(avframe_rgb_), AV_PIX_FMT_BGR24,
                  image_width, image_height);
#else
  avframe_camera_ = avcodec_alloc_frame();
  avframe_rgb_ = avcodec_alloc_frame();

  avpicture_alloc(reinterpret_cast<AVPicture*>(avframe_rgb_), PIX_FMT_BGR24,
                  image_width, image_height);
#endif

  // 设置编码类型、图像尺寸、像素格式、视频类型。
  // 像素格式必须和摄像头输出一致（YUV422P）。
  avcodec_context_->codec_id = AV_CODEC_ID_MJPEG;
  avcodec_context_->width = image_width;
  avcodec_context_->height = image_height;
  // std::cout << "image_width:" << image_width << " x image_height:" << image_height << std::endl;

#if LIBAVCODEC_VERSION_MAJOR > 52
#if LIBAVCODEC_VERSION_INT >= AV_VERSION_INT(57, 0, 0)
  avcodec_context_->pix_fmt = AV_PIX_FMT_YUV422P;
#else
  avcodec_context_->pix_fmt = PIX_FMT_YUV422P;
#endif
  avcodec_context_->codec_type = AVMEDIA_TYPE_VIDEO;
#endif

#if LIBAVCODEC_VERSION_INT >= AV_VERSION_INT(58,0,0)
#elif LIBAVCODEC_VERSION_INT >= AV_VERSION_INT(57, 0, 0)
  // 计算帧内存大小，每帧占用的字节数
  // way 1
  avframe_camera_size_ =
      avpicture_get_size(AV_PIX_FMT_YUV422P, image_width, image_height);
  avframe_rgb_size_ =
      avpicture_get_size(AV_PIX_FMT_BGR24, image_width, image_height);
#else
  avframe_camera_size_ =
      avpicture_get_size(PIX_FMT_YUV422P, image_width, image_height);
  avframe_rgb_size_ =
      avpicture_get_size(PIX_FMT_RGB24, image_width, image_height);
#endif

  /* open it */
  // 打开解码器
  // 之后用 avcodec_send_packet() / avcodec_receive_frame() 来解码 MJPEG 数据。
  // FFmpeg >= 3.x 用 avcodec_send_packet / avcodec_receive_frame，
  // 旧版用 avcodec_decode_video2
  if (avcodec_open2(avcodec_context_, avcodec_, &avoptions_) < 0) {
    AERROR << "Could not open MJPEG Decoder";
    return 0;
  }
  return init_mjpeg_sws();
}

bool UsbCamCv::init_mjpeg_sws() {
  // 初始化 SWSContext，仅初始化一次

  int xsize = avcodec_context_->width;
  int ysize = avcodec_context_->height;

#if LIBAVCODEC_VERSION_INT >= AV_VERSION_INT(57, 0, 0)
  video_sws_ =
      sws_getContext(xsize, ysize, avcodec_context_->pix_fmt, xsize, ysize,
                     AV_PIX_FMT_BGR24, SWS_BILINEAR, nullptr, nullptr, nullptr);
#else
  video_sws_ =
      sws_getContext(xsize, ysize, avcodec_context_->pix_fmt, xsize, ysize,
                     PIX_FMT_BGR24, SWS_BILINEAR, nullptr, nullptr, nullptr);
#endif

  if (!video_sws_) {
    fprintf(stderr, "Could not initialize SwsContext\n");
    return false;
  }

  return true;
}

void UsbCamCv::mjpeg2rgb(char* mjpeg_buffer, int len, cv::Mat& output) {
  // uint8_t* mjpeg_buffer; // 指向 MJPEG 压缩数据
  // int len;               // MJPEG 数据长度

  int got_picture = 0;

#if LIBAVCODEC_VERSION_INT >= AV_VERSION_INT(57, 0, 0)
  // FFmpeg >= 3.1 / libavcodec >= 57，新 API
  AVPacket avpkt;
  av_init_packet(&avpkt);

  avpkt.size = len;
  avpkt.data = reinterpret_cast<uint8_t*>(mjpeg_buffer);

  // 发送数据包到解码器
  int ret = avcodec_send_packet(avcodec_context_, &avpkt);
  if (ret < 0) {
    char errbuf[128];
    av_strerror(ret, errbuf, sizeof(errbuf));
    AERROR << "avcodec_send_packet failed: " << errbuf << " (" << ret << ")";
    return;
  }

  // 接收解码帧
  ret = avcodec_receive_frame(avcodec_context_, avframe_camera_);
  if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
    AERROR << "No frame received";
    return;
  }
  if (ret < 0) {
    AERROR << "Error while decoding frame";
    return;
  }
  got_picture = 1;
#elif LIBAVCODEC_VERSION_MAJOR > 52
  // FFmpeg 53 ~ 56 使用旧 API
  int decoded_len;
  AVPacket avpkt;
  av_init_packet(&avpkt);

  avpkt.size = len;
  avpkt.data = (unsigned char*)mjpeg_buffer;
  decoded_len = avcodec_decode_video2(avcodec_context_, avframe_camera_,
                                      &got_picture, &avpkt);

  if (decoded_len < 0) {
    AERROR << "Error while decoding frame.";
    return;
  }
#else
  avcodec_decode_video(avcodec_context_, avframe_camera_, &got_picture,
                       reinterpret_cast<uint8_t*>(mjpeg_buffer), len);
#endif

  if (!got_picture) {
    AERROR << "Camera: expected picture but didn't get it...";
    return;
  }

  int xsize = avcodec_context_->width;
  int ysize = avcodec_context_->height;
  int pic_size = avpicture_get_size(avcodec_context_->pix_fmt, xsize, ysize);
  if (pic_size != avframe_camera_size_) {
    AERROR << "outbuf size mismatch.  pic_size:" << pic_size
           << ",buffer_size:" << avframe_camera_size_;
    return;
  }

  // cv::Mat 输出
  if (output.empty() || output.cols != xsize || output.rows != ysize) {
      output.create(xsize, ysize, CV_8UC3);
  }
  // std::cout << "output.cols:" << output.cols << ", output.rows:" << output.rows << std::endl;

  // 使用 SwsContext 转换到 BGR24
  uint8_t* dst_data[4] = { output.data, nullptr, nullptr, nullptr };
  int dst_linesize[4] = { static_cast<int>(output.step), 0, 0, 0 };

  sws_scale(video_sws_, avframe_camera_->data, avframe_camera_->linesize, 0,
          ysize, dst_data, dst_linesize);
}

// 对外接口
bool UsbCamCv::poll(const CameraImagePtr& raw_image) {
  raw_image->is_new = 0;
  // free memory in this struct desturctor
  // memset(raw_image->image, 0, raw_image->image_size * sizeof(char));

  fd_set fds;
  struct timeval tv;
  int r = 0;

  FD_ZERO(&fds);
  FD_SET(fd_, &fds);

  /* Timeout. */
  tv.tv_sec = 2;
  tv.tv_usec = 0;

  r = select(fd_ + 1, &fds, nullptr, nullptr, &tv);

  if (-1 == r) {
    if (EINTR == errno) {
      return false;
    }

    // errno_exit("select");
    reconnect();
  }

  if (0 == r) {
    AERROR << "select timeout";
    reconnect();
  }

  int get_new_image = read_frame(raw_image);

  if (!get_new_image) {
    return false;
  }

  raw_image->is_new = 1;
  return true;
}

bool UsbCamCv::open_device(void) {
  struct stat st;

  if (-1 == stat(config_->camera_dev().c_str(), &st)) {
    AERROR << "Cannot identify '" << config_->camera_dev() << "': " << errno
           << ", " << strerror(errno);
    return false;
  }

  if (!S_ISCHR(st.st_mode)) {
    AERROR << config_->camera_dev() << " is no device";
    return false;
  }

  fd_ = open(config_->camera_dev().c_str(), O_RDWR /* required */ | O_NONBLOCK,
             0);

  if (-1 == fd_) {
    AERROR << "Cannot open '" << config_->camera_dev() << "': " << errno << ", "
           << strerror(errno);
    return false;
  }

  return true;
}

bool UsbCamCv::init_device(void) {
  // 查询设备属性： VIDIOC_QUERYCAP
  // 驱动名字 设备名字
  // 设备在系统中的位置 驱动版本号
  // 设备支持的操作 保留字段
  struct v4l2_capability cap;
  struct v4l2_cropcap cropcap;
  struct v4l2_crop crop;
  struct v4l2_format fmt;
  unsigned int min = 0;

  if (-1 == xioctl(fd_, VIDIOC_QUERYCAP, &cap)) {
    if (EINVAL == errno) {
      AERROR << config_->camera_dev() << " is no V4L2 device";
      return false;
    }
    AERROR << "VIDIOC_QUERYCAP";
    return false;
  }

  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    AERROR << config_->camera_dev() << " is no video capture device";
    return false;
  }

  switch (config_->io_method()) {
    case IO_METHOD_READ:
      if (!(cap.capabilities & V4L2_CAP_READWRITE)) {
        AERROR << config_->camera_dev() << " does not support read i/o";
        return false;
      }

      break;

    case IO_METHOD_MMAP:
    case IO_METHOD_USERPTR:
      if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
        AERROR << config_->camera_dev() << " does not support streaming i/o";
        return false;
      }

      break;
    case IO_METHOD_UNKNOWN:
      AERROR << config_->camera_dev() << " does not support unknown i/o";
      return false;
      break;
  }

  /* Select video input, video standard and tune here. */
  CLEAR(cropcap);

  cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  if (0 == xioctl(fd_, VIDIOC_CROPCAP, &cropcap)) {
    crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    crop.c = cropcap.defrect; /* reset to default */

    if (-1 == xioctl(fd_, VIDIOC_S_CROP, &crop)) {
      switch (errno) {
        case EINVAL:
          /* Cropping not supported. */
          break;
      }
    }
  }

  CLEAR(fmt);

  // fmt 是一个联合体结构（v4l2_format 里是 union fmt），
  // 需要告诉驱动程序你是要设置哪种类型的视频流（比如 V4L2_BUF_TYPE_VIDEO_CAPTURE）
  // 否则系统可能误解析 fmt.fmt.pix，导致行为未定义或返回错误。
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = config_->width();
  fmt.fmt.pix.height = config_->height();
  fmt.fmt.pix.pixelformat = pixel_format_;
  // 交错扫描
  // fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
  // 默认逐行扫描
  fmt.fmt.pix.field = V4L2_FIELD_NONE;  

  // VIDIOC_S_FMT 设置新格式（更改模式）
  // VIDIOC_S_FMT 并不是强制生效！它只是“尽量接受”你设置的格式
  // 调用 VIDIOC_S_FMT 设置格式之后，还应该立刻调用 VIDIOC_G_FMT 来确认驱动实际采用的格式！
  if (-1 == xioctl(fd_, VIDIOC_S_FMT, &fmt)) {
    AERROR << "VIDIOC_S_FMT";
    return false;
  }

  // VIDIOC_G_FMT 获取当前格式
  if (-1 == xioctl(fd_, VIDIOC_G_FMT, &fmt)) {
    AERROR << "VIDIOC_G_FMT";
    return false;
  }
  pixel_format_ = fmt.fmt.pix.pixelformat;

  /*
  int tmp_pixel_format = fmt.fmt.pix.pixelformat;
  char fourcc[5] = {
    (char)(tmp_pixel_format & 0xFF),
    (char)((tmp_pixel_format >> 8) & 0xFF),
    (char)((tmp_pixel_format >> 16) & 0xFF),
    (char)((tmp_pixel_format >> 24) & 0xFF),
    0
  };

  AINFO << "Default format: " << fmt.fmt.pix.width << "x" << fmt.fmt.pix.height;
  AINFO << "Default pixel format: " << fourcc;
  AINFO << "Custom format: " << config_->width() << "x" << config_->height();
  AINFO << "Custom pixel format: " << config_->pixel_format();
  */
  /* Note VIDIOC_S_FMT may change width and height. */

  /* Buggy driver paranoia. */
  min = fmt.fmt.pix.width * 2;

  if (fmt.fmt.pix.bytesperline < min) {
    fmt.fmt.pix.bytesperline = min;
  }

  min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;

  if (fmt.fmt.pix.sizeimage < min) {
    fmt.fmt.pix.sizeimage = min;
  }
  // 虽然是通过外部配置文件设置分辨率，但有可能相机不接受分辨率设置
  // 因此需要依据实际的分辨率，更新 config_ 里的分辨率
  config_->set_width(fmt.fmt.pix.width);
  config_->set_height(fmt.fmt.pix.height);

  struct v4l2_streamparm stream_params;
  memset(&stream_params, 0, sizeof(stream_params));
  stream_params.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  // if (xioctl(fd_, VIDIOC_G_PARM, &stream_params) < 0) {
  //   // errno_exit("Couldn't query v4l fps!");
  //   AERROR << "Couldn't query v4l fps!";
  //   // reconnect();
  //   return false;
  // }

  AINFO << "Capability flag: 0x" << stream_params.parm.capture.capability;

  // 设置 V4L2（Video for Linux 2）摄像头采集帧率参数
  // 摄像头驱动是否支持你设置的 timeperframe。
  // 某些设备只支持特定帧率（比如 15, 30, 60），否则会自动调整。
  // 设置后建议用 VIDIOC_G_PARM 再读一次看看实际设置是否生效。
  stream_params.parm.capture.timeperframe.numerator   = 1;
  stream_params.parm.capture.timeperframe.denominator = config_->frame_rate();

  if (xioctl(fd_, VIDIOC_S_PARM, &stream_params) < 0) {
    AINFO << "Couldn't set camera framerate";
  } else {
    double fps = double(stream_params.parm.capture.timeperframe.denominator) /
                 stream_params.parm.capture.timeperframe.numerator;
    AINFO << "Set framerate to be " << config_->frame_rate();
    AINFO << "Real framerate to be " << fps;
    config_->set_frame_rate(fps);
  }

  switch (config_->io_method()) {
    case IO_METHOD_READ:
      init_read(fmt.fmt.pix.sizeimage);
      break;

    case IO_METHOD_MMAP:
      init_mmap();
      break;

    case IO_METHOD_USERPTR:
      init_userp(fmt.fmt.pix.sizeimage);
      break;

    case IO_METHOD_UNKNOWN:
      AERROR << " does not support unknown i/o";
      break;
  }

  return true;
}

// xioctl 是一个 增强版 ioctl 调用，主要用于 安全、稳定地与摄像头（V4L2 设备）交互
int UsbCamCv::xioctl(int fd, int request, void* arg) {
  int r = 0;
  do {
    r = ioctl(fd, request, arg);
  } while (-1 == r && EINTR == errno);

  return r;
}

// 直接复制数据
bool UsbCamCv::init_read(unsigned int buffer_size) {
  buffers_ = reinterpret_cast<buffer*>(calloc(1, sizeof(*buffers_)));

  if (!buffers_) {
    AERROR << "Out of memory";
    // exit(EXIT_FAILURE);
    // reconnect();
    return false;
  }

  buffers_[0].length = buffer_size;
  buffers_[0].start = malloc(buffer_size);

  if (!buffers_[0].start) {
    AERROR << "Out of memory";
    return false;
  }

  return true;
}

bool UsbCamCv::init_mmap(void) {
  struct v4l2_requestbuffers req;
  CLEAR(req);

  // Maybe
  // req.count 的值较小，缓存较小，则无法存储所有相机的串行数据，
  // 导致采集帧率小于相机实际帧率
  req.count = 1;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  if (-1 == xioctl(fd_, VIDIOC_REQBUFS, &req)) {
    if (EINVAL == errno) {
      AERROR << config_->camera_dev() << " does not support memory mapping";
      return false;
    }
    return false;
  }

  buffers_ = reinterpret_cast<buffer*>(calloc(req.count, sizeof(*buffers_)));

  if (!buffers_) {
    AERROR << "Out of memory";
    return false;
  }

  for (n_buffers_ = 0; n_buffers_ < req.count; ++n_buffers_) {
    struct v4l2_buffer buf;
    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = n_buffers_;

    // 查询序号为n_buffers 的缓冲区，得到其起始物理地址和大小
    if (-1 == xioctl(fd_, VIDIOC_QUERYBUF, &buf)) {
      AERROR << "VIDIOC_QUERYBUF";
      return false;
    }

    // 映射内存
    buffers_[n_buffers_].length = buf.length;
    buffers_[n_buffers_].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                                      MAP_SHARED, fd_, buf.m.offset);

    if (MAP_FAILED == buffers_[n_buffers_].start) {
      AERROR << "mmap";
      return false;
    }
  }

  return true;
}

bool UsbCamCv::init_userp(unsigned int buffer_size) {
  struct v4l2_requestbuffers req;
  unsigned int page_size = 0;

  page_size = getpagesize();
  buffer_size = (buffer_size + page_size - 1) & ~(page_size - 1);

  CLEAR(req);

  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_USERPTR;

  if (-1 == xioctl(fd_, VIDIOC_REQBUFS, &req)) {
    if (EINVAL == errno) {
      AERROR << config_->camera_dev()
             << " does not support "
                "user pointer i/o";
      return false;
    }
    AERROR << "VIDIOC_REQBUFS";
    return false;
  }

  buffers_ = reinterpret_cast<buffer*>(calloc(4, sizeof(*buffers_)));

  if (!buffers_) {
    AERROR << "Out of memory";
    return false;
  }

  for (n_buffers_ = 0; n_buffers_ < 4; ++n_buffers_) {
    buffers_[n_buffers_].length = buffer_size;
    buffers_[n_buffers_].start =
        memalign(/* boundary */ page_size, buffer_size);

    if (!buffers_[n_buffers_].start) {
      AERROR << "Out of memory";
      return false;
    }
  }

  return true;
}

bool UsbCamCv::start_capturing(void) {
  if (is_capturing_) {
    return true;
  }

  unsigned int i = 0;
  enum v4l2_buf_type type;

  switch (config_->io_method()) {
    case IO_METHOD_READ:
      /* Nothing to do. */
      break;

    case IO_METHOD_MMAP:
      for (i = 0; i < n_buffers_; ++i) {
        struct v4l2_buffer buf;
        CLEAR(buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        if (-1 == xioctl(fd_, VIDIOC_QBUF, &buf)) {
          AERROR << "VIDIOC_QBUF";
          return false;
        }
      }

      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

      if (-1 == xioctl(fd_, VIDIOC_STREAMON, &type)) {
        AERROR << "VIDIOC_STREAMON";
        return false;
      }

      break;

    case IO_METHOD_USERPTR:
      for (i = 0; i < n_buffers_; ++i) {
        struct v4l2_buffer buf;

        CLEAR(buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_USERPTR;
        buf.index = i;
        buf.m.userptr = reinterpret_cast<uint64_t>(buffers_[i].start);
        buf.length = static_cast<unsigned int>(buffers_[i].length);

        if (-1 == xioctl(fd_, VIDIOC_QBUF, &buf)) {
          AERROR << "VIDIOC_QBUF";
          return false;
        }
      }

      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

      if (-1 == xioctl(fd_, VIDIOC_STREAMON, &type)) {
        AERROR << "VIDIOC_STREAMON";
        return false;
      }

      break;

    case IO_METHOD_UNKNOWN:
      AERROR << "unknown IO";
      return false;
      break;
  }

  is_capturing_ = true;
  return true;
}

void UsbCamCv::set_device_config() {
  if (config_->brightness() >= 0) {
    set_v4l_parameter("brightness", config_->brightness());
  }

  if (config_->contrast() >= 0) {
    set_v4l_parameter("contrast", config_->contrast());
  }

  if (config_->saturation() >= 0) {
    set_v4l_parameter("saturation", config_->saturation());
  }

  if (config_->sharpness() >= 0) {
    set_v4l_parameter("sharpness", config_->sharpness());
  }

  if (config_->gain() >= 0) {
    set_v4l_parameter("gain", config_->gain());
  }

  // check auto white balance
  if (config_->auto_white_balance()) {
    set_v4l_parameter("white_balance_temperature_auto", 1);
  } else {
    set_v4l_parameter("white_balance_temperature_auto", 0);
    set_v4l_parameter("white_balance_temperature", config_->white_balance());
  }

  // check auto exposure
  if (!config_->auto_exposure()) {
    // turn down exposure control (from max of 3)
    set_v4l_parameter("auto_exposure", 1);
    // change the exposure level
    set_v4l_parameter("exposure_absolute", config_->exposure());
  }

  // check auto focus
  if (config_->auto_focus()) {
    set_auto_focus(1);
    set_v4l_parameter("focus_auto", 1);
  } else {
    set_v4l_parameter("focus_auto", 0);
    if (config_->focus() >= 0) {
      set_v4l_parameter("focus_absolute", config_->focus());
    }
  }
}

bool UsbCamCv::uninit_device(void) {
  unsigned int i = 0;

  switch (config_->io_method()) {
    case IO_METHOD_READ:
      free(buffers_[0].start);
      break;

    case IO_METHOD_MMAP:
      for (i = 0; i < n_buffers_; ++i) {
        if (-1 == munmap(buffers_[i].start, buffers_[i].length)) {
          AERROR << "munmap";
          return false;
        }
      }

      break;

    case IO_METHOD_USERPTR:
      for (i = 0; i < n_buffers_; ++i) {
        free(buffers_[i].start);
      }

      break;

    case IO_METHOD_UNKNOWN:
      AERROR << "unknown IO";
      break;
  }

  return true;
}

bool UsbCamCv::close_device(void) {
  // close(fd) 作用就是关闭一个已经打开的文件描述符 fd
  // （包括普通文件、socket、串口、视频设备 /dev/video0 等）。
  if (-1 == close(fd_)) {
    AERROR << "close";
    return false;
  }

  fd_ = -1;
  return true;
}

bool UsbCamCv::stop_capturing(void) {
  if (!is_capturing_) {
    return true;
  }

  is_capturing_ = false;
  enum v4l2_buf_type type;

  switch (config_->io_method()) {
    case IO_METHOD_READ:
      /* Nothing to do. */
      break;

    case IO_METHOD_MMAP:
    case IO_METHOD_USERPTR:
      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

      if (-1 == xioctl(fd_, VIDIOC_STREAMOFF, &type)) {
        AERROR << "VIDIOC_STREAMOFF";
        return false;
      }

      break;
    case IO_METHOD_UNKNOWN:
      AERROR << "unknown IO";
      return false;
      break;
  }

  return true;
}

bool UsbCamCv::read_frame(CameraImagePtr raw_image) {
  struct v4l2_buffer buf;
  unsigned int i = 0;
  int len = 0;

  switch (config_->io_method()) {
    case IO_METHOD_READ:
      len = static_cast<int>(read(fd_, buffers_[0].start, buffers_[0].length));

      if (len == -1) {
        switch (errno) {
          case EAGAIN:
            AINFO << "EAGAIN";
            return false;

          case EIO:

            /* Could ignore EIO, see spec. */

            /* fall through */

          default:
            AERROR << "read";
            return false;
        }
      }

      process_image(buffers_[0].start, len, raw_image);
      break;

    case IO_METHOD_MMAP:
      CLEAR(buf);

      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;

      if (-1 == xioctl(fd_, VIDIOC_DQBUF, &buf)) {
        switch (errno) {
          case EAGAIN:
            return false;

          case EIO:

            /* Could ignore EIO, see spec. */

            /* fall through */

          default:
            AERROR << "VIDIOC_DQBUF";
            reconnect();
            return false;
        }
      }
      assert(buf.index < n_buffers_);
      len = buf.bytesused;

      raw_image->tv_sec  = static_cast<int>(buf.timestamp.tv_sec);
      raw_image->tv_usec = static_cast<int>(buf.timestamp.tv_usec);

      // std::cout << "raw_image->width: " << raw_image->width << std::endl;

      if (pixel_format_ == V4L2_PIX_FMT_MJPEG) {
        if (len <= 0) {
          AERROR << "Empty MJPEG frame";
          return false;
        }
        else {
          process_image_mjpeg(buffers_[buf.index].start, len, raw_image);
        }
      } else {
        if (len < raw_image->width * raw_image->height) {
          AERROR << "Wrong Buffer Len: " << len
                << ", dev: " << config_->camera_dev();
        } else {
          process_image(buffers_[buf.index].start, len, raw_image);
        }
      }

      if (-1 == xioctl(fd_, VIDIOC_QBUF, &buf)) {
        AERROR << "VIDIOC_QBUF";
        return false;
      }

      break;

    case IO_METHOD_USERPTR:
      CLEAR(buf);

      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_USERPTR;

      if (-1 == xioctl(fd_, VIDIOC_DQBUF, &buf)) {
        switch (errno) {
          case EAGAIN:
            return false;

          case EIO:

            /* Could ignore EIO, see spec. */

            /* fall through */

          default:
            AERROR << "VIDIOC_DQBUF";
            return false;
        }
      }

      for (i = 0; i < n_buffers_; ++i) {
        if (buf.m.userptr == reinterpret_cast<uint64_t>(buffers_[i].start) &&
            buf.length == buffers_[i].length) {
          break;
        }
      }

      assert(i < n_buffers_);
      len = buf.bytesused;
      process_image(reinterpret_cast<void*>(buf.m.userptr), len, raw_image);

      if (-1 == xioctl(fd_, VIDIOC_QBUF, &buf)) {
        AERROR << "VIDIOC_QBUF";
        return false;
      }

      break;

    case IO_METHOD_UNKNOWN:
      AERROR << "unknown IO";
      return false;
      break;
  }

  return true;
}

bool UsbCamCv::process_image(void* src, int len, CameraImagePtr dest) {
  if (src == nullptr || dest == nullptr) {
    AERROR << "process image error. src or dest is null";
    return false;
  }
  if (pixel_format_ == V4L2_PIX_FMT_YUYV ||
      pixel_format_ == V4L2_PIX_FMT_UYVY) {
    // Equal to Opencv
    // 确保 cv::Mat 分配好内存
    // YUV422
    // cv::Mat yuv_frame = cv::Mat(config_->height, config_->width, CV_8UC2);
    // BGR
    // cv::Mat frame = cv::Mat(config_->height, config_->width, CV_8UC3);

    // static int capture_buf_size = config_->width() * config_->height() * 2;
    int capture_buf_size = config_->width() * config_->height() * 2;
    memcpy(dest->yuv_image.data, src, capture_buf_size);

    /* // 打印 src 的前8字节
    uint8_t* ptr = static_cast<uint8_t*>(src);
    printf("First bytes: ");
    for (int i = 0; i < 8; ++i) {
      printf("%02X ", ptr[i]);
    }
    printf("\n");
    */

    // 转换为 BGR
    // ==> 默认 config_->output_type() == RGB
    if (pixel_format_ == V4L2_PIX_FMT_YUYV) {
      cv::cvtColor(dest->yuv_image, dest->image, cv::COLOR_YUV2BGR_YUYV);
    } else if (pixel_format_ == V4L2_PIX_FMT_UYVY) {
      // 似乎 v4l2 在之前设置好 fmt.fmt.pix.pixelformat 后，
      // 摄像头读取到的数据 会自动将 YUYV 转为 UYVY，而后续不用再转换
      cv::cvtColor(dest->yuv_image, dest->image, cv::COLOR_YUV2BGR_UYVY);
      // cv::cvtColor(dest->yuv_image, dest->image, cv::COLOR_YUV2BGR_YUYV);
    }
  } else {
    AERROR << "unsupported pixel format:" << pixel_format_;
    return false;
  }
  return true;
}

bool UsbCamCv::process_image_mjpeg(void* src, int len, CameraImagePtr dest) {
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

  // /* way 3
  // std::cout << "config_->height():" << config_->height() << "config_->width():" << config_->width() << std::endl;
  cv::Mat bgr(config_->height(), config_->width(), CV_8UC3);
  char* mjpeg_data = reinterpret_cast<char*>(src);
  this->mjpeg2rgb(mjpeg_data, len, bgr);
  dest->image = bgr;
  // */

  return true;
}

bool UsbCamCv::is_capturing() { return is_capturing_; }

// enables/disables auto focus
void UsbCamCv::set_auto_focus(int value) {
  struct v4l2_queryctrl queryctrl;
  struct v4l2_ext_control control;

  memset(&queryctrl, 0, sizeof(queryctrl));
  queryctrl.id = V4L2_CID_FOCUS_AUTO;

  if (-1 == xioctl(fd_, VIDIOC_QUERYCTRL, &queryctrl)) {
    if (errno != EINVAL) {
      perror("VIDIOC_QUERYCTRL");
      return;
    } else {
      AINFO << "V4L2_CID_FOCUS_AUTO is not supported";
      return;
    }
  } else if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
    AINFO << "V4L2_CID_FOCUS_AUTO is not supported";
    return;
  } else {
    memset(&control, 0, sizeof(control));
    control.id = V4L2_CID_FOCUS_AUTO;
    control.value = value;

    if (-1 == xioctl(fd_, VIDIOC_S_CTRL, &control)) {
      perror("VIDIOC_S_CTRL");
      return;
    }
  }
}

/**
 * Set video device parameter via call to v4l-utils.
 *
 * @param param The name of the parameter to set
 * @param param The value to assign
 */
void UsbCamCv::set_v4l_parameter(const std::string& param, int value) {
  set_v4l_parameter(param, std::to_string(value));
}
/**
 * Set video device parameter via call to v4l-utils.
 *
 * @param param The name of the parameter to set
 * @param param The value to assign
 */
void UsbCamCv::set_v4l_parameter(const std::string& param,
                               const std::string& value) {
  // build the command
  std::stringstream ss;
  ss << "v4l2-ctl --device=" << config_->camera_dev() << " -c " << param << "="
     << value << " 2>&1";
  std::string cmd = ss.str();

  // capture the output
  std::string output;
  char buffer[256];
  FILE* stream = popen(cmd.c_str(), "r");
  if (stream) {
    while (!feof(stream)) {
      if (fgets(buffer, 256, stream) != nullptr) {
        output.append(buffer);
      }
    }

    pclose(stream);
    // any output should be an error
    if (output.length() > 0) {
      AERROR << output.c_str();
    }
  } else {
    AERROR << "usb_cam_node could not run " << cmd.c_str();
  }
}

bool UsbCamCv::wait_for_device() {
  if (is_capturing_) {
    ADEBUG << "is capturing";
    return true;
  }
  if (!open_device()) {
    return false;
  }
  if (!init_device()) {
    close_device();
    return false;
  }
  if (!start_capturing()) {
    uninit_device();
    close_device();
    return false;
  }
#ifndef __aarch64__
  // will continue when trigger failed for self-trigger camera
  // set_adv_trigger();
#endif
  return true;
}

#ifdef __aarch64__
int UsbCam::convert_yuv_to_rgb_pixel(int y, int u, int v) {
  unsigned int pixel32 = 0;
  unsigned char* pixel = (unsigned char*)&pixel32;
  int r, g, b;
  r = (int)((double)y + (1.370705 * ((double)v - 128.0)));
  g = (int)((double)y - (0.698001 * ((double)v - 128.0)) -
            (0.337633 * ((double)u - 128.0)));
  b = (int)((double)y + (1.732446 * ((double)u - 128.0)));
  if (r > 255) r = 255;
  if (g > 255) g = 255;
  if (b > 255) b = 255;
  if (r < 0) r = 0;
  if (g < 0) g = 0;
  if (b < 0) b = 0;
  pixel[0] = (unsigned char)r;
  pixel[1] = (unsigned char)g;
  pixel[2] = (unsigned char)b;
  return pixel32;
}

int UsbCam::convert_yuv_to_rgb_buffer(unsigned char* yuv, unsigned char* rgb,
                                      unsigned int width, unsigned int height) {
  unsigned int in, out = 0;
  unsigned int pixel_16;
  unsigned char pixel_24[3];
  unsigned int pixel32;
  int y0, u, y1, v;

  for (in = 0; in < width * height * 2; in += 4) {
    pixel_16 =
        yuv[in + 3] << 24 | yuv[in + 2] << 16 | yuv[in + 1] << 8 | yuv[in + 0];
    y0 = (pixel_16 & 0x000000ff);
    u = (pixel_16 & 0x0000ff00) >> 8;
    y1 = (pixel_16 & 0x00ff0000) >> 16;
    v = (pixel_16 & 0xff000000) >> 24;
    pixel32 = convert_yuv_to_rgb_pixel(y0, u, v);
    pixel_24[0] = (unsigned char)(pixel32 & 0x000000ff);
    pixel_24[1] = (unsigned char)((pixel32 & 0x0000ff00) >> 8);
    pixel_24[2] = (unsigned char)((pixel32 & 0x00ff0000) >> 16);
    rgb[out++] = pixel_24[0];
    rgb[out++] = pixel_24[1];
    rgb[out++] = pixel_24[2];
    pixel32 = convert_yuv_to_rgb_pixel(y1, u, v);
    pixel_24[0] = (unsigned char)(pixel32 & 0x000000ff);
    pixel_24[1] = (unsigned char)((pixel32 & 0x0000ff00) >> 8);
    pixel_24[2] = (unsigned char)((pixel32 & 0x00ff0000) >> 16);
    rgb[out++] = pixel_24[0];
    rgb[out++] = pixel_24[1];
    rgb[out++] = pixel_24[2];
  }
  return 0;
}
#endif

void UsbCamCv::reconnect() {
  stop_capturing();
  uninit_device();
  close_device();
}

void UsbCamCv::ReleaseDevice() {
  stop_capturing();
  uninit_device();
  close_device();
}

void UsbCamCv::DebugInfo() {
  AINFO << "[UsbCam Config Loaded]"
        << "\ncamera_dev       : " << config_->camera_dev()
        << "\nframe_id         : " << config_->frame_id()
        << "\npixel_format     : " << config_->pixel_format()
        << "\nwidth            : " << config_->width()
        << "\nheight           : " << config_->height()
        << "\nframe_rate       : " << config_->frame_rate()
        << "\nraw_channel_name : " << config_->raw_channel_name();
}

}  // namespace camera
}  // namespace drivers
}  // namespace jojo
