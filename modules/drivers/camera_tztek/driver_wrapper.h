#ifndef DRIVER_WRAPPER_H
#define DRIVER_WRAPPER_H

#include <opencv2/opencv.hpp>

#include "modules/drivers/camera/proto/config.pb.h"

#include "modules/common/environment_conf.h"
#include "third_party/tztek/include/mgr_camera.h"

using apollo::drivers::camera::config::Config;

class DriverWrapperBase {
 public:
  DriverWrapperBase() {};
  ~DriverWrapperBase() {};

  int index = -1;
  // std::shared_ptr<jojo::drivers::camera::UsbCamCv> camera_device;
  std::shared_ptr<CCameraMgr> camera_device;
  std::shared_ptr<Config> conf;

  std::string topic = "";

  bool resize_enabled = false;
};

#include <algorithm>

inline std::string ToUpper(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
    // return std::toupper(c);
    return static_cast<char>(std::toupper(c));
  });
  return s;
}

// 数据类型映射表：将数字编码转换为对应的字符串描述
/*
std::map<uint32_t, const char*> DateTypeMap = {
    {0, "RAW"},  // 原始图像格式
    {1, "YUYV"},  // YUV422格式（YUV交错排列，常见于视频采集）
    {2, "UYVY"},  // YUV422格式（另一种字节排列顺序）
    {3, "RAW10"},
};
*/
extern std::map<std::string, uint32_t> DateTypeMapValue;
extern std::map<uint32_t, const char*> DateTypeMap;

inline int GetVideoIndex(const std::string& dev) {
  size_t pos = dev.find_last_of("0123456789");
  if (pos == std::string::npos) return -1;

  size_t start = dev.find_last_not_of("0123456789", pos);
  return std::stoi(dev.substr(start + 1));
}

#endif
