#ifndef CAMERA_INFO_H
#define CAMERA_INFO_H

#include <string>
#include <vector>

namespace jojo {
namespace perception {
namespace camera {

struct CameraInfo {
  std::string name;
  size_t width;
  size_t height;
};

}  // namespace camera
}  // namespace perception
}  // namespace jojo

#endif