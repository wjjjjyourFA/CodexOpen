#ifndef __PCL_COMMON
#define __PCL_COMMON

#include <cmath>

/* 未公开 / 已移除
template <typename T>
bool HasNaN(T point) {
  // remove nan point, or the feature assocaion will crash, the surf point will containing nan points
  // pcl remove nan not work normally
  // ROS_ERROR("Containing nan point!");

  if (pcl_isnan(point.x) || pcl_isnan(point.y) || pcl_isnan(point.z)) {
    return true;
  } else {
    return false;
  }
}
*/

template <typename T>
inline bool HasNaN(const T& p) {
  return std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z);
}

template <typename T>
inline bool IsFinitePoint(const T& p) {
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

#endif  // __PCL_COMMON