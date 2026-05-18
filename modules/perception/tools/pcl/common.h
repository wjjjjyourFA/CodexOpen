#ifndef __PCL_COMMON
#define __PCL_COMMON

template <typename T>
bool has_nan(T point) {
  // remove nan point, or the feature assocaion will crash, the surf point will containing nan points
  // pcl remove nan not work normally
  // ROS_ERROR("Containing nan point!");
  if (pcl_isnan(point.x) || pcl_isnan(point.y) || pcl_isnan(point.z)) {
    return true;
  } else {
    return false;
  }
}

#endif  // __PCL_COMMON