#ifndef __COLOR_CELL_H__
#define __COLOR_CELL_H__

#include <cfloat>
#include <cstdint>

#include <pcl/point_types.h>

#include <opencv2/opencv.hpp>

class ColorCell {
 public:
  ColorCell() { this->Reset(); }
  ~ColorCell() = default;

  uint8_t rgb[3];
  float max_z;

  uint32_t count;
  bool b_valid;

  void Reset();

 public:
  void UpdateCellByMaxZ(const cv::Vec3b& color, float z);

  void UpdateCellByMaxZ(const pcl::PointXYZRGB& p);

  void UpdateCellByAvg(const pcl::PointXYZRGB& p);

 protected:
  // rgb
  uint32_t sum_[3];
};

#endif
