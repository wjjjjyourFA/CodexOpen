#ifndef __OBSTACLE_CELL_H__
#define __OBSTACLE_CELL_H__

#include <cfloat>
#include <cstdint>

#include <pcl/point_types.h>

#include <opencv2/opencv.hpp>

enum ObstacleType {
  UNKNOWN = 0,
  STATIC  = 1,
  DYNAMIC = 2,
  MAX     = 3,
};

class ObstacleCell {
 public:
  ObstacleCell() { this->Reset(); }
  ~ObstacleCell() = default;

  ObstacleType type;
  float max_z;

  uint32_t count;
  bool b_valid;

  void Reset();

 public:
  void UpdateCellByType(ObstacleType type, float z);
};

#endif
