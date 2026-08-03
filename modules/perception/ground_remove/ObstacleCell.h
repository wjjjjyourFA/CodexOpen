#ifndef GROUND_REMOVE_OBSTACLE_CELL_H
#define GROUND_REMOVE_OBSTACLE_CELL_H

#include <cfloat>
#include <cstdint>

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>

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
