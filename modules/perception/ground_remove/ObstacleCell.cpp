#include "modules/perception/ground_remove/ObstacleCell.h"

void ObstacleCell::Reset() {
  type = ObstacleType::UNKNOWN;

  count   = 0;
  b_valid = false;
}

void ObstacleCell::UpdateCellByType(ObstacleType t, float z) {
  if (!b_valid || z > max_z) {
    max_z = z;
  }

  type = t;
  ++count;
  b_valid = true;
}