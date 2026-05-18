#include "modules/perception/common/lidar/convert/vd_sort_map.h"

VdSMap64::VdSMap64() {
  for (int i = 0; i < 64; i++) laser_sort_tmp[i] = i;
  for (int i = 0; i < 64; i++) {  // let laser_sort from up to down.
    for (int j = i; j > 0; j--) {
      if (vertical_angle[laser_sort_tmp[j]] <
          vertical_angle[laser_sort_tmp[j - 1]]) {
        float temp            = laser_sort_tmp[j - 1];
        laser_sort_tmp[j - 1] = laser_sort_tmp[j];
        laser_sort_tmp[j]     = temp;
      }
    }
  }
  for (int i = 0; i < 64; ++i) {
    laser_sort[laser_sort_tmp[i]] = i;
  }
}