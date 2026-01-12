#include "modules/perception/common/lidar/convert/rs_sort_map.h"

RsSMap128::RsSMap128() {
  for (int i = 0; i < 128; i++) laser_sort_tmp[i] = i;
  for (int i = 0; i < 128; i++) {  // let laser_sort from up to down.
    for (int j = i; j > 0; j--) {
      // clang-format off
      if (vertical_angle[laser_sort_tmp[j]] < vertical_angle[laser_sort_tmp[j - 1]]) {
        float temp            = laser_sort_tmp[j - 1];
        laser_sort_tmp[j - 1] = laser_sort_tmp[j];
        laser_sort_tmp[j]     = temp;
      }
      // clang-format on
    }
  }
  for (int i = 0; i < 128; ++i) {
    laser_sort[laser_sort_tmp[i]] = i;
  }
}

void RsSMap128::Init() {
  for (int i = 0; i < 128; ++i) {
    m_hSinTable[i] = sin(horizontal_angle[i] * M_PI / 180.0);
    m_hCosTable[i] = cos(horizontal_angle[i] * M_PI / 180.0);
    m_vSinTable[i] = sin(vertical_angle[i] * M_PI / 180.0);
    m_vCosTable[i] = cos(vertical_angle[i] * M_PI / 180.0);
  }
}

RsSMap64::RsSMap64() {}

void RsSMap64::Init() {
  for (int i = 0; i < 64; i++) {
    m_hSinTable[i] = sin(horizontal_angle[i] * M_PI / 180.0);
    m_hCosTable[i] = cos(horizontal_angle[i] * M_PI / 180.0);
    m_vSinTable[i] = sin(vertical_angle[i] * M_PI / 180.0);
    m_vCosTable[i] = cos(vertical_angle[i] * M_PI / 180.0);
  }
}