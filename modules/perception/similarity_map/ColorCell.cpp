#include "modules/perception/similarity_map/ColorCell.h"

void ColorCell::Reset() {
  rgb[0] = 0;
  rgb[1] = 0;
  rgb[2] = 0;

  sum_[0] = 0;
  sum_[1] = 0;
  sum_[2] = 0;

  max_z = -FLT_MAX;

  count   = 0;
  b_valid = false;
}

void ColorCell::UpdateCellByMaxZ(const cv::Vec3b& color, float z) {
  if (!b_valid || z > max_z) {
    rgb[0] = color[2];
    rgb[1] = color[1];
    rgb[2] = color[0];

    max_z = z;
  }

  ++count;
  b_valid = true;
}

void ColorCell::UpdateCellByMaxZ(const pcl::PointXYZRGB& p) {
  if (!b_valid || p.z > max_z) {
    rgb[0] = p.r;
    rgb[1] = p.g;
    rgb[2] = p.b;

    max_z = p.z;
  }

  ++count;
  b_valid = true;
}

void ColorCell::UpdateCellByAvg(const pcl::PointXYZRGB& p) {
  sum_[0] += p.r;
  sum_[1] += p.g;
  sum_[2] += p.b;

  ++count;

  rgb[0] = static_cast<uint8_t>(sum_[0] / count);
  rgb[1] = static_cast<uint8_t>(sum_[1] / count);
  rgb[2] = static_cast<uint8_t>(sum_[2] / count);

  if (!b_valid || p.z > max_z) {
    max_z = p.z;
  }

  b_valid = true;
}
