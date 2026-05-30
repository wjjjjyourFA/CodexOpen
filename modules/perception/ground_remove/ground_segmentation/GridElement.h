#pragma once

#include <vector>
#include <cfloat>

namespace jojo {
namespace perception {

// 2d grid
struct GridElement {
  // 不存储点云数据，只存储栅格数据和点云索引 <== frame；
  // pcl::PointCloud<pcl::PointXYZI>::Ptr points;
  std::vector<uint32_t> indices;  // 点云中点的索引
  uint16_t point_num;

  // 栅格状态
  bool active;
  bool is_ground;

  float mean_z;
  float max_z;
  float min_z;
  float delta_z;
  float std_z;  // 标准差
  float sum_z;
  float sum_z2;

  float max_z_distance;
  float min_z_distance;

  GridElement() {
    // 每个格子默认存储64个点
    indices.reserve(32);
    Reset();
  }

  inline void Reset() {
    indices.clear();
    point_num = 0;

    active    = false;
    is_ground = false;

    mean_z  = 0;
    max_z   = -FLT_MAX;
    min_z   = FLT_MAX;
    delta_z = 0;
    std_z   = 0;
    sum_z   = 0.f;
    sum_z2  = 0.f;

    max_z_distance = -FLT_MAX;
    min_z_distance = FLT_MAX;
  }

  inline void Reserve(size_t n) { indices.reserve(n); }

  inline void AddPoint(uint32_t idx, float z, float z_range = -1.f) {
    indices.push_back(idx);
    point_num++;

    sum_z += z;
    // sum_z2 += z * z;

    // clang-format off
    if (z > max_z) {
      max_z = z;
      max_z_distance = z_range;
    }

    if (z < min_z) {
      min_z = z;
      min_z_distance = z_range;
    }
    // clang-format on
  }

  inline void ComputeStatistic() {
    if (point_num == 0) return;

    mean_z  = sum_z / point_num;
    delta_z = max_z - min_z;

    // float variance = sum_z2 / point_num - mean_z * mean_z;
    // std_z = std::sqrt(std::max(variance, 0.f));
  }
};

}  // namespace perception
}  // namespace jojo
