#pragma once

#include <vector>
#include <cfloat>
#include <algorithm>

namespace jojo {
namespace perception {

// 用于绑定 z 值和距离
struct ZPoint {
  uint32_t indices;  // 点云中点的索引
  float z;  // height
  float r;  // 该点对应的距离
};

// 2d grid
struct GridElement {
  // 不存储点云数据，只存储栅格数据和点云索引 <== frame；
  // pcl::PointCloud<pcl::PointXYZI>::Ptr points;
  // “统计压缩模型（min/max/mean）”，升级到 “分布模型（percentile / ground probability）”
  std::vector<ZPoint> z_points;

  // 栅格状态
  bool active;
  bool is_ground;

  // 统计量
  ZPoint max_z;
  ZPoint min_z;
  float mean_z;
  float delta_z;
  float std_z;  // 标准差
  float sum_z;
  float sum_z2;

  // 代表点
  ZPoint rep_z;

  GridElement() {
    // 每个格子默认存储64个点
    z_points.reserve(32);
    Reset();
  }

  inline void Reset() {
    z_points.clear();

    active    = false;
    is_ground = false;

    max_z   = {0u, -FLT_MAX, -FLT_MAX};
    min_z   = {0u, FLT_MAX, FLT_MAX};
    mean_z  = 0;
    delta_z = 0;
    std_z   = 0;
    sum_z   = 0.f;
    sum_z2  = 0.f;

    rep_z = {0u, 0.f, 0.f};
  }

  inline void Reserve(size_t n) { z_points.reserve(n); }

  inline void AddPoint(uint32_t idx, float z, float z_range = -1.f) {
    // 通常一个 bin 里：10~30个点，保留最近64个点即可
    // if (z_points.size() < 64) {
    z_points.push_back({idx, z, z_range});

    sum_z += z;
    // sum_z2 += z * z;

    if (z > max_z.z) {
      max_z.z = z;
      max_z.r = z_range;
    }

    if (z < min_z.z) {
      min_z.z = z;
      min_z.r = z_range;
    }
    // }
  }

  inline void ComputeStatistic() {
    if (z_points.empty()) return;

    mean_z  = sum_z / z_points.size();
    delta_z = max_z.z - min_z.z;

    // float variance = sum_z2 / z_points.size() - mean_z * mean_z;
    // std_z = std::sqrt(std::max(variance, 0.f));
  }

  inline void ComputePercentile(float p = 0.2f) {
    // O(n) 平均复杂度的快速选择算法
    const int n = static_cast<int>(z_points.size());

    if (n == 0) {
      rep_z = {0u, 0.f, 0.f};
      return;
    }

    // 点太少时直接使用最低点
    if (n < 5) {
      rep_z = min_z;
      return;
    }

    // C17
    // const int idx = std::clamp(static_cast<int>(n * p), 0, n - 1);
    const int idx = std::max(0, std::min(static_cast<int>(n * p), n - 1));

    std::nth_element(
        z_points.begin(), z_points.begin() + idx, z_points.end(),
        [](const ZPoint& a, const ZPoint& b) { return a.z < b.z; });

    rep_z = z_points[idx];
  }
};

}  // namespace perception
}  // namespace jojo
