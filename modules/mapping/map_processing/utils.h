#pragma once

#include <cmath>
#include <unordered_map>

#include "Eigen/Core"

namespace utils {

#define SMALL_EPS 1e-10
#define HASH_P 116101
#define MAX_N 10000000019

class VOXEL_LOC {
 public:
  int64_t x, y, z;

  VOXEL_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0)
      : x(vx), y(vy), z(vz) {}

  bool operator==(const VOXEL_LOC& other) const {
    return (x == other.x && y == other.y && z == other.z);
  }
};

struct M_POINT {
  float xyz[3] = {0, 0, 0};
  int count    = 0;
};

}  // namespace utils

namespace std {

template <>
struct hash<utils::VOXEL_LOC> {
  size_t operator()(const utils::VOXEL_LOC& s) const {
    using std::size_t;
    using std::hash;
    long index_x, index_y, index_z;
    double cub_len = 1.0 / 8;

    index_x = int(std::round(std::floor((s.x) / cub_len + SMALL_EPS)));
    index_y = int(std::round(std::floor((s.y) / cub_len + SMALL_EPS)));
    index_z = int(std::round(std::floor((s.z) / cub_len + SMALL_EPS)));

    return (((((index_z * HASH_P) % MAX_N + index_y) * HASH_P) % MAX_N) +
            index_x) %
           MAX_N;
  }
};

}  // namespace std