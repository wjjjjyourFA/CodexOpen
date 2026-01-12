#ifndef COLORS_H
#define COLORS_H

#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <algorithm>

extern const u_int8_t jet_color_map[640][3];
extern const u_int16_t jet_color_idx[640];

extern const u_int8_t spectral_color_map[256][3];

namespace jojo {
namespace perception {
namespace base {

// 如果是 C++17 或以上，用 std::clamp
#if __cplusplus >= 201703L
#define CLAMP std::clamp
#else
// 否则自己实现
template <typename T>
inline T clamp(const T& v, const T& lo, const T& hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}
#define CLAMP clamp
#endif

// v 是一个 0～1 之间的归一化数值（normalized value），用来映射到 colormap（颜色图）的颜色索引。
inline void calc_jet_color(float v, int& r, int& g, int& b) {
  v = std::max(0.f, std::min(1.f, v));
  r = int(255 * CLAMP(1.5f - std::fabs(4 * v - 3), 0.f, 1.f));
  g = int(255 * CLAMP(1.5f - std::fabs(4 * v - 2), 0.f, 1.f));
  b = int(255 * CLAMP(1.5f - std::fabs(4 * v - 1), 0.f, 1.f));
}

}  // namespace base
}  // namespace perception
}  // namespace jojo

#endif  // COLORS_HPP
