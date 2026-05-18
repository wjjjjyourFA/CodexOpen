#pragma once

namespace fastlio {

inline void HeightToColorRgb(float t,  // ∈ [0,1]
                             uint8_t& r, uint8_t& g, uint8_t& b) {
  t = std::clamp(t, 0.0f, 1.0f);

  if (t < 0.33f) {
    // 蓝 -> 绿
    float k = t / 0.33f;
    r       = 0;
    g       = static_cast<uint8_t>(255 * k);
    b       = static_cast<uint8_t>(255 * (1.0f - k));
  } else if (t < 0.66f) {
    // 绿 -> 黄
    float k = (t - 0.33f) / 0.33f;
    r       = static_cast<uint8_t>(255 * k);
    g       = 255;
    b       = 0;
  } else {
    // 黄 -> 红
    float k = (t - 0.66f) / 0.34f;
    r       = 255;
    g       = static_cast<uint8_t>(255 * (1.0f - k));
    b       = 0;
  }
}

}