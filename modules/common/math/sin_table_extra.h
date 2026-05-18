#ifndef ANGLE_TABLE_H
#define ANGLE_TABLE_H

#pragma once

namespace jojo {
namespace common {
namespace math {

// ---- 角度正弦余弦表 ----
// 角度从 0 到 180 度，每隔 0.5 度采样一次的正弦值查找表（LUT）
extern const float sin_theta[361];

extern const float cos_theta[361];

}  // namespace math
}  // namespace common
}  // namespace jojo

#endif  //