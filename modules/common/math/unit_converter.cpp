#include "modules/common/math/unit_converter.h"

namespace jojo {
namespace common {
namespace math {

// 初始化静态成员
double UnitConverter::PI        = M_PI;
double UnitConverter::RAW_PI_2  = UnitConverter::PI / 2.0;
double UnitConverter::DEG_TO_RAD = UnitConverter::PI / 180.0;
double UnitConverter::RAD_TO_DEG = 180.0 / UnitConverter::PI;

UnitConverter::UnitConverter() {
  // PI = M_PI;  // π的常数值
  // RAW_PI_2 = PI / 2.0;  // π/2
  // DEG_TO_RAD = PI / 180.0;
  // RAD_TO_DEG = 180.0 / PI;
}

}  // namespace math
}  // namespace common
}  // namespace jojo