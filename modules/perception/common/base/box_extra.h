#ifndef BOX_EXTRA_H_
#define BOX_EXTRA_H_
//          0 -------- 1
//          |          |
//          .          .
//          |          |
//          3 -------- 2
// p0: left-top  左上角
// p1: right-top
// p2: right-bottom  右下角
// p3: left-bottom

#pragma once

#include <algorithm>
#include <sstream>
#include <string>

#include "modules/perception/common/base/box.h"
#include "modules/perception/common/base/comparison_traits.h"
#include "modules/perception/common/base/point.h"

namespace jojo {
namespace perception {
namespace base {

struct BBox2DExtra {
  apollo::perception::base::BBox2DF box;

  // clang-format off
  BBox2DExtra(float x_min, float y_min, float x_max, float y_max)
      : box(x_min, y_min, x_max, y_max),
        corners{Point2DF{x_min, y_min}, Point2DF{x_max, y_min},
                Point2DF{x_max, y_max}, Point2DF{x_min, y_max}},
        width(x_max - x_min),
        height(y_max - y_min) {}

  explicit BBox2DExtra(const apollo::perception::base::RectF& rect)
      : BBox2DExtra(rect.x, rect.y, rect.x + rect.width, rect.y + rect.height) {}

  // explicit BBox2DExtra(float x_in, float y_in, float width_in, float height_in)
  //     : BBox2DExtra(x_in, y_in, x_in + width_in, y_in + height_in) {}
  // clang-format on

  // 严格保持一致的语义顺序
  Point2DF corners[4];

  // 基于左上角和宽高的表示
  float width;
  float height;
};

inline apollo::perception::base::BBox2DF xywh2box(float x /*center_x*/,
                                                  float y /*center_y*/,
                                                  float width, float height) {
  apollo::perception::base::BBox2DF bbox;
  bbox.xmin = x - width * 0.5f;
  bbox.ymin = y - height * 0.5f;
  bbox.xmax = x + width * 0.5f;
  bbox.ymax = y + height * 0.5f;
  return bbox;
}

inline apollo::perception::base::BBox2DF tlwh2box(float x /*minx*/,
                                                  float y /*miny*/, float width,
                                                  float height) {
  apollo::perception::base::BBox2DF bbox;
  bbox.xmin = x;
  bbox.ymin = y;
  bbox.xmax = x + width;
  bbox.ymax = y + height;
  return bbox;
}

}  // namespace base
}  // namespace perception
}  // namespace jojo

#endif  // BOX_EXTRA_H_