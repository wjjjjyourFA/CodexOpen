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

// #include "modules/perception/common/base/comparison_traits.h"
#include "modules/perception/common/base/point.h"

namespace jojo {
namespace perception {
namespace base {

struct BBoxSimple {
  // 2 points 2x4=8
  float box[8] = {0};

  Point2DF p[4];

  // 基于左上角和宽高的表示
  float width;
  float height;

  void set_rect(float x /*min_x*/, float y /*min_y*/, float w, float h) {
    p[0] = {x, y};  // 左上
    p[1] = {x + w, y};  // 右上
    p[2] = {x + w, y + h};  // 右下
    p[3] = {x, y + h};  // 左下

    width  = w;
    height = h;
  }

  void get_vertex() {
    float* box_ptr = box;  // 指向 box[0]
    for (auto& vertex : p) {
      vertex.x = *box_ptr++;
      vertex.y = *box_ptr++;
    }
  };

  void update_box_4points() {
    /* way 1
    for (int i = 0; i < 4; i++) {
      box[i * 2 + 0] = p[i].x;
      box[i * 2 + 1] = p[i].y;
    }
    */
    // way 2
    float* box_ptr = box;
    for (const auto& vertex : p) {
      *box_ptr++ = vertex.x;
      *box_ptr++ = vertex.y;
    }
  };
};

}  // namespace base
}  // namespace perception
}  // namespace jojo

#endif  // BOX_EXTRA_H_