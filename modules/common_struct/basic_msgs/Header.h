#pragma once

#include <string>

namespace jojo {
namespace common_struct {

struct Header {
  uint64_t timestamp{0};  // 默认 0
  uint32_t seq{0};  // 默认 0

  // frame → 听起来像是一个具体的「坐标系对象」
  // frame_id → 明确表示这是「坐标系的名字/标识符 (identifier)」
  std::string frame_id;  // 默认构造为 ""

  Header() = default;
  Header(uint64_t t, const std::string& f) : timestamp(t), frame_id(f) {}
};

}  // namespace common_struct
}  // namespace jojo