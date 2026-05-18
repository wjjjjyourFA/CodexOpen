/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef CYBER_BASE_FOR_EACH_H_
#define CYBER_BASE_FOR_EACH_H_

#include <type_traits>

#include "cyber/base/macros.h"

namespace apollo {
namespace cyber {
namespace base {

// 定义了一个类型特征 HasLess，用于检查一个类型是否实现了 < 运算符
DEFINE_TYPE_TRAIT(HasLess, operator<)  // NOLINT

// 主要用于比大小，如果 Value 和 End 都实现了 < 运算符，则直接使用 < 运算符比较
template <class Value, class End>
typename std::enable_if<HasLess<Value>::value && HasLess<End>::value,
                        bool>::type
LessThan(const Value& val, const End& end) {
  return val < end;
}

// 如果 Value 和 End 有一个没有实现 < 运算符，则使用 != 运算符比较
// 如果 val 不等于 end，它会返回 true，如果 val 等于 end，它会返回 false
template <class Value, class End>
typename std::enable_if<!HasLess<Value>::value || !HasLess<End>::value,
                        bool>::type
LessThan(const Value& val, const End& end) {
  return val != end;
}

#define FOR_EACH(i, begin, end)           \
  for (auto i = (true ? (begin) : (end)); \
       apollo::cyber::base::LessThan(i, (end)); ++i)

}  // namespace base
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_BASE_FOR_EACH_H_
