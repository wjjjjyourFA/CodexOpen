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

#ifndef CYBER_COMMON_TYPES_H_
#define CYBER_COMMON_TYPES_H_

#include <cstdint>

namespace apollo {
namespace cyber {

class NullType {};

// Return code definition for cyber internal function return.
enum ReturnCode {
  SUCC = 0,
  FAIL = 1,
};

/**
 * @brief Describe relation between nodes, writers/readers...
 */
enum Relation : std::uint8_t {
  NO_RELATION = 0,
  DIFF_HOST,  // different host
  DIFF_PROC,  // same host, but different process
  SAME_PROC,  // same process
};

static const char SRV_CHANNEL_REQ_SUFFIX[] = "__SRV__REQUEST";
static const char SRV_CHANNEL_RES_SUFFIX[] = "__SRV__RESPONSE";

}  // namespace cyber
}  // namespace apollo

namespace jojo {
namespace cyber {

/**
   * @brief From UGV CommonDefinitionX.hh...
   */
// 64-bit types
typedef signed long INT64;
typedef unsigned long UINT64;

// 32-bit types
typedef signed int INT32;
typedef unsigned int UINT32;

// 16-bit types
typedef signed short INT16;
typedef unsigned short UINT16;

// 8-bit types
typedef signed char INT8;
typedef unsigned char UINT8;

// #define PI 3.1415926535897932384626433832795
constexpr double PI = 3.1415926535897932384626433832795;

/**
   * @brief Describe relation between nodes, ros1/ros2/dds...
   */
enum NodeType { ROS1_NODE = 0, ROS2_NODE, DDS_NET_NODE, DDS_ZCC_NODE };

}  // namespace cyber
}  // namespace jojo

#endif  // CYBER_COMMON_TYPES_H_
