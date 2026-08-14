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
#pragma once

#include <map>
#include <string>

#include <opencv2/highgui.hpp>

namespace jojo {
namespace perception {
namespace base {

enum class ObjectType {
  UNKNOWN           = 0,
  UNKNOWN_MOVABLE   = 1,
  UNKNOWN_UNMOVABLE = 2,

  TREE,
  GRASS,
  ROCK_PILE,  // 石堆

  OILBOX,
  AMMOBOX,
  CHEVALIER,  // 拒马
  TRACK_ABATIS,  // 反履带路障（阻挡履带车辆的障碍，轨条砦）
  RUBBLE,  // 建筑残骸、碎石堆、废墟
  SHELTER,  // 掩体 / 避难所
  BLOCKHOUSE,  // 碉堡

  PERSON_CIVILIAN,
  PERSON_ARMY = 14,
  BICYCLE,
  VEHICLE_GENERAL,
  VEHICLE_ARMY,

  TANK,  // 坦克
  BUSES,  // 公交车 / 巴士
  TRUCK,  // 卡车 / 货车

  BATTLE_EQUIPMENT,  // 军事装备
  BATTLE_DAMAGED_EQUIPMENT,  // 战损装备
  AMPHIBIOUS,  // 两栖车辆 / 两栖装备
  MILITARY_SHIPS,  // 军舰

  MAX_OBJECT_TYPE
};

// @brief semantic object type
enum class ObjectSemanticType {
  UNKNOWN                   = 0,
  IGNORE                    = 1,
  GROUND                    = 2,
  OBJECT                    = 3,
  CURB                      = 4,
  VEGETATION                = 5,
  FENCE                     = 6,
  NOISE                     = 7,
  WALL                      = 8,
  MAX_OBJECT_SEMANTIC_LABEL = 9,
};

// @brief motion state
enum class MotionState {
  UNKNOWN    = 0,
  MOVING     = 1,
  STATIONARY = 2,
  MAX_MOTION_STATE,
};
/**
 * ObjectType mapping
 */
/* 将 enum class 转换为字符串的函数
inline const char* BoxTypetoString(ObjectType obj) {
  switch (obj) {
  case ObjectType::GENERAL: return "GENERAL";
  case ObjectType::OILBOX: return "OILBOX";
}
*/

const std::map<ObjectType, std::string> kObjectType2NameMap = {
    {ObjectType::UNKNOWN, "UNKNOWN"},
    {ObjectType::UNKNOWN_MOVABLE, "UNKNOWN_MOVABLE"},
    {ObjectType::UNKNOWN_UNMOVABLE, "UNKNOWN_UNMOVABLE"},

    {ObjectType::TREE, "TREE"},
    {ObjectType::GRASS, "GRASS"},
    {ObjectType::ROCK_PILE, "ROCK_PILE"},

    {ObjectType::OILBOX, "OILBOX"},
    {ObjectType::AMMOBOX, "AMMOBOX"},
    {ObjectType::CHEVALIER, "CHEVALIER"},
    {ObjectType::TRACK_ABATIS, "TRACK_ABATIS"},
    {ObjectType::RUBBLE, "RUBBLE"},
    {ObjectType::SHELTER, "SHELTER"},
    {ObjectType::BLOCKHOUSE, "BLOCKHOUSE"},

    {ObjectType::PERSON_CIVILIAN, "PERSON_CIVILIAN"},
    {ObjectType::PERSON_ARMY, "PERSON_ARMY"},
    {ObjectType::BICYCLE, "BICYCLE"},
    {ObjectType::VEHICLE_GENERAL, "VEHICLE_GENERAL"},
    {ObjectType::VEHICLE_ARMY, "VEHICLE_ARMY"},

    {ObjectType::TANK, "TANK"},
    {ObjectType::BUSES, "BUSES"},
    {ObjectType::TRUCK, "TRUCK"},

    {ObjectType::BATTLE_EQUIPMENT, "BATTLE_EQUIPMENT"},
    {ObjectType::BATTLE_DAMAGED_EQUIPMENT, "BATTLE_DAMAGED_EQUIPMENT"},
    {ObjectType::AMPHIBIOUS, "AMPHIBIOUS"},
    {ObjectType::MILITARY_SHIPS, "MILITARY_SHIPS"},

    {ObjectType::MAX_OBJECT_TYPE, "MAX_OBJECT_TYPE"}};

const std::map<std::string, ObjectType> kObjectName2TypeMap = {
    {"UNKNOWN", ObjectType::UNKNOWN},
    {"UNKNOWN_MOVABLE", ObjectType::UNKNOWN_MOVABLE},
    {"UNKNOWN_UNMOVABLE", ObjectType::UNKNOWN_UNMOVABLE},

    {"TREE", ObjectType::TREE},
    {"GRASS", ObjectType::GRASS},
    {"ROCK_PILE", ObjectType::ROCK_PILE},

    {"OILBOX", ObjectType::OILBOX},
    {"AMMOBOX", ObjectType::AMMOBOX},
    {"CHEVALIER", ObjectType::CHEVALIER},
    {"TRACK_ABATIS", ObjectType::TRACK_ABATIS},
    {"RUBBLE", ObjectType::RUBBLE},
    {"SHELTER", ObjectType::SHELTER},
    {"BLOCKHOUSE", ObjectType::BLOCKHOUSE},

    {"PERSON_CIVILIAN", ObjectType::PERSON_CIVILIAN},
    {"PERSON_ARMY", ObjectType::PERSON_ARMY},
    {"BICYCLE", ObjectType::BICYCLE},
    {"VEHICLE_GENERAL", ObjectType::VEHICLE_GENERAL},
    {"VEHICLE_ARMY", ObjectType::VEHICLE_ARMY},

    {"TANK", ObjectType::TANK},
    {"BUSES", ObjectType::BUSES},
    {"TRUCK", ObjectType::TRUCK},

    {"BATTLE_EQUIPMENT", ObjectType::BATTLE_EQUIPMENT},
    {"BATTLE_DAMAGED_EQUIPMENT", ObjectType::BATTLE_DAMAGED_EQUIPMENT},
    {"AMPHIBIOUS", ObjectType::AMPHIBIOUS},
    {"MILITARY_SHIPS", ObjectType::MILITARY_SHIPS},

    {"MAX_OBJECT_TYPE", ObjectType::MAX_OBJECT_TYPE}};

inline cv::Scalar BoxTypetoColor(const ObjectType& obj) {
  switch (obj) {
    case ObjectType::PERSON_CIVILIAN:
    case ObjectType::PERSON_ARMY:
      return cv::Scalar(0, 255, 255);  // yellow

    case ObjectType::VEHICLE_GENERAL:
    case ObjectType::VEHICLE_ARMY:
    case ObjectType::BATTLE_EQUIPMENT:
    case ObjectType::BATTLE_DAMAGED_EQUIPMENT:
    case ObjectType::TANK:
      return cv::Scalar(0, 0, 255);  // red

    case ObjectType::MILITARY_SHIPS:
      return cv::Scalar(255, 0, 255);  // pink

    case ObjectType::BLOCKHOUSE:
      return cv::Scalar(0, 255, 0);  // green

    case ObjectType::TRACK_ABATIS:
    case ObjectType::CHEVALIER:
    case ObjectType::SHELTER:
      return cv::Scalar(255, 255, 0);  // cyan

    case ObjectType::AMPHIBIOUS:
      return cv::Scalar(255, 0, 0);  // blue

    default:
      return cv::Scalar(255, 255, 255);  // white for unknown types
  }
}

inline std::string BoxTypetoString(const ObjectType& obj) {
  std::string name = "UNKNOWN";
  // name = kObjectType2NameMap.at(obj);
  auto it = kObjectType2NameMap.find(obj);
  if (it != kObjectType2NameMap.end()) {
    name = it->second;
  }

  // std::cout << "BoxTypetoString: " << name << std::endl;
  return name;
}

inline ObjectType SwitchBoxTypeWrapper(const int& idx) {
  // 这里是 外部 label 到 内部 label 的映射
  // 根据 obj.idx 的值选择相应的 ObjectType 枚举成员
  // 如果 idx 不在映射范围内，则返回 ObjectType::UNKNOWN
  static constexpr ObjectType kTypeMap[] = {
      ObjectType::UNKNOWN,  // 0
      ObjectType::PERSON_CIVILIAN,  // 1
      ObjectType::PERSON_ARMY,  // 2
      ObjectType::VEHICLE_GENERAL,  // 3
      ObjectType::VEHICLE_ARMY,  // 4
      ObjectType::TANK,  // 5
      ObjectType::SHELTER,  // 6
  };
  constexpr int kMapSize = sizeof(kTypeMap) / sizeof(kTypeMap[0]);

  if (idx < 0 || idx >= kMapSize) {
    return ObjectType::UNKNOWN;
  }

  return kTypeMap[idx];
}

inline int BoxTypeToInt(const ObjectType& t) { return static_cast<int>(t); }

}  // namespace base
}  // namespace perception
}  // namespace jojo
