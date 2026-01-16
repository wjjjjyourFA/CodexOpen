/*
 * @Author: HuangWei
 * @Date: 2024-04-09 11:20:06
 * @LastEditors: HuangWei
 * @LastEditTime: 2024-04-09 16:24:05
 * @FilePath: /chcnav_ros_0.48/misc/rac_gnss_receiver.h
 * @Description:
 *
 * Copyright (c) 2024 by JOJO, All Rights Reserved.
 */
#ifndef RAC_GNSS_RECEIVER_H
#define RAC_GNSS_RECEIVER_H

// #include <qstringlist.h>

#include "ros/ros.h"

#include "custom_sensor_msgs/Gnss.h"
#include "custom_sensor_msgs/Imu.h"
#include "monitor_msgs/HeartBeat.h"

#include "chcnav/hc_sentence.h"
#include "chcnav/int8_array.h"
#include "chcnav/string.h"

// #include "modules/common/math/math_utils_extra.h"
#include "modules/common/math/unit_converter.h"
#include "modules/common/math/unit_converter.h"
#include "modules/common_struct/sensor_msgs/GnssData.h"
// #include "modules/localization/common/algorithm/blh2gaussxy.h"

namespace math = jojo::common::math;
// using namespace jojo::localization::algorithm;

class RacGnssReceiver {
 public:
  RacGnssReceiver();
  ~RacGnssReceiver();

 public:
  void SetMsg(custom_sensor_msgs::Gnss *gnss_msg);
  // void Resolve(const char * const buf, int n);
  void Resolve(chcnav::string *nmea_msg);
  void GetMsg(custom_sensor_msgs::Gnss *gnss_msg);
  void ResetMsg();
    
 private:
  void ResolveGPRMC(const std::vector<std::string> &cmd_list);
  void ResolveGPGGA(const std::vector<std::string> &cmd_list);
  void ResolveGPCHC(const std::vector<std::string> &cmd_list);

 private:
  custom_sensor_msgs::Gnss *gnss_msg;

 private:
  double rot_z_offset = 0;

  // GaussProjection gp_;
};

#endif  // RAC_GNSS_RECEIVER_H
