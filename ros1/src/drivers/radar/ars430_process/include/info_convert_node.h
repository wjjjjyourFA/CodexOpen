#pragma once

#include <thread>

#include <ros/ros.h>
#include <ros/package.h>

#include <ars430_process/RadarDetection.h>
#include <ars430_process/RadarPacket.h>
#include <ars430_process/SensorStatus.h>

#include "common.h"
#include "data_struct.h"
#include "processPacket.h"

#include "config/config_manager.h"
#include "config/runtime_config.h"

using namespace jojo::drivers;

class PacketProcessor;

class InfoConverNode {
 public:
  InfoConverNode() {};
  ~InfoConverNode() {};

  bool init(ros::NodeHandle& nh, ros::NodeHandle& private_nh,
            std::shared_ptr<RuntimeConfig> param);
  void run();

  void RawDataCallback(const ars430_process::RadarPacket::ConstPtr& msg);

  // Publisher Functions
  void publishRadarPacketMsg(ars430_process::RadarPacket& radar_packet_msg);

  uint8_t pubCallback(PacketGroup_t* Packets);

  std::shared_ptr<PacketProcessor> packet_processor;

 protected:
  std::shared_ptr<RuntimeConfig> param_;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Subscriber UnfilteredSub;

  // Publisher Objects
  ros::Publisher packet_pub_;
  ros::Publisher cloud_pub_;  // Not used now, kept if useful
};