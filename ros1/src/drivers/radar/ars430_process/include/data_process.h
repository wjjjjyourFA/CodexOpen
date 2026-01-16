#ifndef DATA_PROCESS_H
#define DATA_PROCESS_H

#include <math.h>

#include <ros/ros.h>

#include <ars430_process/RadarDetection.h>
#include <ars430_process/RadarPacket.h>

#include <ars430_process/SensorStatus.h>

#include "common.h"
#include "data_struct.h"
#include "resMultipliers.h"

#include "config/runtime_config.h"

using namespace jojo::drivers;

#define DETECTIONS_IN_ELEMENT_BYTE_POS 55
#define PARSE_SS 1

//Resolution Multipliers, make static const global since never changing
static const resMultipliersRDI_t resRDIMults;
static const resMultipliersSS_t  resSSMults;

class DataProcess {
 public:
  DataProcess();
  ~DataProcess();

  void init(ros::NodeHandle& nh, ros::NodeHandle& private_nh,
            std::shared_ptr<RuntimeConfig> param);

  uint8_t parse_packet(udphdr_t* udphdr,
                       const unsigned char* packetptr);  // Parser in parser.cpp

  void publishRDIPacket(RDIPacket_t* packet);

  void publishSSPacket(SSPacket_t* packet);

  void loadPacketMsg(RDIPacket_t* packet,
                     ars430_process::RadarPacket* msg);

  bool loadSSMessage(SSPacket_t* packet,
                     ars430_process::SensorStatus* msg);

 protected:
  std::shared_ptr<RuntimeConfig> param_;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher unfilteredPublisher;
  ros::Publisher ssPublisher;
};

#endif  // DATA_PROCESS_H