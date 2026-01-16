#include <thread>

#include <ros/ros.h>
#include <ros/package.h>

#include <ars430_process/RadarDetection.h>
#include <ars430_process/RadarPacket.h>

#include <ars430_process/SensorStatus.h>

#include "sniffer.h"
#include "data_struct.h"

#include "config/config_manager.h"
#include "config/runtime_config.h"

using namespace jojo::drivers;

class RadarProcessNode {
 public:
  RadarProcessNode();
  ~RadarProcessNode();

  bool init(ros::NodeHandle& nh, ros::NodeHandle& private_nh,
            std::shared_ptr<RuntimeConfig> param);
  void run();

 protected:
  Sniffer sniffer;

  void Reset();

  std::shared_ptr<RuntimeConfig> param_;
  void receiveThread();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // ros::Publisher objects_pub;
  // ros::Publisher detections_pub;
  // ros::Publisher status_pub;

  std::thread rec_thread;
};