#ifndef ROS1_CONVERT_H
#define ROS1_CONVERT_H

#include <chrono>
#include <thread>

#include <ros/ros.h>

#include "custom_sensor_msgs/Gnss.h"
#include "custom_sensor_msgs/Imu.h"
#include "monitor_msgs/HeartBeat.h"

#include "modules/drivers/gnss/hc_wrapper/rac_gnss_receiver.h"
#include "modules/drivers/gnss/hc_wrapper/config/runtime_config.h"
#include "modules/drivers/gnss/hc_wrapper/config/config_manager.h"

namespace drivers = jojo::drivers;
namespace math = jojo::common::math;

class DrvierWrapper {
 public:
  DrvierWrapper() {};
  ~DrvierWrapper() {};

  int index = -1;
  std::shared_ptr<RacGnssReceiver> gnss_device;
  ros::Publisher gnss_puber;
  ros::Publisher imu_puber;
  ros::Publisher hb_puber;  // heartbeat
  drivers::RuntimeConfig conf;

  custom_sensor_msgs::Gnss gnss_msg;
  custom_sensor_msgs::Imu imu_msg;
  monitor_msgs::HeartBeat hb_msg;

  void GprmcConvert(const GnssData *data);
  void GpggaConvert(const GnssData *data);
  void GpchcConvert(const GnssData *data);
};

class Ros1Convert {
 public:
  Ros1Convert();
  ~Ros1Convert();

  bool Init(ros::NodeHandle &nh, ros::NodeHandle &private_nh,
            std::shared_ptr<drivers::ConfigManager> param);
  void Run();

 protected:
  void SingleChannel(int index);
  void MultiChannel();

 private:
  std::shared_ptr<drivers::ConfigManager> param_;

  ros::NodeHandle nh_;
  std::string ns;

  std::vector<DrvierWrapper> driver_vector;

  std::vector<std::thread> threads_;  // 可选管理线程
};

#endif
