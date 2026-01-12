#ifndef ROS2_CONVERT_H
#define ROS2_CONVERT_H

#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "custom_sensor_msgs/msg/gnss.hpp"
#include "custom_sensor_msgs/msg/imu.hpp"
#include "monitor_msgs/msg/heart_beat.hpp"

#include "modules/drivers/gnss/hc_wrapper/rac_gnss_receiver.h"
#include "modules/drivers/gnss/hc_wrapper/params/params.h"

namespace drivers = jojo::drivers;
namespace math = jojo::common::math;

class DrvierWrapper {
 public:
  DrvierWrapper() {};
  ~DrvierWrapper() {};

  // clang-format off
  int index = -1;
  std::shared_ptr<RacGnssReceiver> gnss_device;
  rclcpp::Publisher<custom_sensor_msgs::msg::Gnss>::SharedPtr gnss_puber;
  rclcpp::Publisher<custom_sensor_msgs::msg::Imu>::SharedPtr imu_puber;
  rclcpp::Publisher<monitor_msgs::msg::HeartBeat>::SharedPtr hb_puber;  // heartbeat
  // clang-format on

  drivers::ParamSimple conf;

  custom_sensor_msgs::msg::Gnss gnss_msg;
  custom_sensor_msgs::msg::Imu imu_msg;
  monitor_msgs::msg::HeartBeat hb_msg;

  void GprmcConvert(const GnssData *data);
  void GpggaConvert(const GnssData *data);
  void GpchcConvert(const GnssData *data);

 public:
  std::shared_ptr<rclcpp::Node> node;
};

class Ros2Convert {
 public:
  Ros2Convert();
  ~Ros2Convert();

  bool Init(std::shared_ptr<rclcpp::Node> nh,
            std::shared_ptr<drivers::ParamManager> param);
  void Run();

 protected:
  void SingleChannel(int index);
  void MultiChannel();

 private:
  std::shared_ptr<drivers::ParamManager> param_;

  std::shared_ptr<rclcpp::Node> node;
  std::string ns;

  std::vector<DrvierWrapper> driver_vector;

  std::vector<std::thread> threads_;  // 可选管理线程
};

#endif
