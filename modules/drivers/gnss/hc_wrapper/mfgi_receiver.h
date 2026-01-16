#ifndef MFGI_RECEIVER_H
#define MFGI_RECEIVER_H

#include <QStringList>

#include "ros/ros.h"
#include "ros/init.h"

#include "localization_msgs/Gps.h"
#include "localization_msgs/Imu.h"

#include "modules/common/math/unit_converter.h"
#include "modules/drivers/gnss/hc_wrapper/config/runtime_config.h"
#include "common/tcp_interface.h"

class MfgiReceiver : public TcpClient {
 public:
  MfgiReceiver();
  ~MfgiReceiver();

  bool InitRos(ros::NodeHandle& nh, ros::NodeHandle& private_nh,
               std::shared_ptr<jojo::drivers::RuntimeConfig> param);

 protected:
  void Resolve(const char* const buf, int n);

  std::shared_ptr<jojo::drivers::RuntimeConfig> param_ /*param_manager_*/;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher gps_puber;
  ros::Publisher imu_puber;

  localization_msgs::Gps gps_msg;
  localization_msgs::Imu imu_msg;
};

#endif  // MFGI_RECEIVER_H
