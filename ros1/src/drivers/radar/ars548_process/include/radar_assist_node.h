#include <thread>

#include <ros/ros.h>
#include <ros/package.h>

#include <ars548_process/AccelerationLateralCog.h>
#include <ars548_process/AccelerationLongitudinalCog.h>
#include <ars548_process/CharacteristicSpeed.h>
#include <ars548_process/DrivingDirection.h>
#include <ars548_process/SteeringAngleFrontAxle.h>
#include <ars548_process/VelocityVehicle.h>
#include <ars548_process/YawRate.h>

#include "config/config_manager.h"
#include "config/runtime_config.h"

using namespace jojo::drivers;

// 协助雷达实现某种高级功能，例如融合定位参考
// 不是配置雷达的基础IP通信地址等
class RadarAssistNode {
 public:
  RadarAssistNode() {};
  ~RadarAssistNode() {};

  bool init(ros::NodeHandle& nh, ros::NodeHandle& private_nh,
            std::shared_ptr<RuntimeConfig> param);
  void run();

  // 这些消息是用于radar的实时自我校正使用
  void publishAccLateralCog();
  void publishAccLongitudinalCog();
  void publishCharacteristicSpeed();
  void publishDrivingDirection();
  void publishSteeringAngle();
  void publishVelocityVehicle();
  void publishYawRate();

 protected:
  std::shared_ptr<RuntimeConfig> param_;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher acc_lateral_cog_pub;
  ros::Publisher acc_longitudinal_cog_pub;
  ros::Publisher characteristic_speed_pub;
  ros::Publisher driving_direction_pub;
  ros::Publisher steering_angle_pub;
  ros::Publisher velocity_vehicle_pub;
  ros::Publisher yaw_rate_pub;
};
