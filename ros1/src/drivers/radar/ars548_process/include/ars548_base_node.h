#include <thread>
#include <iomanip>

#include <ros/ros.h>
#include <ros/package.h>

#include <ars548_process/Object.h>
#include <ars548_process/ObjectList.h>
#include <ars548_process/Detection.h>
#include <ars548_process/DetectionList.h>

#include <ars548_process/SensorConfiguration.h>
#include <ars548_process/SensorStatus.h>

#include <ars548_process/AccelerationLateralCog.h>
#include <ars548_process/AccelerationLongitudinalCog.h>
#include <ars548_process/CharacteristicSpeed.h>
#include <ars548_process/DrivingDirection.h>
#include <ars548_process/SteeringAngleFrontAxle.h>
#include <ars548_process/VelocityVehicle.h>
#include <ars548_process/YawRate.h>

#include "data_struct.h"
#include "data_process.h"
#include "convert_type.h"
#include "udp_interface.h"

#include "config/config_manager.h"
#include "config/runtime_config.h"

using namespace jojo::drivers;

class RadarBaseNode {
 public:
  RadarBaseNode();
  virtual ~RadarBaseNode();

  bool Init(ros::NodeHandle& nh, std::shared_ptr<RuntimeConfig> param);
  void SetUdpIo(std::shared_ptr<UdpInterface> udp_io);

  void Run();

  void publishObjectList(const RadarObjectList& list);
  void publishDetectionList(const RadarDetectionList& list);
  void publishRadarStatus(const RadarStatus& status);

  RadarObjectList object_list;
  RadarDetectionList detection_list;
  RadarStatus radar_status;

 protected:
  ConvertType cvt;
  char send_data[1024];

  // only set radar config
  std::shared_ptr<UdpInterface> udp_io_;
  // 第二种发送方式，在底层是线程安全的，因此可以共享 udp_io;

  // clang-format off
  void AccLateralCogCallBack(const ars548_process::AccelerationLateralCog& msg);
  void AccLongitudinalCogCallBack(const ars548_process::AccelerationLongitudinalCog& msg);
  void CharacteristicSpeedCallBack(const ars548_process::CharacteristicSpeed& msg);
  void DrivingDirectionCallBack(const ars548_process::DrivingDirection& msg);
  void SteeringAngleCallBack(const ars548_process::SteeringAngleFrontAxle& msg);
  void VelocityVehicleCallBack(const ars548_process::VelocityVehicle& msg);
  void YawRateCallBack(const ars548_process::YawRate& msg);
  // clang-format on

  std::shared_ptr<RuntimeConfig> param_;

  void Reset();

 private:
  ros::NodeHandle node;

  // clang-format off
  ros::Publisher objects_pub;
  ros::Publisher detections_pub;
  ros::Publisher status_pub;

  ros::Subscriber acc_lateral_cog_sub;
  ros::Subscriber acc_longitudinal_cog_sub;
  ros::Subscriber characteristic_speed_sub;
  ros::Subscriber driving_direction_sub;
  ros::Subscriber steering_angle_sub;
  ros::Subscriber velocity_vehicle_sub;
  ros::Subscriber yaw_rate_sub;
  // clang-format on
};