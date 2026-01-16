#include "radar_assist_node.h"

void RadarAssistNode::publishAccLateralCog() {
  ars548_process::AccelerationLateralCog msg;

  msg.header.frame_id = param_->frame_id;
  msg.header.stamp = ros::Time::now();

  msg.acceleration_lateral_err_amp = 0;
  msg.acceleration_lateral_err_amp_invalid_flag = 0;
  msg.qualifier_acceleration_lateral = 0;
  msg.acceleration_lateral = 0;
  msg.acceleration_lateral_invalid_flag = 0;
  msg.acceleration_lateral_event_data_qualifier = 0;

  acc_lateral_cog_pub.publish(msg);
}

void RadarAssistNode::publishAccLongitudinalCog() {
  ars548_process::AccelerationLongitudinalCog msg;

  msg.header.frame_id = param_->frame_id;
  msg.header.stamp = ros::Time::now();

  msg.acceleration_longitudinal_err_amp = 0;
  msg.acceleration_longitudinal_err_amp_invalid_flag = 0;
  msg.qualifier_acceleration_longitudinal = 0;
  msg.acceleration_longitudinal = 0;
  msg.acceleration_longitudinal_invalid_flag = 0;
  msg.acceleration_longitudinal_event_data_qualifier = 0;

  acc_longitudinal_cog_pub.publish(msg);
}

void RadarAssistNode::publishCharacteristicSpeed() {
  ars548_process::CharacteristicSpeed msg;

  msg.header.frame_id = param_->frame_id;
  msg.header.stamp = ros::Time::now();

  msg.characteristic_speed_err_amp = 0;
  msg.qualifier_characteristic_speed = 0;
  // 本车特征速度 km/h
  msg.characteristic_speed = 60;

  characteristic_speed_pub.publish(msg);
}

void RadarAssistNode::publishDrivingDirection() {
  ars548_process::DrivingDirection msg;

  msg.header.frame_id = param_->frame_id;
  msg.header.stamp = ros::Time::now();

  msg.driving_direction_unconfirmed = 0;
  // 车辆行驶方向
  // 0: Standstill
  // 1: Forward
  // 2: Backwards
  msg.driving_direction_confirmed = 1;

  driving_direction_pub.publish(msg);
}

void RadarAssistNode::publishSteeringAngle() {
  ars548_process::SteeringAngleFrontAxle msg;

  msg.header.frame_id = param_->frame_id;
  msg.header.stamp = ros::Time::now();

  msg.qualifier_steering_angle_front_axle = 0;
  msg.steering_angle_front_axle_err_amp = 0;
  msg.steering_angle_front_axle_err_amp_invalid_flag = 0;
  msg.steering_angle_front_axle = 0;
  msg.steering_angle_front_axle_invalid_flag = 0;
  msg.steering_angle_front_axle_event_data_qualifier = 0;

  steering_angle_pub.publish(msg);
}

void RadarAssistNode::publishVelocityVehicle() {
  ars548_process::VelocityVehicle msg;

  msg.header.frame_id = param_->frame_id;
  msg.header.stamp = ros::Time::now();

  msg.status_velocity_near_standstill = 0;
  msg.qualifier_velocity_vehicle = 0;
  msg.velocity_vehicle_event_data_qualifier = 0;
  msg.velocity_vehicle = 0;
  msg.velocity_vehicle_invalid_flag = 0;

  velocity_vehicle_pub.publish(msg);
}

void RadarAssistNode::publishYawRate() {
  ars548_process::YawRate msg;

  msg.header.frame_id = param_->frame_id;
  msg.header.stamp = ros::Time::now();

  msg.yaw_rate_err_amp = 0;
  msg.yaw_rate_err_amp_invalid_flag = 0;
  msg.qualifier_yaw_rate = 0;
  msg.yaw_rate = 0;
  msg.yaw_rate_invalid_flag = 0;
  msg.yaw_rate_event_data_qualifier = 0;

  yaw_rate_pub.publish(msg);
}

bool RadarAssistNode::init(ros::NodeHandle& nh, ros::NodeHandle& private_nh,
                           std::shared_ptr<RuntimeConfig> param) {
  nh_ = nh;
  private_nh_ = private_nh;
  param_ = param;

  std::string ns = param_->GetVehicleName();
  std::string topic;
  /*
  if(param_->index == 0){
    topic = "/" + ns + param_->channel_name;
  }else{
    topic = "/" + ns + param_->channel_name + "_" + std::to_string(param_->index);
  }
  */
  topic = "/" + ns + param_->channel_name;
  
  // clang-format off
  acc_lateral_cog_pub = nh_.advertise<ars548_process::AccelerationLateralCog>(topic + "/acc_lateral_cog", 10);
  acc_longitudinal_cog_pub = nh_.advertise<ars548_process::AccelerationLongitudinalCog>(topic + "/acc_longitudinal_cog", 10);
  characteristic_speed_pub = nh_.advertise<ars548_process::CharacteristicSpeed>(topic + "/characteristic_speed", 10);
  driving_direction_pub = nh_.advertise<ars548_process::DrivingDirection>(topic + "/driving_direction", 10);
  steering_angle_pub = nh_.advertise<ars548_process::SteeringAngleFrontAxle>(topic + "/steering_angle", 10);
  velocity_vehicle_pub = nh_.advertise<ars548_process::VelocityVehicle>(topic + "/velocity_vehicle", 10);
  yaw_rate_pub = nh_.advertise<ars548_process::YawRate>(topic + "/yaw_rate", 10);
  // clang-format on

  return true;
}

void RadarAssistNode::run() {
  ros::Rate r(10);

  while (ros::ok()) {
    publishAccLateralCog();
    publishAccLongitudinalCog();
    publishCharacteristicSpeed();
    publishDrivingDirection();
    publishSteeringAngle();
    publishVelocityVehicle();
    publishYawRate();

    r.sleep();
  }
}

void SingleChannel(ros::NodeHandle& nh, ros::NodeHandle& private_nh,
                   std::shared_ptr<RuntimeConfig> param) {
  RadarAssistNode radar_assist_node;
  radar_assist_node.init(nh, private_nh, param);
  radar_assist_node.run();
}

int main(int argc, char** argv) {
  std::string base_path = ros::package::getPath("ars548_process");
  std::string config_file_path_ = base_path + "/../../../../../install/common/vehicle_sensor_config.yaml";

  auto param_manager = std::make_shared<ConfigManager>();
  param_manager->LoadConfig(config_file_path_);
  std::string name = param_manager->GetVehicleName() + "_ars548_assist_node";

  ros::init(argc, argv, name);
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  std::vector<SensorConfig>& radar_configs =
      param_manager->vehicle_model_config.radar_configs;

  if (radar_configs.size() < 1) {
    return false;
  }

  // 初始化多个Radar Process
  int num = 1;
  std::vector<std::thread> threads_;
  for (auto config : radar_configs) {
    auto param = std::make_shared<RuntimeConfig>();
    param->vehicle_name = param_manager->GetVehicleName();
    param->LoadConfig(config.config_file);
    // std::cout << "vehicle_name: " << param->vehicle_name << std::endl;

    // 复制当前轮的 num，确保 lambda 捕获的是正确值
    int current_num = num;
    // 启动 单线程/多线程
    threads_.emplace_back([current_num, &nh, &private_nh, param]() {
      SingleChannel(nh, private_nh, param);
    });

    num++;
  }

  // 主线程等待退出
  ros::waitForShutdown();
  
  return 0;
}