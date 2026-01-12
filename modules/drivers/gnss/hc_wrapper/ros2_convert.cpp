#include "modules/drivers/gnss/hc_wrapper/ros2_convert.h"

void DrvierWrapper::GprmcConvert(const GnssData *data) {
  // gnss_msg.header.seq++;
  gnss_msg.header.stamp = node->get_clock()->now();
  gnss_msg.header.frame_id = conf.frame_id;
  gnss_msg.message_num++;

  gnss_msg.position.latitude  = math::UnitConverter::latlon_to_int(data->position.latitude);
  gnss_msg.position.longitude = math::UnitConverter::latlon_to_int(data->position.longitude);
  gnss_msg.position.altitude  = math::UnitConverter::meters_to_cm(data->position.altitude);

  gnss_msg.orientation.azimuth =
      math::UnitConverter::angle_to_millirad(data->orientation.azimuth);

  gnss_puber->publish(gnss_msg);
}

void DrvierWrapper::GpggaConvert(const GnssData *data) {}

void DrvierWrapper::GpchcConvert(const GnssData *data) {
  // gnss_msg.header.seq++;
  gnss_msg.header.stamp = node->get_clock()->now();
  gnss_msg.header.frame_id = conf.frame_id;
  gnss_msg.message_num++;
  gnss_msg.week = data->week;
  gnss_msg.time = data->time;

  gnss_msg.position.latitude  = math::UnitConverter::latlon_to_int(data->position.latitude);
  gnss_msg.position.longitude = math::UnitConverter::latlon_to_int(data->position.longitude);
  gnss_msg.position.altitude  = math::UnitConverter::meters_to_cm(data->position.altitude);

  gnss_msg.velocity.east  = math::UnitConverter::meters_to_cm(data->velocity.east);
  gnss_msg.velocity.north = math::UnitConverter::meters_to_cm(data->velocity.north);
  gnss_msg.velocity.up    = math::UnitConverter::meters_to_cm(data->velocity.up);
  gnss_msg.velocity.speed = math::UnitConverter::meters_to_cm(data->velocity.speed);

  gnss_msg.orientation.azimuth =
      math::UnitConverter::angle_to_millirad(data->orientation.azimuth);
  gnss_msg.orientation.pitch = math::UnitConverter::angle_to_millirad(data->orientation.pitch);
  gnss_msg.orientation.roll  = math::UnitConverter::angle_to_millirad(data->orientation.roll);

  gnss_msg.acc.x  = math::UnitConverter::meters_to_cm(data->acc.x);
  gnss_msg.acc.y  = math::UnitConverter::meters_to_cm(data->acc.y);
  gnss_msg.acc.z  = math::UnitConverter::meters_to_cm(data->acc.z);
  gnss_msg.gyro.x = math::UnitConverter::angle_to_millirad(data->gyro.x);  // 0.001rad/s
  gnss_msg.gyro.y = math::UnitConverter::angle_to_millirad(data->gyro.y);  // 0.001rad/s
  gnss_msg.gyro.z = math::UnitConverter::angle_to_millirad(data->gyro.z);

  gnss_msg.main_satellite_num = data->main_satellite_num;
  gnss_msg.vice_satellite_num = data->vice_satellite_num;
  gnss_msg.status             = data->status;
  gnss_msg.age                = data->age;

  gnss_puber->publish(gnss_msg);

  if (conf.pub_imu) {  
    // imu_msg.header.seq++;
    imu_msg.header.stamp = node->get_clock()->now();
    imu_msg.header.frame_id = conf.frame_id;
    imu_msg.message_num++;

    imu_msg.raw.acc.x = math::UnitConverter::meters_to_cm(data->acc.x);
    imu_msg.raw.acc.y = math::UnitConverter::meters_to_cm(data->acc.y);
    imu_msg.raw.acc.z = math::UnitConverter::meters_to_cm(data->acc.z);
    imu_msg.raw.omega.x =
        math::UnitConverter::angle_to_millirad(data->gyro.x);  // 0.001rad/s
    imu_msg.raw.omega.y =
        math::UnitConverter::angle_to_millirad(data->gyro.y);  // 0.001rad/s
    imu_msg.raw.omega.z =
        math::UnitConverter::angle_to_millirad(data->gyro.z);

    imu_puber->publish(imu_msg);
  }
}

Ros2Convert::Ros2Convert() {}

Ros2Convert::~Ros2Convert() {
  for (auto &t : threads_) {
    if (t.joinable()) t.join();
  }
}

bool Ros2Convert::Init(std::shared_ptr<rclcpp::Node> nh,
                       std::shared_ptr<drivers::ParamManager> param) {
  node  = nh;
  param_ = param;

  ns = param_->GetVehicleName();

  std::vector<SensorConfig> &gnss_configs =
      param_->vehicle_model_config.gnss_configs;

  if (gnss_configs.size() < 1) {
    return false;
  }

  for (auto config : gnss_configs) {
    DrvierWrapper driver;
    driver.node = node;
    driver.conf.vehicle_name = param_->GetVehicleName();
    driver.conf.ReadParam(config.config_file);

    std::string topic = "/" + ns + driver.conf.channel_name;

    driver.gnss_puber =
        node->create_publisher<localization_msgs::msg::Gps>(topic + "/gps_data", 10);

    driver.hb_puber =
        node->create_publisher<monitor_msgs::msg::HeartBeat>(topic + "/hb", 10);

    if (driver.conf.pub_imu) {
      driver.imu_puber =
          node->create_publisher<localization_msgs::msg::Imu>(topic + "/imu_data", 10);
    }

    //
    if (driver.conf.type != "uart") {
      std::cerr << " gnss_drvier only support uart! " << std::endl;
      return false;
    }

    auto gnss_device = std::make_shared<RacGnssReceiver>();
    gnss_device->Init(driver.conf.dev_addr, 400 /*buf_size*/,
                      driver.conf.baudrate);

    driver.gnss_device = gnss_device;
    driver.gnss_device->SetGprmcFunc(std::bind(&DrvierWrapper::GprmcConvert, &driver,
                                               std::placeholders::_1));
    driver.gnss_device->SetGppggaFunc(std::bind(&DrvierWrapper::GpggaConvert, &driver,
                                                std::placeholders::_1));
    driver.gnss_device->SetGpchcFunc(std::bind(&DrvierWrapper::GpchcConvert, &driver,
                                               std::placeholders::_1));

    driver_vector.push_back(driver);
  }

  return true;
}

void Ros2Convert::Run() {
  if (driver_vector.size() == 1) {
    // SingleChannel(0);
    MultiChannel();
  } else if (driver_vector.size() > 1) {
    MultiChannel();
  } else {
    return;
  }
}

void Ros2Convert::SingleChannel(int index) {
  auto driver_i = driver_vector.at(index);

  driver_i.gnss_device->Start();

  // ros::Rate loop_rate(driver_i.conf.frame_rate);

  // while (ros::ok()) {
  //   ros::spinOnce();
  //   loop_rate.sleep();
  // }
}

void Ros2Convert::MultiChannel() {
  for (size_t i = 0; i < driver_vector.size(); ++i) {
    threads_.emplace_back([this, i]() { SingleChannel(i); });
  }

  // 主线程等待退出
  // rclcpp::wait_for_shutdown();
}
