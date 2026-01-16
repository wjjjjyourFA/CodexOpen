#include "rac_gnss_receiver.h"

RacGnssReceiver::RacGnssReceiver() {
  // gauss_proj_.SetCoordinateSystem(0);
}

RacGnssReceiver::~RacGnssReceiver() {}

// void RacGnssReceiver::Resolve(const char * const buf, int n)
void RacGnssReceiver::Resolve(chcnav::string *nmea_msg) {
  // "$GPCHC,0,568.34,0.00,0.00,0.00,0.00,0.00,0.00,0.0000,0.0000,0.0000,0.00000000,0.00000000,0.00,0.000,0.000,0.000,0.000,0,0,00,0,2*53"
  std::vector<std::string> cmd_list;
  std::istringstream iss(nmea_msg->sentence.data());
  std::string token;

  while (std::getline(iss, token, ',')) {
    cmd_list.push_back(token);
  }

  // 打印分割后的字符串
  // for (const auto& cmd : cmd_list) {
  //     std::cout << cmd << std::endl;
  // }

  if (cmd_list.size() > 10 && cmd_list[0] == "$GPRMC") {
    ResolveGPRMC(cmd_list);
  } else if (cmd_list.size() > 10 && cmd_list[0] == "$GPGGA") {
    ResolveGPGGA(cmd_list);
  } else if (cmd_list.size() > 10 && cmd_list[0] == "$GPCHC") {
    ResolveGPCHC(cmd_list);
  }
}

void RacGnssReceiver::ResolveGPRMC(const std::vector<std::string> &cmd_list) {
  printf("GPRMC: no process\n");
}

void RacGnssReceiver::ResolveGPGGA(const std::vector<std::string> &cmd_list) {
  printf("GPGGA: no process\n");
}

/*
 * 系统状态
（低半字节）：
    0 初始化
    1 卫导模式
    2 组合导航模式
    3 纯惯导模式卫星状态
（高半字节）：
    0：不定位不定向；
    1：单点定位定向；
    2：伪距差分定位定向；
    3：组合推算；
    4：RTK 稳定解定位定向；
    5：RTK 浮点解定位定向；
    6：单点定位不定向；
    7：伪距差分定位不定向；
    8：RTK 稳定解定位不定向；
    9:RTK 浮点解定位不定向
*******************************************
  warning
    bit0:1:无 GPS 消息，0：正常
    bit1:1:无车辆消息，0：正常
    bit3:1 陀螺错误，0：正常
    bit4:1 加表错误，0：正常
*/
void RacGnssReceiver::ResolveGPCHC(const std::vector<std::string> &cmd_list) {
  static uint8_t heartbeat_count = 0;

  if (cmd_list.size() == 24) {
    // clang-format off
    GnssData data;
    data.week = std::stoi(cmd_list[1]); // 自 1980-1-6 至当前的星期数
    data.time = std::stod(cmd_list[2]); // 自本周日 0:00:00 至当前的秒数
    float azimuth_origin = std::stod(cmd_list[3]);   // 偏航角（0 至 359.99)
    data.orientation.pitch = std::stod(cmd_list[4]); // 俯仰角（-90 至 90)
    data.orientation.roll = std::stod(cmd_list[5]);  // 横滚角（-180 至 180）
    data.gyro.x = std::stod(cmd_list[6]);  // 陀螺 X 轴
    data.gyro.y = std::stod(cmd_list[7]);  // 陀螺 Y 轴
    data.gyro.z = std::stod(cmd_list[8]);  // 陀螺 Z 轴
    data.acc.x  = std::stod(cmd_list[9]);  // 加表 X 轴
    data.acc.y  = std::stod(cmd_list[10]); // 加表 Y 轴
    data.acc.z  = std::stod(cmd_list[11]); // 加表 Z 轴
    data.position.latitude  = std::stod(cmd_list[12]);  // 纬度（-90 至 90）
    data.position.longitude = std::stod(cmd_list[13]);  // 经度（-180 至 180）
    data.position.altitude  = std::stod(cmd_list[14]);  // 高度，单位（米）
    data.velocity.east      = std::stod(cmd_list[15]);  // 东向速度，单位（米/秒）
    data.velocity.north     = std::stod(cmd_list[16]);  // 北向速度，单位（米/秒）
    data.velocity.up        = std::stod(cmd_list[17]);  // 天向速度，单位（米/秒）
    data.velocity.speed     = std::stod(cmd_list[18]);  // 车辆速度，单位（米/秒）
    data.main_satellite_num = std::stoi(cmd_list[19]);  // 主天线 1 卫星数
    data.vice_satellite_num = std::stoi(cmd_list[20]);  // 副天线 2 卫星数
    data.status             = std::stoi(cmd_list[21]);  // 系统状态
    data.age                = std::stoi(cmd_list[22]);  // 差分延时

    // QStringList warning_and_check_sum = cmd_list[23].split('*');
    std::vector<std::string> warning_and_check_sum;
    std::istringstream iss(cmd_list[23]);
    std::string token;
    while (std::getline(iss, token, '*')) {
      warning_and_check_sum.push_back(token);
    }

    // int warning = 0;
    // std::string check_sum = "";
    if (warning_and_check_sum.size() == 2) {
      data.warning = std::stoi(warning_and_check_sum[0]);
      data.check_sum = warning_and_check_sum[1];
    }
    // clang-format on

    // 将一个原始角度
    // azimuth_origin（以北为0°，顺时针为正）转换为以正东为0°、逆时针为正的角度系统，
    // 并将结果归一化到 [0, 360) 的范围内。
    // data.orientation.azimuth = fmod(-azimuth_origin + 450.0, 360.0);
    data.orientation.azimuth = azimuth_origin;
    data.gyro.z = data.gyro.z + rot_z_offset;

    // 采用融合定位方式，不再在GPS进行处理
    // double gauss[2];
    // gauss_proj_.Forward(lon, lat, &gauss[0], &gauss[1]);

    // clang-format off
    // printf(
    //     "GPCHC[%d] time:%.3lf lon:%.6lf lat:%.6lf gauss x:%d y:%d "
    //     "azimuth:%.1lf\n"
    //     "main_satellite_num:%d status:%d warning:%d\n",
    //     // FLAGS_ID, gps_time, lon, lat, (int)gauss.x, (int)gauss.y, azimuth, 0,
    //     gps_time, lon, lat, (int)gauss[0], (int)gauss[1], azimuth,
    //     main_satellite_num, status, warning);
    // printf("debug\n");

    // std::stringstream stream;
    // stream << std::fixed << std::setprecision(7) << lon << " " << lat << " "
    //        << std::setprecision(1) << (int)gauss.x << " " << (int)gauss.y << " "
    //        << azimuth << " " << v << " " << main_satellite_num << " "
    //        << vice_satellite_num << " " << status << " " << warning << " "
    //        << std::fixed << std::setprecision(3) << gps_time;
    // LOG(INFO) << stream.str();
    // clang-format on

    /* UGV_2024
    self_state::GlobalPose gnss_msg;
    gnss_msg->UTC_time = data.time * 1000;
    gnss_msg->local_time = ros::Time::now().toSec();
    gnss_msg->latitude = data.position.latitude;
    gnss_msg->longitude = data.position.longitude;
    gnss_msg->gaussX = (int)gauss[0];  // (int)gauss.x;
    gnss_msg->gaussY = (int)gauss[1];  // (int)gauss.y;
    gnss_msg->azimuth = data.orientation.azimuth;
    gnss_msg->pitch = data.orientation.pitch;
    gnss_msg->roll = data.orientation.roll;
    gnss_msg->height = data.position.altitude;
    gnss_msg->vNorth = data.velocity.north;
    gnss_msg->vEast = data.velocity.east;
    gnss_msg->pos_type.pos_type = data.status;
    gnss_msg->reserved[0] = data.warning;
    // gnss_msg.number_of_sattelite = data.main_satellite_num;
    gnss_msg->reserved[1] = data.vice_satellite_num;
    */

    gnss_msg->header.seq++;
    gnss_msg->header.stamp = ros::Time::now();
    gnss_msg->message_num++;
    gnss_msg->week = data.week;
    gnss_msg->time = data.time;
    // gnss_msg->gauss_x = int32_t(gauss[0] * 100);
    // gnss_msg->gauss_y = int32_t(gauss[1] * 100);

    // 纬度 经度
    gnss_msg->position.latitude  = math::UnitConverter::latlon_to_int(data.position.latitude);
    gnss_msg->position.longitude = math::UnitConverter::latlon_to_int(data.position.longitude);
    gnss_msg->position.altitude  = math::UnitConverter::meters_to_cm(data.position.altitude);
  
    gnss_msg->velocity.east  = math::UnitConverter::meters_to_cm(data.velocity.east);
    gnss_msg->velocity.north = math::UnitConverter::meters_to_cm(data.velocity.north);
    gnss_msg->velocity.up    = math::UnitConverter::meters_to_cm(data.velocity.up);
    gnss_msg->velocity.speed = math::UnitConverter::meters_to_cm(data.velocity.speed);

    gnss_msg->orientation.azimuth =
        math::UnitConverter::angle_to_millirad(data.orientation.azimuth);
    gnss_msg->orientation.pitch = math::UnitConverter::angle_to_millirad(data.orientation.pitch);
    gnss_msg->orientation.roll  = math::UnitConverter::angle_to_millirad(data.orientation.roll);

    gnss_msg->acc.x  = math::UnitConverter::meters_to_cm(data.acc.x);
    gnss_msg->acc.y  = math::UnitConverter::meters_to_cm(data.acc.y);
    gnss_msg->acc.z  = math::UnitConverter::meters_to_cm(data.acc.z);
    gnss_msg->gyro.x = math::UnitConverter::angle_to_millirad(data.gyro.x);  // 0.001rad/s
    gnss_msg->gyro.y = math::UnitConverter::angle_to_millirad(data.gyro.y);  // 0.001rad/s
    gnss_msg->gyro.z = math::UnitConverter::angle_to_millirad(data.gyro.z);

    gnss_msg->main_satellite_num = data.main_satellite_num;
    gnss_msg->vice_satellite_num = data.vice_satellite_num;
    gnss_msg->status             = data.status;
    gnss_msg->age                = data.age;
  }
}

void RacGnssReceiver::SetMsg(custom_sensor_msgs::Gnss *msg) {
  this->gnss_msg = msg;
}

void RacGnssReceiver::GetMsg(custom_sensor_msgs::Gnss *msg) {
  // 将类成员变量的值复制到传入的指针所指向的对象中
  if (msg && gnss_msg) {
    *msg = *(this->gnss_msg);  // 拷贝内容
  }

  // 只是修改了传入指针的值，而没有修改指针指向的内存区域
  // 可能导致竞争
  // 没有将数据传递出去
  // msg = &gnss_msg;
}

void RacGnssReceiver::ResetMsg() {
  // memset(&gps_msg_, 0, sizeof(custom_sensor_msgs::Gnss));
  if (!gnss_msg) {
    // 使用默认构造函数重置内容
    *gnss_msg = custom_sensor_msgs::Gnss();
  }
}
