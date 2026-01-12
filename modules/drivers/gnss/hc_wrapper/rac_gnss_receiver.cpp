#include "modules/drivers/gnss/hc_wrapper/rac_gnss_receiver.h"

RacGnssReceiver::RacGnssReceiver() {
  SetResolveFunc(std::bind(&RacGnssReceiver::Resolve, this,
                           std::placeholders::_1, std::placeholders::_2));

  gprmc_func_ = [](const GnssData* data) { printf("undefined gprmc func \n"); };
  gpgga_func_ = [](const GnssData* data) { printf("undefined gpgga func \n"); };
  gpchc_func_ = [](const GnssData* data) { printf("undefined gpchc func \n"); };

  // gp_.SetCoordinateSystem(0);
}

RacGnssReceiver::~RacGnssReceiver() {}

void RacGnssReceiver::Resolve(const char* const buf, int n) {
  static QStringList msg_list;
  // QStringList input_msg_list = QString(QLatin1String(buf)).split("\n\n");
  QStringList input_msg_list = QString::fromUtf8(buf).split("\n\n");

  bool find_tail = false;

  if (input_msg_list.empty()) {
    return;
  }

  if (input_msg_list.back().size() > 2 &&
      input_msg_list.back().at(input_msg_list.back().size() - 1) == '\n' &&
      input_msg_list.back().at(input_msg_list.back().size() - 2) == '\n') {
    find_tail = true;
  }

  // 字符串拼接
  if (msg_list.empty()) {
    msg_list = input_msg_list;
  } else if (n > 1 && buf[0] == '$') {
    msg_list.clear();
    msg_list = input_msg_list;
  } else {
    msg_list.back() += input_msg_list.front();
    input_msg_list.pop_front();
    msg_list += input_msg_list;
  }

  while (!msg_list.empty()) {
    if (msg_list.size() > 1 || (msg_list.size() == 1 && find_tail)) {
      QStringList cmd_list = msg_list.front().split(',');

      if (cmd_list.size() > 10 && cmd_list[0] == "$GPRMC") {
        ResolveGPRMC(cmd_list);
      } else if (cmd_list.size() > 10 && cmd_list[0] == "$GPGGA") {
        ResolveGPGGA(cmd_list);
      } else if (cmd_list.size() > 10 && cmd_list[0] == "$GPCHC") {
        ResolveGPCHC(cmd_list);
      }

      msg_list.pop_front();
    } else {
      break;
    }
  }
}

void RacGnssReceiver::ResolveGPRMC(const QStringList& cmd_list) {
  GnssData data;
  // "度分"格式 ==> 十进制度
  double lat_clock = cmd_list[3].toDouble();
  double lat_min   = fmod(lat_clock, 100);
  double lat_sec   = 0;  // fmod(lat_clock, 1) * 100;
  data.position.latitude =
      floor(lat_clock / 100.0) + (lat_min + (lat_sec / 60.0)) / 60.0;

  double lon_clock = cmd_list[5].toDouble();
  double lon_min   = fmod(lon_clock, 100);
  double lon_sec   = 0;  // fmod(lon_clock, 1) * 100;
  data.position.longitude =
      floor(lon_clock / 100.0) + (lon_min + (lon_sec / 60.0)) / 60.0;

  if (cmd_list[8].size() > 0) {
    double azimuth_origin = cmd_list[8].toDouble();

    // 卡尔曼滤波
    azimuth_origin = this->KalmanFilter(azimuth_origin);

    // 将一个原始角度 azimuth_origin（以北为0°，顺时针为正）转换为以正东为0°、逆时针为正的角度系统，
    // 并将结果归一化到 [0, 360) 的范围内。
    // data.orientation.azimuth = fmod(-azimuth_origin + 450.0, 360.0);
    data.orientation.azimuth = azimuth_origin;
  }

  // printf("RacGnssReceiver lon:%.6lf lat:%.6lf gauss x:%d y:%d azimuth:%.1lf\n",
  //        lon, lat, (int)gauss.x, (int)gauss.y, azimuth);

  // std::stringstream stream;
  // stream << std::fixed << std::setprecision(7) << lon << " " << lat << " "
  //        << std::setprecision(1) << (int)gauss.x << " " << (int)gauss.y << " "
  //        << azimuth;
  // LOG(INFO) << stream.str();

  gprmc_func_(&data);
}

void RacGnssReceiver::ResolveGPGGA(const QStringList& cmd_list) {
  printf("GPGGA lon:%s\n", cmd_list.join(',').toStdString().c_str());

  GnssData data;
  data.position.altitude = std::round(cmd_list[9].toDouble());

  gpgga_func_(&data);
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
void RacGnssReceiver::ResolveGPCHC(const QStringList& cmd_list) {
  if (cmd_list.size() == 24) {
    // clang-format off
    GnssData data;
    data.week = cmd_list[1].toInt();     // 自 1980-1-6 至当前的星期数
    data.time = cmd_list[2].toDouble();  // 自本周日 0:00:00 至当前的秒数
    double azimuth_origin    = cmd_list[3].toDouble();  // 偏航角（0 至 359.99)
    data.orientation.pitch   = cmd_list[4].toDouble();  // 俯仰角（-90 至 90)
    data.orientation.roll    = cmd_list[5].toDouble();  // 横滚角（-180 至 180）
    data.gyro.x = cmd_list[6].toDouble();  // 陀螺 X 轴
    data.gyro.y = cmd_list[7].toDouble();  // 陀螺 Y 轴
    data.gyro.z = cmd_list[8].toDouble();  // 陀螺 Z 轴
    data.acc.x  = cmd_list[9].toDouble();  // 加表 X 轴
    data.acc.y  = cmd_list[10].toDouble(); // 加表 Y 轴
    data.acc.z  = cmd_list[11].toDouble(); // 加表 Z 轴
    data.position.latitude  = cmd_list[12].toDouble();  // 纬度（-90 至 90）
    data.position.longitude = cmd_list[13].toDouble();  // 经度（-180 至 180）
    data.position.altitude  = cmd_list[14].toDouble();  // 高度，单位（米）
    data.velocity.east      = cmd_list[15].toDouble();  // 东向速度，单位（米/秒）
    data.velocity.north     = cmd_list[16].toDouble();  // 北向速度，单位（米/秒）
    data.velocity.up        = cmd_list[17].toDouble();  // 天向速度，单位（米/秒）
    data.velocity.speed     = cmd_list[18].toDouble();  // 车辆速度，单位（米/秒）
    data.main_satellite_num = cmd_list[19].toInt();  // 主天线 1 卫星数
    data.vice_satellite_num = cmd_list[20].toInt();  // 副天线 2 卫星数
    data.status             = cmd_list[21].toInt();  // 系统状态
    data.age                = cmd_list[22].toInt();  // 差分延时

    QStringList warning_and_check_sum = cmd_list[23].split('*');
    if (warning_and_check_sum.size() == 2) {
      data.warning = warning_and_check_sum[0].toInt();
      data.check_sum = warning_and_check_sum[1].toStdString();
    }
    // clang-format on

    // 卡尔曼滤波
    azimuth_origin = this->KalmanFilter(azimuth_origin);

    // 将一个原始角度 azimuth_origin（以北为0°，顺时针为正）转换为以正东为0°、逆时针为正的角度系统，
    // 并将结果归一化到 [0, 360) 的范围内。
    // data.orientation.azimuth = fmod(-azimuth_origin + 450.0, 360.0);
    data.orientation.azimuth = azimuth_origin;
    data.gyro.z              = data.gyro.z + rot_z_offset;

    // 采用融合定位方式，不再在GPS进行处理
    // double gauss[2];
    // gp_.Forward(lon, lat, &gauss[0], &gauss[1]);

    // clang-format off
    // printf(
    //     "GPCHC[%d] time:%.3lf lon:%.6lf lat:%.6lf gauss x:%d y:%d "
    //     "azimuth:%.1lf\n"
    //     "main_satellite_num:%d status:%d warning%d\n",
    //     // FLAGS_ID, gps_time, lon, lat, (int)gauss.x, (int)gauss.y, azimuth,
    //     gps_time, lon, lat, (int)gauss[0], (int)gauss[1], azimuth,
    //     main_satellite_num, status, warning);

    // std::stringstream stream;
    // stream << std::fixed << std::setprecision(7) << lon << " " << lat << " "
    //        << std::setprecision(1) << (int)gauss.x << " " << (int)gauss.y << " "
    //        << azimuth << " " << v << " " << main_satellite_num << " "
    //        << vice_satellite_num << " " << status << " " << warning << " "
    //        << std::fixed << std::setprecision(3) << gps_time;
    // LOG(INFO) << stream.str();
    // clang-format on

    gpchc_func_(&data);
  }
}

/*
void RacGnssReceiver::SetMsg(localization_msgs::Gps *gps_msg) {
  this->gps_msg_ = gps_msg;
}
*/

void RacGnssReceiver::ResetMsg() {
  // memset(&gps_msg_, 0, sizeof(localization_msgs::Gps));
  /*
  if (!gps_msg_) {
    // 使用默认构造函数重置内容
    *gps_msg_ = localization_msgs::Gnss();
  }
  */
}

void RacGnssReceiver::SetGprmcFunc(
    const std::function<void(const GnssData* data)>& gprmc_func) {
  gprmc_func_ = gprmc_func;
}

void RacGnssReceiver::SetGppggaFunc(
    const std::function<void(const GnssData* data)>& gpgga_func) {
  gpgga_func_ = gpgga_func;
}

void RacGnssReceiver::SetGpchcFunc(
    const std::function<void(const GnssData* data)>& gpchc_func) {
  gpchc_func_ = gpchc_func;
}

double RacGnssReceiver::KalmanFilter(const double& azimuth_deg) {
  // deg to rad
  double azimuth = math::UnitConverter::angle_to_rad(azimuth_deg);

  if (!kf_.IsInitialized()) {
    kf_.SetStateEstimate(azimuth);  // 初始状态

    // 存储上一帧角度
    last_azimuth_ = azimuth;

    return azimuth_deg;  // 第一次直接返回原值
  }

  kf_.PredictTheta(last_azimuth_, azimuth);  // 用增量预测当前角度
  kf_.CorrectTheta(azimuth);  // 用当前测量更新状态

  // 更新上一帧状态
  last_azimuth_ = kf_.GetStateEstimate();

  // rad to deg
  return math::UnitConverter::rad_to_angle(azimuth);
}
