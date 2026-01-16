#include "modules/drivers/gnss/hc_wrapper/mfgi_receiver.h"

namespace drivers = jojo::drivers;
namespace math = jojo::common::math;

MfgiReceiver::MfgiReceiver() {
  SetResolveFunc(std::bind(&MfgiReceiver::Resolve, this, std::placeholders::_1,
                           std::placeholders::_2));
}

MfgiReceiver::~MfgiReceiver() {}

void MfgiReceiver::InitRos(ros::NodeHandle& nh, ros::NodeHandle& private_nh,
                           std::shared_ptr<drivers::RuntimeConfig> param) {
  nh_         = nh;
  private_nh_ = private_nh;
  param_      = param;

  std::string ns    = param_->GetVehicleName();
  std::string topic = "/" + ns + param_->channel_name;

  gps_puber = nh_.advertise<localization_msgs::Gps>(topic + "/gps_data", 1000);
  imu_puber = nh_.advertise<localization_msgs::Imu>(topic + "/imu_data", 10);

  gps_msg.header.frame_id = param->frame_id;
  gps_msg.header.seq      = 0;
  gps_msg.header.stamp    = ros::Time::now();

  imu_msg.header.frame_id = param->frame_id;
  imu_msg.header.seq      = 0;
  imu_msg.header.stamp    = ros::Time::now();
}

void MfgiReceiver::Resolve(const char* const buf, int n) {
  QString qstr_cmd = QString(QLatin1String(buf, n));
  printf("MfgiReceiver: %s\n", qstr_cmd.toStdString().c_str());

  QStringList cmd_list = qstr_cmd.split(',');

  if (cmd_list.size() > 13 && cmd_list[0] == "$AIPOV") {
    // double heading = cmd_list[2].toDouble();  // z
    // double roll    = cmd_list[3].toDouble();  // y
    // double pitch   = cmd_list[4].toDouble();  // x

    // double rot_roll    = cmd_list[5].toDouble();
    // double rot_pitch   = cmd_list[6].toDouble();
    // double rot_heading = cmd_list[7].toDouble();

    // double acc_roll    = cmd_list[8].toDouble();
    // double acc_pitch   = cmd_list[9].toDouble();
    // double acc_heading = cmd_list[10].toDouble();

    double lat = cmd_list[11].toDouble();
    double lon = cmd_list[12].toDouble();

    gps_msg.header.seq++;
    gps_msg.header.stamp = ros::Time::now();
    imu_msg.rot_y        = cmd_list[5].toDouble();
    imu_msg.rot_x        = cmd_list[6].toDouble();
    imu_msg.rot_z        = cmd_list[7].toDouble();
    imu_msg.raw.acc_y    = cmd_list[8].toDouble();
    imu_msg.raw.acc_x    = cmd_list[9].toDouble();
    imu_msg.raw.acc_z    = cmd_list[10].toDouble();
    imu_puber.publish(imu_msg);

    gps_msg.header.seq++;
    gps_msg.header.stamp = ros::Time::now();
    gps_msg.time         = ros::Time::now().toSec();
    gps_msg.position.lat = UnitConverter::latlon_to_int(lat);
    gps_msg.position.lon = UnitConverter::latlon_to_int(lon);
    gps_puber.publish(gps_msg);

    printf("MfgiReceiver lon:%lf lat:%lf x:%.1lf y:%.1lf\n", lon, lat, gauss.x,
           gauss.y);

    if (param_->save) {
      // log_file_ptr_->SaveLocation(lon, lat, gauss.x, gauss.y);
    }
  }
}
