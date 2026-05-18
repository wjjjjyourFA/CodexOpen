#ifndef FRAME2D_TRANSFORM_COORDS_HPP
#define FRAME2D_TRANSFORM_COORDS_HPP

#include <cmath>

// 用于帧数据的坐标转换

namespace jojo {
namespace localization {
namespace common {

// unit : m  m/s  rad rad/s
class Frame2dTransform {
 public:
  Frame2dTransform() {
    offset_x_     = 0;
    offset_y_     = 0;
    offset_theta_ = 0;
    sinot_        = sin(offset_theta_);
    cosot_        = cos(offset_theta_);
  }

  // 设置 传感器坐标系 在 车身坐标系 中的位置
  // set sensor_frame in body_frame
  // eg : radar - SetSensorInBody(4, -0.15, -90);
  //      lidar - SetSensorInBody(0.93, 0, -90);
  //      image using lidar range information, so set as liadr too
  void SetSensorInBody(const double& offset_x, const double& offset_y,
                       const double& offset_theta);

  // 设置 车身坐标系 在 全局坐标系 中的位置；
  // 全局坐标系： 各种 odometry；
  // set body_frame in global_frame
  // eg : SetBodyInOdom(100, 200, 150);
  //      fixed current vehicle local_pose here
  void SetBodyInOdom(const double& dr_x, const double& dr_y,
                     const double& dr_theta);

  // 变换 位置 point without angle
  void SensorPose2Body(double sensor_x, double sensor_y, double& body_x,
                       double& body_y) {
    body_x = cosot_ * sensor_x - sinot_ * sensor_y + offset_x_;
    body_y = sinot_ * sensor_x + cosot_ * sensor_y + offset_y_;
  }

  void BodyPose2Sensor(double body_x, double body_y, double& sensor_x,
                       double& sensor_y) {
    sensor_x = cosot_ * (body_x - offset_x_) + sinot_ * (body_y - offset_y_);
    sensor_y = -sinot_ * (body_x - offset_x_) + cosot_ * (body_y - offset_y_);
  }

  void BodyPose2Odom(double body_x, double body_y, double& odom_x,
                     double& odom_y) {
    odom_x = cosdt_ * body_x - sindt_ * body_y + dr_x_;
    odom_y = sindt_ * body_x + cosdt_ * body_y + dr_y_;
  }

  void OdomPose2Body(double odom_x, double odom_y, double& body_x,
                     double& body_y) {
    body_x = cosdt_ * (odom_x - dr_x_) + sindt_ * (odom_y - dr_y_);
    body_y = -sindt_ * (odom_x - dr_x_) + cosdt_ * (odom_y - dr_y_);
  }

  void SensorPose2Odom(double sensor_x, double sensor_y, double& odom_x,
                       double& odom_y) {
    double body_x, body_y;

    SensorPose2Body(sensor_x, sensor_y, body_x, body_y);
    BodyPose2Odom(body_x, body_y, odom_x, odom_y);
  }

  void OdomPose2Sensor(double odom_x, double odom_y, double& sensor_x,
                       double& sensor_y) {
    double body_x, body_y;

    OdomPose2Body(odom_x, odom_y, body_x, body_y);
    BodyPose2Sensor(body_x, body_y, sensor_x, sensor_y);
  }

  // 变换 位置 point with angle ：需要旋转 + 平移 + 角度叠加。
  void SensorPose2Body(double sensor_x, double sensor_y, double sensor_theta,
                       double& body_x, double& body_y, double& body_theta) {
    body_x     = cosot_ * sensor_x - sinot_ * sensor_y + offset_x_;
    body_y     = sinot_ * sensor_x + cosot_ * sensor_y + offset_y_;
    body_theta = sensor_theta + offset_theta_;
  }

  void BodyPose2Sensor(double body_x, double body_y, double body_theta,
                       double& sensor_x, double& sensor_y,
                       double& sensor_theta) {
    sensor_x = cosot_ * (body_x - offset_x_) + sinot_ * (body_y - offset_y_);
    sensor_y = -sinot_ * (body_x - offset_x_) + cosot_ * (body_y - offset_y_);
    sensor_theta = body_theta - offset_theta_;
  }

  void BodyPose2Odom(double body_x, double body_y, double body_theta,
                     double& odom_x, double& odom_y, double& odom_theta) {
    odom_x     = cosdt_ * body_x - sindt_ * body_y + dr_x_;
    odom_y     = sindt_ * body_x + cosdt_ * body_y + dr_y_;
    odom_theta = body_theta + dr_theta_;
  }

  void OdomPose2Body(double odom_x, double odom_y, double odom_theta,
                     double& body_x, double& body_y, double& body_theta) {
    body_x     = cosdt_ * (odom_x - dr_x_) + sindt_ * (odom_y - dr_y_);
    body_y     = -sindt_ * (odom_x - dr_x_) + cosdt_ * (odom_y - dr_y_);
    body_theta = odom_theta - dr_theta_;
  }

  void SensorPose2Odom(double sensor_x, double sensor_y, double sensor_theta,
                       double& odom_x, double& odom_y, double odom_theta) {
    double body_x, body_y, body_theta;

    SensorPose2Body(sensor_x, sensor_y, sensor_theta, body_x, body_y,
                    body_theta);
    BodyPose2Odom(body_x, body_y, body_theta, odom_x, odom_y, odom_theta);
  }
  void OdomPose2Sensor(double odom_x, double odom_y, double odom_theta,
                       double& sensor_x, double& sensor_y,
                       double sensor_theta) {
    double body_x, body_y, body_theta;

    OdomPose2Body(odom_x, odom_y, odom_theta, body_x, body_y, body_theta);
    BodyPose2Sensor(body_x, body_y, body_theta, sensor_x, sensor_y,
                    sensor_theta);
  }

  // 变换 速度 ：只需要旋转，不用平移，也不用加角度偏移。
  void SensorVel2Body(double sensor_vel_x, double sensor_vel_y,
                      double& body_vel_x, double& body_vel_y) {
    body_vel_x = cosot_ * sensor_vel_x - sinot_ * sensor_vel_y;
    body_vel_y = sinot_ * sensor_vel_x + cosot_ * sensor_vel_y;
  }

  void BodyVel2Sensor(double body_vel_x, double body_vel_y,
                      double& sensor_vel_x, double& sensor_vel_y) {
    sensor_vel_x = cosdt_ * body_vel_x + sindt_ * body_vel_y;
    sensor_vel_y = -sindt_ * body_vel_x + cosdt_ * body_vel_y;
  }

  void BodyVel2Odom(double body_vel_x, double body_vel_y, double& odom_vel_x,
                    double& odom_vel_y) {
    odom_vel_x = cosdt_ * body_vel_x - sindt_ * body_vel_y;
    odom_vel_y = sindt_ * body_vel_x + cosdt_ * body_vel_y;
  }

  void OdomVel2Body(double odom_vel_x, double odom_vel_y, double& body_vel_x,
                    double& body_vel_y) {
    body_vel_x = cosdt_ * odom_vel_x + sindt_ * odom_vel_y;
    body_vel_y = -sindt_ * odom_vel_x + cosdt_ * odom_vel_y;
  }

  void SensorVel2Odom(double sensor_vel_x, double sensor_vel_y,
                      double& odom_vel_x, double& odom_vel_y) {
    double body_vel_x, body_vel_y;

    SensorVel2Body(sensor_vel_x, sensor_vel_y, body_vel_x, body_vel_y);
    BodyVel2Odom(body_vel_x, body_vel_y, odom_vel_x, odom_vel_y);
  }

  void OdomVel2Sensor(double odom_vel_x, double odom_vel_y,
                      double& sensor_vel_x, double& sensor_vel_y) {
    double body_vel_x, body_vel_y;

    OdomVel2Body(odom_vel_x, odom_vel_y, body_vel_x, body_vel_y);
    BodyVel2Sensor(body_vel_x, body_vel_y, sensor_vel_x, sensor_vel_y);
  }

 protected:
  // base is body
  double offset_x_, offset_y_, offset_theta_;
  // sin_offset_theta_ \ cos_offset_theta_
  double sinot_, cosot_;

  // base is odom
  double dr_x_, dr_y_, dr_theta_;
  double sindt_, cosdt_;
};

}  // namespace common
}  // namespace localization
}  // namespace jojo

#endif
