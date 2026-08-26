#include "modules/localization/common/transform/frame2d_transform.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

using jojo::localization::common::Frame2dTransform;

namespace {

constexpr double kPi = 3.14159265358979323846;

void ExpectNear(double actual, double expected, const char* label) {
  constexpr double kTolerance = 1e-12;
  if (!std::isfinite(actual) || std::abs(actual - expected) > kTolerance) {
    std::cerr << label << " mismatch: actual=" << actual
              << ", expected=" << expected << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

}  // namespace

int main() {
  Frame2dTransform transform;

  // A default transform is the identity for every coordinate pair.
  double x = 0.0;
  double y = 0.0;
  transform.BodyPose2Odom(3.0, -4.0, x, y);
  ExpectNear(x, 3.0, "default pose x");
  ExpectNear(y, -4.0, "default pose y");

  transform.SetSensorInBody(1.5, -0.25, kPi / 3.0);
  transform.SetBodyInOdom(10.0, -5.0, -kPi / 4.0);

  double odom_x     = 0.0;
  double odom_y     = 0.0;
  double odom_theta = 0.0;
  transform.SensorPose2Odom(2.0, -1.0, 0.2, odom_x, odom_y, odom_theta);

  double sensor_x     = 0.0;
  double sensor_y     = 0.0;
  double sensor_theta = 0.0;
  transform.OdomPose2Sensor(odom_x, odom_y, odom_theta, sensor_x, sensor_y,
                            sensor_theta);
  ExpectNear(sensor_x, 2.0, "pose round-trip x");
  ExpectNear(sensor_y, -1.0, "pose round-trip y");
  ExpectNear(sensor_theta, 0.2, "pose round-trip theta");

  double body_vx = 0.0;
  double body_vy = 0.0;
  transform.SensorVel2Body(4.0, -2.0, body_vx, body_vy);
  double sensor_vx = 0.0;
  double sensor_vy = 0.0;
  transform.BodyVel2Sensor(body_vx, body_vy, sensor_vx, sensor_vy);
  ExpectNear(sensor_vx, 4.0, "velocity round-trip x");
  ExpectNear(sensor_vy, -2.0, "velocity round-trip y");

  return EXIT_SUCCESS;
}
