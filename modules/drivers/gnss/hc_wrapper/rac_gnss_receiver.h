#ifndef RAC_GNSS_RECEIVER_H
#define RAC_GNSS_RECEIVER_H

#include <chrono>
#include <thread>

#include <QString>
#include <QStringList>

#include "modules/common/math/math_utils_extra.h"
#include "modules/common/math/unit_converter.h"
#include "modules/common/math/kalman_filter_1d.h"
#include "modules/common_struct/sensor_msgs/GnssData.h"
// #include "modules/localization/common/algorithm/blh2gaussxy.h"

#include "common/serial_interface.h"
// #include "modules/drivers/gnss/hc_wrapper/config/runtime_config.h"

// namespace drivers = jojo::drivers;
namespace math = jojo::common::math;
// namespace algorithm = jojo::localization::algorithm;

class RacGnssReceiver : public SerialInterface {
 public:
  RacGnssReceiver();
  ~RacGnssReceiver();

 public:
  // void SetMsg(localization_msgs::Gps *gps_msg);
  void Resolve(const char* const buf, int n);
  void ResetMsg();

  void SetGprmcFunc(
      const std::function<void(const GnssData* data)>& gprmc_func);
  void SetGppggaFunc(
      const std::function<void(const GnssData* data)>& gpgga_func);
  void SetGpchcFunc(
      const std::function<void(const GnssData* data)>& gpchc_func);

 private:
  void ResolveGPRMC(const QStringList& cmd_list);
  void ResolveGPGGA(const QStringList& cmd_list);
  void ResolveGPCHC(const QStringList& cmd_list);

  std::function<void(const GnssData* data)> gprmc_func_;
  std::function<void(const GnssData* data)> gpgga_func_;
  std::function<void(const GnssData* data)> gpchc_func_;

 private:
  double rot_z_offset = 0;
  // GaussProjection gp_;

  math::KalmanFilter1D<double> kf_;
  double KalmanFilter(const double& azimuth_deg);
  double last_azimuth_ = 0;
};

#endif
