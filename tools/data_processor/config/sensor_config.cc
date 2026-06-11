#include "tools/data_processor/config/sensor_config.h"

const SensorRegistry& SensorRegistry::Instance() {
  static SensorRegistry instance;
  return instance;
}

SensorRegistry::SensorRegistry() {
  imu_map_ = {{"mid360", ImuType::MID360}};

  camera_map_ = {{"ar0231", CameraType::AR0231},
                 {"ar0147", CameraType::AR0147},
                 {"infra", CameraType::INFRA}

  };

  radar_map_ = {{"esr", RadarType::ESR}, {"ars408", RadarType::ARS408}};

  radar4d_map_ = {{"ars548", Radar4dType::ARS548}, {"arbe", Radar4dType::ARBE}};

  lidar_map_ = {{"m1p", LidarType::M1P},
                {"rs128", LidarType::RS128},
                {"mid360", LidarType::MID360}};
}

ImuType SensorRegistry::GetImuType(const std::string& name) const {
  auto it = imu_map_.find(name);
  if (it != imu_map_.end()) {
    return it->second;
  }
  return ImuType::UNKNOWN;
}

CameraType SensorRegistry::GetCameraType(const std::string& name) const {
  auto it = camera_map_.find(name);
  if (it != camera_map_.end()) {
    return it->second;
  }
  return CameraType::UNKNOWN;
}

RadarType SensorRegistry::GetRadarType(const std::string& name) const {
  auto it = radar_map_.find(name);
  if (it != radar_map_.end()) {
    return it->second;
  }
  return RadarType::UNKNOWN;
}

Radar4dType SensorRegistry::GetRadar4dType(const std::string& name) const {
  auto it = radar4d_map_.find(name);
  if (it != radar4d_map_.end()) {
    return it->second;
  }
  return Radar4dType::UNKNOWN;
}

LidarType SensorRegistry::GetLidarType(const std::string& name) const {
  auto it = lidar_map_.find(name);
  if (it != lidar_map_.end()) {
    return it->second;
  }
  return LidarType::UNKNOWN;
}

int SensorRegistry::GetDifopNum(LidarType type) const {
  switch (type) {
    case LidarType::M1P:
      return 16 * 35;
    case LidarType::RS128:
      return 2;
    default:
      return 0;
  }
}