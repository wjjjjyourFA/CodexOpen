#ifndef __CONFIG_H
#define __CONFIG_H

#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <semaphore.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

using namespace std;

enum class ImuType { UNKNOWN = 0, MID360 };

enum class CameraType { UNKNOWN = 0, AR0231, AR0147, INFRA };

enum class RadarType { UNKNOWN = 0, ESR, ARS408 };

enum class Radar4dType { UNKNOWN = 0, ARS548, ARBE };

enum class LidarType { UNKNOWN = 0, RS128, M1P, MID360 };

class SensorRegistry {
 public:
  static const SensorRegistry& Instance();

  ImuType GetImuType(const std::string& name) const;
  CameraType GetCameraType(const std::string& name) const;
  RadarType GetRadarType(const std::string& name) const;
  Radar4dType GetRadar4dType(const std::string& name) const;
  LidarType GetLidarType(const std::string& name) const;

  // LidarCode
  // M1P = 16 * 35 for ros2
  int GetDifopNum(LidarType type) const;

 private:
  SensorRegistry();  // 构造时完成注册

 private:
  std::unordered_map<std::string, ImuType> imu_map_;
  std::unordered_map<std::string, CameraType> camera_map_;
  std::unordered_map<std::string, RadarType> radar_map_;
  std::unordered_map<std::string, Radar4dType> radar4d_map_;
  std::unordered_map<std::string, LidarType> lidar_map_;
};

#endif  // __CONFIG_H
