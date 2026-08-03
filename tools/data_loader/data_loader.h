#ifndef DATA_LOADER_H
#define DATA_LOADER_H

#pragma once

#include <algorithm>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#if __cplusplus >= 201703L  // C++17 或更高
#include <filesystem>
namespace fs = std::filesystem;
#else  // C++14 或更低
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;
#endif

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "cyber/common/file.h"
#include "modules/common_struct/localization_msgs/OdometryData.h"
#include "modules/common_struct/sensor_msgs/GnssData.h"
#include "modules/common_struct/sensor_msgs/ImuData.h"
#include "modules/perception/common/camera/common/undistortion_handler.h"
#include "modules/perception/common/camera/params/camera_params.h"
#include "modules/perception/common/radar/convert/ars408.h"
#include "modules/perception/common/radar/convert/ars548.h"
#include "modules/perception/common/radar/convert/hugin.h"
#include "tools/data_loader/config/interface_config.h"
#include "tools/data_loader/config/runtime_config.h"
#include "tools/data_loader/data_container.h"

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZI> PointCloudXYZI;

namespace jojo {
namespace tools {

class DataLoader {
 public:
  DataLoader();
  virtual ~DataLoader();

  // 也许可以，通过指向基类，实现多态。
  virtual bool Init(std::shared_ptr<jojo::tools::RuntimeConfig> param,
                    std::shared_ptr<jojo::tools::InterfaceConfig> interface);
  virtual void InitUndistortion();

  void Start();
  void Stop() {};

  bool LoadTimeStamp(const std::string& path, const std::string& ts_file,
                     DataContainerBase& data_container);

  bool ExtractTimestamp(const std::string& path,
                        DataContainerBase& data_container);

  bool SaveTimeStamp(const std::string& path, const std::string& ts_file,
                     const DataContainerBase& data_container);

  std::string prefix;
  std::string postfix;
  std::string path_global_pose;
  std::string path_local_pose;
  std::string path_imu_data;
  std::string path_lidar;
  std::vector<std::string> path_camera;
  std::vector<std::string> path_infra;
  std::vector<std::string> path_star;
  std::string path_radar;
  std::vector<std::string> path_radar4d;

 protected:
  std::shared_ptr<jojo::tools::RuntimeConfig> rparam_;
  std::shared_ptr<jojo::tools::InterfaceConfig> iparam_;

  std::vector<std::string> SetDataFolderVector(const std::string& prefix,
                                               const std::string& name,
                                               int count);
  virtual void LoadDataFolder();

 private:
  // clang-format off
  std::shared_ptr<jojo::perception::camera::CameraParams> camera_params;
  // std::vector<std::shared_ptr<jojo::perception::camera::CameraParams>> camera_params_vector;
  // clang-format on
 protected:
  // clang-format off
  std::vector<std::shared_ptr<jojo::perception::camera::UndistortionHandler>> undistort_vector;
  std::vector<bool> undistort_init;
  // clang-format on

 private:
  // 简易版本，不使用模板特化等功能
  // std::vector<DataContainer<uint64_t /*sensor_msgs::Image*/>> dc_camera;
  // std::vector<DataContainer<uint64_t /*sensor_msgs::Image*/>> dc_infra;
  // std::vector<DataContainer<uint64_t /*sensor_msgs::Image*/>> dc_star;
};

}  // namespace tools
}  // namespace jojo

#endif  // DATA_LOADER_H
