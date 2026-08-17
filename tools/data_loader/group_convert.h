#ifndef DATA_LOADER_GROUP_CONVERT_H
#define DATA_LOADER_GROUP_CONVERT_H

#pragma once

#include <math.h>

#include "cyber/common/file.h"
#include "modules/common/math/unit_converter.h"
#include "modules/perception/tools/pcl/point_types.h"
#include "tools/data_loader/data_loader.h"
#include "tools/data_processor/config/sensor_config.h"

namespace jojo {
namespace tools {
using namespace std;

template <typename DataT>
struct SensorData {
  using TimeType = uint64_t;

  // frame end_time
  TimeType time = 0;
  DataT data;

  TimeType start_time = 0;
};

template <typename DataT>
struct SensorDataQueue {
  using TimeType = uint64_t;

  std::vector<TimeType> time_vec;
  std::vector<DataT> data_vec;
};

// 一帧数据，包含所有传感器数据
class MeasureGroupBase {
 public:
  MeasureGroupBase()          = default;
  virtual ~MeasureGroupBase() = default;

  std::vector<SensorData<cv::Mat>> camera;
  std::vector<SensorData<cv::Mat>> infra;
  std::vector<SensorData<cv::Mat>> star;
  SensorData<PointCloudXYZI::Ptr> radar;
  std::vector<SensorData<PointCloudXYZI::Ptr>> radar4d;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class MeasureGroup : public MeasureGroupBase {
 public:
  MeasureGroup()          = default;
  virtual ~MeasureGroup() = default;

  // SensorData<PointCloudXYZI::Ptr> lidar;
  // only for struct PointCloud
  SensorData<pcl::PointCloud<pcl::PointXYZIRT>::Ptr> lidar;
  SensorData<jojo::common_struct::ImuData> imu;
  std::deque<jojo::common_struct::ImuData> imu_vec;
  SensorData<jojo::common_struct::GnssData> gnss;
  SensorData<jojo::common_struct::OdomData> odom;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class GroupConvert {
 public:
  GroupConvert();
  virtual ~GroupConvert();

  virtual bool Init(std::shared_ptr<jojo::tools::RuntimeConfig> param,
                    std::shared_ptr<jojo::tools::InterfaceConfig> interface);

  virtual bool InitGroup();

  virtual std::shared_ptr<const MeasureGroupBase> ReadNext();

  bool LoadGlobalPose(const std::string& file_path);
  bool LoadGlobalPose(const std::string& path, const std::string& data_file);

  bool LoadLocalPose(const std::string& file_path);
  bool LoadLocalPose(const std::string& path, const std::string& data_file);

  bool LoadImuData(const std::string& file_path);
  bool LoadImuData(const std::string& path, const std::string& data_file);

  bool IsEnd() { return !is_running_; }

  std::shared_ptr<DataLoader> data_loader;

 protected:
  std::shared_ptr<MeasureGroupBase> group;  // 一帧数据
  // for IsEnd() default need to be true
  bool is_running_ = true;
  bool started_    = false;

  uint64_t index_ts = 0;

  std::shared_ptr<jojo::tools::RuntimeConfig> rparam_;
  std::shared_ptr<jojo::tools::InterfaceConfig> iparam_;

  std::vector<DataContainer<uint64_t>> dc_camera;
  std::vector<DataContainer<uint64_t>> dc_infra;
  std::vector<DataContainer<uint64_t>> dc_star;

  DataContainer<uint64_t> dc_radar;
  std::vector<DataContainer<uint64_t>> dc_radar4d;
  DataContainer<uint64_t> dc_lidar;

 private:
  std::shared_ptr<MeasureGroup> group_ds = nullptr;

  DataContainer<jojo::common_struct::OdomData> dc_local_pose;
  DataContainer<jojo::common_struct::GnssData> dc_global_pose;
  DataContainer<jojo::common_struct::ImuData> dc_imu_data;

 protected:
  uint64_t last_pcd_time_ = 0;
  // bool GetLidarBase(DataContainer<uint64_t>& data_c,
  //                   pcl::PointCloud<pcl::PointXYZI>::Ptr cur_cloud_ptr,
  //                   uint64_t& time, uint64_t& start_time);
  template <typename PointT>
  bool GetLidarBase(DataContainer<uint64_t>& data_c,
                    typename pcl::PointCloud<PointT>::Ptr& cur_cloud_ptr,
                    uint64_t& time, uint64_t& start_time);

  template <typename PointT>
  bool GetLidarBase(DataContainer<uint64_t>& data_c,
                    typename pcl::PointCloud<PointT>::Ptr& cur_cloud_ptr,
                    uint64_t& time, uint64_t& start_time,
                    const std::string& file);
  // void GetLidar();

  bool GetImageBase(DataContainer<uint64_t>& data_c, cv::Mat& cur_image,
                    uint64_t& time, int id, int mode);
  void GetImage(cv::Mat& cur_image, uint64_t& time, int id, int mode);

  template <typename T>
  bool GetDataBase(DataContainerBase* tmp, T& data_out, uint64_t& time) {
    if (tmp->is_end()) {
      tmp->stop();
      return false;
    }

    tmp->GetCurData(&data_out);

    tmp->GetCurTime(time);
    tmp->next();

    return true;
  }

  void print_imu_vec(const std::deque<jojo::common_struct::ImuData>& imu_vec);
};

template <typename PointT>
bool GroupConvert::GetLidarBase(DataContainer<uint64_t>& data_c,
                                typename pcl::PointCloud<PointT>::Ptr& cloud,
                                uint64_t& time, uint64_t& start_time) {
  auto tmp = &data_c;

  if (tmp->is_end()) {
    tmp->stop();
    return false;
  }

  char file[300];

  if (!rparam_->use_bin_or_pcd) {
    sprintf(file, "%s/%.13ld.bin", data_loader->path_lidar.c_str(),
            tmp->cur_time);
  } else {
    sprintf(file, "%s/%.13ld.pcd", data_loader->path_lidar.c_str(),
            tmp->cur_time);
  }

  return GetLidarBase<PointT>(data_c, cloud, time, start_time,
                              std::string(file));
}

template <typename PointT>
bool GroupConvert::GetLidarBase(
    DataContainer<uint64_t>& data_c,
    typename pcl::PointCloud<PointT>::Ptr& cur_cloud_ptr, uint64_t& time,
    uint64_t& start_time, const std::string& file) {
  auto tmp = &data_c;

  if (tmp->is_end()) {
    tmp->stop();
    return false;
  }

  if (!rparam_->use_bin_or_pcd) {
    // clang-format off
    if (!apollo::cyber::common::FileExists(file)) {
      std::cerr << "[ERROR] Failed to load file: " << file << std::endl;
      return false;
    }
    // clang-format on

    // 读取 转为 .pcl
    auto& cloud = cur_cloud_ptr;
    cloud->clear();
    // cloud->points.reserve(100000);
    cloud->is_dense = false;

    int tmp_point[4];
    FILE* fp = fopen(file.c_str(), "rb");
    if (!fp) {
      std::cerr << "Failed to open file: " << file << std::endl;
      return false;
    }
    while (fread(tmp_point, sizeof(int), 4, fp) == 4) {
      PointT pt;
      // cm => m
      pt.x = tmp_point[0] / 100.0f;
      pt.y = tmp_point[1] / 100.0f;
      pt.z = tmp_point[2] / 100.0f;
      // 自动转 float
      pt.intensity = static_cast<float>(tmp_point[3]);

      cloud->points.emplace_back(pt);
    }
    fclose(fp);

    cloud->width  = cloud->points.size();
    cloud->height = 1;
  } else {
    int ret = pcl::io::loadPCDFile<PointT>(std::string(file), *cur_cloud_ptr);
    // int ret = pcl::io::loadPCDFile<pcl::PointXYZI>(std::string(file), *cur_cloud_ptr);
    if (ret == -1) {
      std::cerr << "[Warning] Failed to load PCD file: " << time
                << ", skipping frame." << std::endl;
    }

    /* debug 雷达-IMU 非正装
    Eigen::Matrix4d lidar_ext = Eigen::Matrix4d::Identity();
    lidar_ext << 1.0, 0.0, 0.0, 0.0,
                 0.0, 0.6428, -0.7660, 0.0,
                 0.0, 0.7660, 0.6428, 0.0,
                 0.0, 0.0, 0.0, 1.0;
    pcl::transformPointCloud(*cur_cloud_ptr, *cur_cloud_ptr, lidar_ext);
    */

    // warning
    // pcl::PointCloud<pcl::PointXYZI> cloud;
    // pcl::fromPCLPointCloud2(*cur_cloud_ptr, cloud);
    // pcl::transformPointCloud(cloud, cloud, transform_mat);
    // pcl::toPCLPointCloud2(cloud, *cur_cloud_ptr);
    // toc(t);
  }

  // show_pointcloud_height(*cur_cloud_ptr, 1);
  // show_pointcloud_num(*cur_cloud_ptr, 1800);
  // show2d_lidar_data_normal<PointT>(cur_cloud_ptr);

  time = tmp->cur_time;
  // std::cout << tmp->cur_time << " writing LidarData!" << std::endl;
  start_time = last_pcd_time_;
  if (start_time == 0) {
    start_time = time - 100;
  }
  last_pcd_time_ = time;

  tmp->next();

  return true;
}

}  // namespace tools
}  // namespace jojo

#endif
