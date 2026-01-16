#ifndef GROUP_CONVERT_H
#define GROUP_CONVERT_H

#pragma once

#include <math.h>

#include "cyber/common/file.h"
#include "tools/data_processor/config/sensor_config.h"

#include "tools/data_loader/data_loader.h"

namespace jojo {
namespace tools {
using namespace std;

template <typename DataT>
struct SensorData {
  using TimeType = uint64_t;

  TimeType time = 0;
  DataT data;
};

// 一帧数据，包含所有传感器数据
class MeasureGroup {
 public:
  MeasureGroup() {};
  virtual ~MeasureGroup() {};

  SensorData<PointCloudXYZI::Ptr> lidar;
  std::vector<SensorData<cv::Mat>> camera;
  std::vector<SensorData<cv::Mat>> infra;
  std::vector<SensorData<cv::Mat>> star;
  SensorData<PointCloudXYZI::Ptr> radar;
  std::vector<SensorData<PointCloudXYZI::Ptr>> radar_4d;
  SensorData<ImuData> imu;
};

class GroupConvert {
 public:
  GroupConvert();
  virtual ~GroupConvert();

  void Init(std::shared_ptr<RuntimeConfigOffline> param);
  void InitGroup();

  std::shared_ptr<const MeasureGroup> ReadNext();

  bool LoadGlobalPose(const std::string &file_path);
  bool LoadGlobalPose(const std::string &path, const std::string &data_file);

  bool LoadLocalPose(const std::string &file_path);
  bool LoadLocalPose(const std::string &path, const std::string &data_file);

  bool LoadImuData(const std::string &file_path);
  bool LoadImuData(const std::string &path, const std::string &data_file);

  bool IsEnd() { return !is_running_; }

  std::shared_ptr<DataLoader> data_loader;

 protected:
  std::shared_ptr<MeasureGroup> group;  // 一帧数据
  bool is_running_  = true;
  uint64_t index_ts = 0;

 private:
  std::shared_ptr<RuntimeConfigOffline> param_ /*param_simple_base*/;

  DataContainer<OdomData> dc_local_pose;
  DataContainer<GnssData> dc_global_pose;
  DataContainer<ImuData> dc_imu_data;

  std::vector<DataContainer<uint64_t>> dc_camera;
  std::vector<DataContainer<uint64_t>> dc_infra;
  std::vector<DataContainer<uint64_t>> dc_star;

  DataContainer<uint64_t> dc_radar;
  std::vector<DataContainer<uint64_t>> dc_radar_4d;
  DataContainer<uint64_t> dc_lidar;

  void GetDataBase(DataContainerBase *tmp) {};

  bool GetLidarBase(DataContainer<uint64_t> &data_c,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr cur_cloud_ptr,
                    uint64_t &time);
  // void GetLidar();

  bool GetImageBase(DataContainer<uint64_t> &data_c, cv::Mat &cur_image,
                    uint64_t &time, int id, int mode);
  void GetImage(cv::Mat &cur_image, uint64_t &time, int id, int mode);

  template <typename T>
  bool GetDataBase(DataContainerBase *tmp, T &data_out, uint64_t &time) {
    if (tmp->end()) {
      tmp->stop();
      return false;
    }

    tmp->GetCurData(&data_out);

    tmp->GetCurTime(time);
    tmp->next();

    return true;
  }
};

}  // namespace tools
}  // namespace jojo

#endif
