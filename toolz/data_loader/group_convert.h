#ifndef DATA_LOADER_GROUP_CONVERT_DATASET_H
#define DATA_LOADER_GROUP_CONVERT_DATASET_H

#pragma once

#include "modules/common/transform/geometry/rotation_conversions.h"
#include "tools/data_loader/group_convert.h"
#include "toolz/data_loader/data_loader.h"

namespace jojo {
namespace tools {
using namespace std;

// 一帧数据，包含所有传感器数据
class MeasureGroupDataSet : public MeasureGroupBase {
 public:
  MeasureGroupDataSet()          = default;
  virtual ~MeasureGroupDataSet() = default;

  SensorData<pcl::PointCloud<pcl::PointXYZI>::Ptr> lidar;
  SensorData<jojo::common_struct::SE3Pose> se3_pose;
  // 用于计算其他数据的插值位姿，default vec.size() == 3
  std::deque<jojo::common_struct::SE3Pose> se3_pose_vec;

  // 计算 pose center 用于调整偏移，优化点云对齐
  Eigen::Vector3d pose_center = Eigen::Vector3d::Zero();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class GroupConvertDataSet : public GroupConvert {
 public:
  GroupConvertDataSet();
  virtual ~GroupConvertDataSet();

  bool Init(std::shared_ptr<jojo::tools::RuntimeConfig> param,
            std::shared_ptr<jojo::tools::InterfaceConfig> interface) override;

  void InitGroup() override;

  // std::shared_ptr<const MeasureGroupDataSet> ReadNext();
  std::shared_ptr<const MeasureGroupBase> ReadNext() override;

  bool LoadPose(const std::string& file_path);
  bool LoadPose(const std::string& path, const std::string& data_file);

  // way 1 子类通过基类指针调用
  // std::shared_ptr<DataLoaderDataSet> data_loader;

  // way 2 隐藏父类变量
  // 父类函数用的是旧 group
  // 子类函数用的是新 group
  // 数据完全分裂
  // std::shared_ptr<MeasureGroupDataSet> group;

 private:
  std::shared_ptr<MeasureGroupDataSet> group_ds = nullptr;

  DataContainer<jojo::common_struct::SE3Pose> dc_se3_pose;

 protected:
  // using GroupConvert::GetLidarBase;

  void print_pose_vec(const std::deque<jojo::common_struct::SE3Pose>& pose_vec);
};

}  // namespace tools
}  // namespace jojo

#endif
