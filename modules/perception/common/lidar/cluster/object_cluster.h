#ifndef __CLUSTER_FH_H_
#define __CLUSTER_FH_H_

#pragma once

#include "opencv2/opencv.hpp"

#include "modules/perception/common/base/segment.h"
#include "modules/perception/common/algorithm/point_cloud_processing/cluster_postprocess.h"
#include "modules/perception/common/algorithm/point_cloud_processing/kdtree/kdtree.h"

// PointXYZI;
// PointXYZV;

namespace jojo {
namespace perception {
namespace lidar {

struct ObjectClusterHyperparams {
  // normal lidar distance threshold
  uint far  = 50;
  uint mid  = 30;
  uint near = 10;

  // radius 固定倍数
  float far_r_sacle = 1.0f;
  float mid_r_sacle = 1.5f;

  // radius 线性系数
  float r_sacle = 0.037f;

  // filter eps scale
  float far_e_scale  = 0.25f;
  float mid_e_scale  = 0.5f;
  float near_e_scale = 1.0f;
};

// DBSCAN || RegionGrowing
class ObjectCluster {
 public:
  ObjectCluster();
  ~ObjectCluster();

  void init3d(int width, int height, int length);
  void init2d(int width, int height);
  bool isInited() const { return initialized_; };
  void set_params(float eps, int minPts);

  void SetInputCloud(float* cloud, unsigned char* intensity, int number);
  void SetInputCloud(float* cloud, float* intensity, int number);
  void SetInputCloud(float* cloud, int number);
  void SetInputCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr);
  void SetInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);

  void Run(int mode);
  virtual bool GetOutputSegs(
      std::vector<std::shared_ptr<jojo::perception::base::Segment>>& SegVector);

  void OnceRegionGrowing(int& ind /*index*/, std::vector<int>& n_ind);
  void RegionGrowing(int& ind /*index*/, std::vector<int>& n_ind);
  void MaxRegionGrowing(int& ind /*index*/, std::vector<int>& n_ind);

  void MergeClusters(int id_a, int id_b);

  void DBSCAN();
  void FilterClusters();

  void ClusterPolicy(std::shared_ptr<jojo::perception::base::Segment>& result,
                     uint mode = 1);

 protected:
  float radius_ = 0.5;  // --> eps
  int minPts_   = 5;
  std::vector<int> labels_;

  // clang-format off
  void UpdateSegment(int& ind, int& c_id, jojo::perception::base::Segment& segment);
  std::vector<jojo::perception::base::Segment> ClusterVector;
  std::vector<std::shared_ptr<jojo::perception::base::Segment>> OutputObjectVector;
  // clang-format on

 private:
  int cluster_id_ = 0;  // 数值代表分出几类点云簇

  // 分配的最大内存
  int map_size       = 0;
  float* max_data_3D = nullptr;
  float* max_data_2D = nullptr;
  // unsigned char* data_intensity = nullptr;
  float* data_intensity = nullptr;
  // float *data_velocity; // 预留速度信息
  int data_num;  // --> real data size

  // jojo::perception::algorithm::KDTree* max_kdtree = nullptr;
  std::shared_ptr<jojo::perception::algorithm::KDTree> max_kdtree = nullptr;

  uint data_type_ = 1;  // 0: 2D, 1: 3D
  float distance(const std::vector<float>& a);
  float GetRadius(float& distance);

  std::atomic_bool initialized_{false};

  ObjectClusterHyperparams hps_;
};

}  // namespace lidar
}  // namespace perception
}  // namespace jojo

#endif  // __CLUSTER_FH_H_
