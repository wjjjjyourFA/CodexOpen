#pragma once

#include <deque>
#include <memory>

#include <pcl/common/transforms.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "modules/perception/lidar_local_mapping/common.h"

namespace jojo {
namespace perception {

template <typename PointT>
struct KeyFrame {
  using Ptr = std::shared_ptr<KeyFrame<PointT>>;

  // 关键帧用于添加的时候，默认是true的，只有在更新删除的时候，才会设置为false
  bool valid = true;

  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

  typename pcl::PointCloud<PointT>::Ptr cloud;

  KeyFrame() { cloud.reset(new pcl::PointCloud<PointT>); }
};

struct LocalMappingHyperparams {
  std::size_t max_keyframes = 50;

  // 单位 m
  float keyframe_dist_thresh = 2.0f;

  // 过滤器参数
  float cur_voxel_size = 0.2f;
  float map_voxel_size = 0.1f;
};

template <typename PointT>
class LocalMappingBase {
 public:
  using CloudType = pcl::PointCloud<PointT>;
  using CloudPtr  = typename CloudType::Ptr;

  LocalMappingBase() {};
  virtual ~LocalMappingBase() {};

  virtual void Init();

  void Configure(const LocalMappingHyperparams& hyperparams,
                 std::size_t local_map_reserve, std::size_t scratch_reserve);

  std::size_t KeyframeCount() const noexcept { return keyframes_.size(); }

  CloudPtr GetLocalMapSnapshot() const {
    CloudPtr snapshot(new CloudType);
    if (local_map_) *snapshot = *local_map_;
    return snapshot;
  }

  // !! 注意指针使用，此处禁止共享给外部
  // CloudPtr GetLocalMap() { return this->local_map_; }

 protected:
  bool NeedNewKeyFrame(const Eigen::Matrix4f& pose);

  void AddKeyFrame(const CloudPtr& cloud, const Eigen::Matrix4f& pose);

  virtual void BuildCurrentFrameCloud(const CloudPtr& frame, CloudPtr& out);

  void UpdateLocalMap();

 protected:
  std::deque<typename KeyFrame<PointT>::Ptr> keyframes_;
  // std::deque<std::shared_ptr<KeyFrame<PointT>>> keyframes_;
  CloudPtr local_map_;

  bool initialized_ = false;

 protected:
  LocalMappingHyperparams hps_;

  pcl::ApproximateVoxelGrid<PointT> voxel_;

  // 缓存变量
  CloudPtr transformed_cloud_;
  CloudPtr filtered_cloud_;
  Eigen::Matrix4f last_key_pose_ = Eigen::Matrix4f::Identity();

  // 预分配大小
  size_t local_map_reserve_size  = 500000;
  size_t temp_cloud_reserve_size = 100000;
};

template <typename PointT>
void LocalMappingBase<PointT>::Init() {
  local_map_.reset(new CloudType);
  local_map_->reserve(local_map_reserve_size);

  transformed_cloud_.reset(new CloudType);
  transformed_cloud_->reserve(temp_cloud_reserve_size);

  filtered_cloud_.reset(new CloudType);
  filtered_cloud_->reserve(temp_cloud_reserve_size);
}

template <typename PointT>
void LocalMappingBase<PointT>::Configure(
    const LocalMappingHyperparams& hyperparams, std::size_t local_map_reserve,
    std::size_t scratch_reserve) {
  hps_ = hyperparams;

  local_map_reserve_size  = local_map_reserve;
  temp_cloud_reserve_size = scratch_reserve;
  keyframes_.clear();
  initialized_ = false;
  last_key_pose_.setIdentity();

  Init();
}

template <typename PointT>
bool LocalMappingBase<PointT>::NeedNewKeyFrame(
    const Eigen::Matrix4f& cur_pose) {
  if (!initialized_) return true;

  return Distance(cur_pose, last_key_pose_) > hps_.keyframe_dist_thresh;
}

template <typename PointT>
void LocalMappingBase<PointT>::AddKeyFrame(const CloudPtr& cloud,
                                           const Eigen::Matrix4f& pose) {
  auto kf  = std::make_shared<KeyFrame<PointT>>();
  kf->pose = pose;
  // Keyframes are snapshots;
  // retaining the caller's mutable cloud would make later frame reuse silently rewrite historical map data.
  if (cloud) {
    // 深拷贝（数据复制）
    // *kf->cloud = *cloud;
    // 不拷贝
    kf->cloud = cloud;
  }
  keyframes_.push_back(kf);

  while (keyframes_.size() > hps_.max_keyframes) {
    keyframes_.pop_front();
  }

  last_key_pose_ = pose;

  initialized_ = true;
}

template <typename PointT>
void LocalMappingBase<PointT>::BuildCurrentFrameCloud(const CloudPtr& frame,
                                                      CloudPtr& out) {
  voxel_.setLeafSize(hps_.cur_voxel_size, hps_.cur_voxel_size,
                     hps_.cur_voxel_size);
  voxel_.setInputCloud(frame);

  out->clear();
  voxel_.filter(*out);
}

template <typename PointT>
void LocalMappingBase<PointT>::UpdateLocalMap() {
  local_map_->clear();

  if (keyframes_.empty()) return;

  auto& current_kf = keyframes_.back();

  Eigen::Matrix4f T_current_inv = current_kf->pose.inverse();

  // TODO：这里是全量的，可以优化成增量
  for (auto& kf : keyframes_) {
    if (!kf->valid) continue;

    Eigen::Matrix4f T = T_current_inv * kf->pose;

    transformed_cloud_->clear();
    pcl::transformPointCloud(*kf->cloud, *transformed_cloud_, T);

    *local_map_ += *transformed_cloud_;
  }

  voxel_.setLeafSize(hps_.map_voxel_size, hps_.map_voxel_size,
                     hps_.map_voxel_size);
  voxel_.setInputCloud(local_map_);

  filtered_cloud_->clear();
  voxel_.filter(*filtered_cloud_);

  /* 指针内容交换，不拷贝；
    注意因为这个方式，那么该 local_map 不能共享给外部使用，否则会导致地图加载错误；
    local_map_      -> 内存空间
    filtered_cloud_ -> scratch buffer
    ==>
    local_map_      -> filtered
    filtered_cloud_ -> 内存空间
  */
  local_map_.swap(filtered_cloud_);
}

}  // namespace perception
}  // namespace jojo
