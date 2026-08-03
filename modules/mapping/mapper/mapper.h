#ifndef CMapper_H
#define CMapper_H

#include <sys/stat.h>

#include <unordered_set>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "unistd.h"

#define PCL_NO_PRECOMPILE
#include "cyber/common/file.h"
#include "modules/mapping/map_processing/downsampling.h"
#include "modules/mapping/mapper/config/runtime_config.h"
#include "modules/mapping/mapper/config/static_config.h"
#include "opencv2/opencv.hpp"
#include "pcl/visualization/pcl_visualizer.h"

using namespace std;
using namespace cv;

namespace jojo {
namespace mapping {

struct MapperHyperparams {
  // 单位 m
  float sampling_distance = 1.0;
  float map_resolution    = 0.1;

  // rad default
  float sampling_angle_rad = 0.1;
};

class Mapper {
 public:
  Mapper();
  virtual ~Mapper();

  void Init(std::shared_ptr<jojo::mapping::RuntimeConfig> rparam,
            std::shared_ptr<jojo::mapping::StaticConfig> saram);

  void InitViewer();

  void SetPoseCenter(const Eigen::Vector3d& p_center);

  void Run(const pcl::PointCloud<pcl::PointXYZI>::Ptr& frame,
           const Eigen::Matrix4f& in_pose);

  virtual void Reset();

  void RealTimeShow(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_in);
  virtual void VisualizeMap(bool b_pause = true);

  virtual void UpdateIncrementalMap();

  virtual void SaveMap(const std::string& path);

 protected:
  virtual void FlushBufferToMap();

  void CommitRealtimeHistory();
  void RebuildRealtimeHistory(float voxel_size);

  pcl::visualization::PCLVisualizer::Ptr vis_ = NULL;
  bool vis_inited_                            = false;

 protected:
  std::shared_ptr<jojo::mapping::RuntimeConfig> rparam_;
  std::shared_ptr<jojo::mapping::StaticConfig> sparam_;

  MapperHyperparams hps_;

  Eigen::Vector3d pose_center = Eigen::Vector3d::Zero();

 private:
  pcl::PointCloud<pcl::PointXYZI>::Ptr world_point_cloud;
  int world_point_cloud_idx = 0;

  // 缓存
  pcl::PointCloud<pcl::PointXYZI>::Ptr ds_history_point_cloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr ds_world_point_cloud;

  // 实时可视化专用
  pcl::PointCloud<pcl::PointXYZI>::Ptr vis_frame;
  pcl::PointCloud<pcl::PointXYZI>::Ptr vis_recent_cloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr vis_history_cloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr vis_downsampled_frame_;
  std::unordered_set<::utils::VOXEL_LOC> vis_history_voxels_;
  int vis_pending_frames_       = 0;
  float vis_history_voxel_size_ = 0.8f;

 protected:
  Eigen::Matrix4f last_pose;
  Eigen::Matrix4f curr_pose;

  // relavant to lidar single frame size
  int frame_memory_size = 200000;
  // relavant to your RAM size
  int point_memory_size = 50000000;

  int ignore_count = 0;
};

}  // namespace mapping
}  // namespace jojo

#endif
