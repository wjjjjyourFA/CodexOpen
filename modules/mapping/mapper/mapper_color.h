#ifndef CMapper_COLOR_H
#define CMapper_COLOR_H

#include "modules/common/math/math_utils.h"

#include "modules/common_struct/basic_msgs/Pose6D.h"
#include "modules/perception/common/camera/params/camera_params.h"
#include "modules/perception/common/config/sensor_extrinsics.h"
#include "modules/mapping/mapper/mapper.h"

#include "toolz/data_loader/group_convert.h"

using namespace std;
using namespace cv;

namespace jojo {
namespace mapping {

class MapperColor : public Mapper {
 public:
  MapperColor();
  virtual ~MapperColor();

  void InitCameraParams();

  void Run(std::shared_ptr<const jojo::tools::MeasureGroupDataSet> Measures);

  void Reset() override;

  void RealTimeShow(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);
  void VisualizeMap(bool b_pause = true) override;

  void UpdateIncrementalMap() override;

  void SaveMap(const std::string& path) override;

 protected:
  void FlushBufferToMap() override;

  bool InterpolatePose(double timestamp,
                       const jojo::common_struct::SE3Pose& pose1,
                       const jojo::common_struct::SE3Pose& pose2,
                       Eigen::Matrix4d& pose_out);

 private:
  std::shared_ptr<jojo::perception::camera::Lidar2CameraMatrix> matrix;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr world_point_cloud;
  int world_point_cloud_idx = 0;

  // color 缓存
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr ds_history_point_cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr ds_world_point_cloud;

  // 实时可视化专用
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr vis_frame;
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> vis_chunks_;
  int vis_chunk_id_ = 0;
};

}  // namespace mapping
}  // namespace jojo

#endif
