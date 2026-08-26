#include "modules/perception/lidar_local_mapping/lidar_local_mapping.h"

namespace jojo {
namespace perception {

LidarLocalMapping::LidarLocalMapping() { LocalMappingBase::Init(); }

LidarLocalMapping::~LidarLocalMapping() {}

void LidarLocalMapping::Run(const pcl::PointCloud<pcl::PointXYZI>::Ptr& frame,
                            const Eigen::Matrix4f& pose) {
  if (!frame) return;

  // 先使用传入的 pose 进行粗筛
  if (!this->NeedNewKeyFrame(pose)) return;

  // clang-format off
  pcl::PointCloud<pcl::PointXYZI>::Ptr current_frame(new pcl::PointCloud<pcl::PointXYZI>);
  this->BuildCurrentFrameCloud(frame, current_frame);
  // clang-format on

  // TODO：ScanMatch() ==> scan-to-local_map
  // if (!keyframes_.empty()) {
  //   new_pose = this->ScanMatch(cur_cloud_, pose);
  // }

  this->AddKeyFrame(current_frame, pose);

  this->UpdateLocalMap();
}

}  // namespace perception
}  // namespace jojo
