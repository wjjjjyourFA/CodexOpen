#pragma once

#include "modules/perception/lidar_local_mapping/local_mapping_base.h"

namespace jojo {
namespace perception {

class LidarLocalMapping : public LocalMappingBase<pcl::PointXYZI> {
 public:
  LidarLocalMapping();
  ~LidarLocalMapping();

  void Run(const pcl::PointCloud<pcl::PointXYZI>::Ptr& frame,
           const Eigen::Matrix4f& pose);
};

}  // namespace perception
}  // namespace jojo
