#include "modules/perception/common/fusion/radar2camera/radar_camera_fusion.h"

namespace jojo {
namespace perception {
namespace fusion {

RadarCameraFusion::RadarCameraFusion() {
  // 父类构造函数已经初始化好 cloud_
  this->set_params("Radar", 500);
}

// RadarCameraFusion::~RadarCameraFusion() {}

void RadarCameraFusion::SetRadarPointCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) {
  SetLidarPointCloud(cloud);
}

void RadarCameraFusion::SetRadarPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
  SetLidarPointCloud(cloud);
}

// void RadarCameraFusion::fuse() { LidarCameraFusion::fuse(1, false); }

void RadarCameraFusion::show_radar_color_cloud() { show_lidar_color_cloud(); }

}  // namespace fusion
}  // namespace perception
}  // namespace jojo
