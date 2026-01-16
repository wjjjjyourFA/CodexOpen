#ifndef RADAR_CAMERA_FUSION_H
#define RADAR_CAMERA_FUSION_H

#include "modules/perception/common/fusion/lidar2camera/lidar_camera_fusion.h"

namespace jojo {
namespace perception {
namespace fusion {

// 大部分 逻辑功能一样，但函数名变化了，因此采用继承与内部调用
// 父类的函数只会操作 父类的成员变量。
// 子类自己定义的同名变量，不会被父类的函数修改。
class RadarCameraFusion : public LidarCameraFusion {
 public:
  RadarCameraFusion();
  ~RadarCameraFusion() = default;

  void SetRadarPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);
  void SetRadarPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

  // void fuse();
  void show_radar_color_cloud();
};

}  // namespace fusion
}  // namespace perception
}  // namespace jojo

#endif  // RADAR_CAMERA_FUSION_H