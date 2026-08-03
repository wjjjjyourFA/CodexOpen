#ifndef SEGMENT_H
#define SEGMENT_H

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "Eigen/Core"

// #include "modules/perception/common/base/point.h"
// #include "modules/perception/common/base/point_cloud.h"
#include "modules/perception/common/base/box3d_extra.h"

// jojo 定义的一种 object 处理中间态；包含 object 的所有信息；
namespace jojo {
namespace perception {
namespace base {

struct Segment {
  Segment();
  std::string ToString() const;
  void Reset();

  // @brief object id per frame, required
  int id = -1;

  // object 对应的密集点云，不是凸包；
  pcl::PointCloud<pcl::PointXYZI>::Ptr points;

  // 用于记录点在点云中的索引
  // 便于合并簇操作时更新点云
  std::vector<int> points_index_vector;

  // @brief boundingbox of the object, required
  BBox3DExtra bbox;
  // @brief center of the boundingbox (cx, cy, cz), required
  // 多传感器融合（雷达+相机）\ 轨迹预测 \ 稀疏点云 / 远距离雷达 / 单帧检测
  Eigen::Vector3f center = Eigen::Vector3f(0, 0, 0);
  // 稠密点云 / 近距离雷达 （几何中心 质心）
  Eigen::Vector3d centroid = Eigen::Vector3d(0, 0, 0);
  /* @brief size = [length, width, height] of boundingbox
     length is the size of the main direction, required
  */
  Eigen::Vector3f size = Eigen::Vector3f(0, 0, 0);
  // @brief anchor point, required
  // “底面中心点” 适合放在地面坐标系或车体坐标系中，作为参考原点
  Eigen::Vector3d anchor_point = Eigen::Vector3d(0, 0, 0);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // 拷贝构造函数（深拷贝点云）
  Segment(const Segment& other) {
    id           = other.id;
    bbox         = other.bbox;
    center       = other.center;
    centroid     = other.centroid;
    size         = other.size;
    anchor_point = other.anchor_point;
    // 其他成员拷贝...

    if (other.points) {
      // points = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>(*other.points);
      points.reset(new pcl::PointCloud<pcl::PointXYZI>(*other.points));
    }
  }

  // 此处保证外界调用 = 号时，不会出现浅拷贝，而是深拷贝
  // 使得 points 指针不会被覆盖
  // 拷贝赋值运算符（深拷贝点云）
  Segment& operator=(const Segment& other) {
    if (this != &other) {
      id           = other.id;
      bbox         = other.bbox;
      center       = other.center;
      centroid     = other.centroid;
      size         = other.size;
      anchor_point = other.anchor_point;
      // 其他成员拷贝...

      if (other.points) {
        // points = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>(*other.points);
        points.reset(new pcl::PointCloud<pcl::PointXYZI>(*other.points));
      } else {
        points.reset();
      }
    }
    return *this;
  }
};

using SegmentPtr      = std::shared_ptr<Segment>;
using SegmentConstPtr = std::shared_ptr<const Segment>;

}  // namespace base
}  // namespace perception
}  // namespace jojo

#endif  // SEGMENT_H
