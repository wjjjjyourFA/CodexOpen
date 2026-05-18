#pragma once

#include <memory>
#include <utility>
#include <vector>

namespace jojo {
namespace perception {
namespace algorithm {

template <typename PointT>
void DownsamplingVoxelGrid(typename pcl::PointCloud<PointT>::Ptr cloud,
                  typename pcl::PointCloud<PointT>::Ptr down_cloud) {
  // 创建降采样后的点云对象
  // pcl::PointCloud<PointT>::Ptr down_cloud(new pcl::PointCloud<PointT>);

  if (!cloud || !down_cloud) {
    throw std::runtime_error("Null pointer input");
  }

  // TODO：设置成全局实例使用
  // 设置 VoxelGrid 滤波器
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud(cloud);
  // 设置体素网格的大小
  // 0.1f ~ 0.3f   // 常规
  // 0.5f          // 粗
  // 1.0f          // 很粗（地图级）
  // sor.setLeafSize(0.5f, 0.5f, 0.5f);
  sor.setLeafSize(1.0f, 1.0f, 1.0f);
  sor.filter(*down_cloud);

  std::cout << "downsampled point cloud has " << down_cloud->size()
            << " points." << std::endl;
}

}  // namespace algorithm
}  // namespace perception
}  // namespace jojo
