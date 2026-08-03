#ifndef EDGE_EXTRACTION_H
#define EDGE_EXTRACTION_H

#include <stdio.h>
#include <stdlib.h>

#include <iostream>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

// #include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

// #include <opencv2/core.hpp>
/* 我暂时希望只以点云操作 */

namespace jojo {
namespace perception {
namespace algorithm {

// 通过 impl 模板套壳，避免大段函数被写入 .h 文件
// 注意需要在 .cpp 中显示实例化
class PointCloudEdgeExtractor {
 public:
  PointCloudEdgeExtractor() {};
  ~PointCloudEdgeExtractor() {};

  template <typename PointT>
  typename pcl::PointCloud<PointT>::Ptr estimate_borders(
      typename pcl::PointCloud<PointT>::Ptr& cloud, float normal_radius,
      float radius_estimation, float angle_threshold = default_angle_threshold,
      bool show = false) {
    return estimate_borders_impl<PointT>(
        cloud, normal_radius, radius_estimation, angle_threshold, show);
  }

  void show_points(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);

 private:
  template <typename PointT>
  typename pcl::PointCloud<PointT>::Ptr estimate_borders_impl(
      typename pcl::PointCloud<PointT>::Ptr& cloud, float normal_radius,
      float radius_estimation, float angle_threshold, bool show);

  // default params
  static constexpr float default_angle_threshold = M_PI / 4;
};

}  // namespace algorithm
}  // namespace perception
}  // namespace jojo

#endif  // EDGE_EXTRACTION_H
