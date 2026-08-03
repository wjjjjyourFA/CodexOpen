#include "modules/mapping/map_processing/downsampling.h"

void VoxelizePreserveLabel(pcl::PointCloud<pcl::PointXYZI>::Ptr& src,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr& dst,
                           float leaf_size) {
  // code from ERASOR
  /**< IMPORTANT
  * Because PCL voxlizaiton just does average the intensity of point cloud,
  * so this function is to conduct voxelization followed by nearest points 
  * search to re-assign the label of each point */

  // std::cout << "VoxelizePreserveLabel start ... " << std::endl;

  // clang-format off
  pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_voxelized(new pcl::PointCloud<pcl::PointXYZI>);
  // pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_reassigned(new pcl::PointCloud<pcl::PointXYZI>);
  auto& ptr_reassigned = dst;
  // clang-format on

  // 1. Voxelization
  pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
  voxel_filter.setInputCloud(src);
  voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel_filter.filter(*ptr_voxelized);

  // 2. Find nearest point to update intensity (index and id)
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud(src);

  ptr_reassigned->clear();
  ptr_reassigned->points.reserve(ptr_voxelized->points.size());

  int K = 1;
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

  // 距离约束
  // float dist_limit = 0.866f * 0.866f * leaf_size * leaf_size;
  float dist_limit = leaf_size * leaf_size;

  // Set dst <- output
  for (const auto& pt : ptr_voxelized->points) {
    if (!pcl::isFinite(pt)) continue;

    if (kdtree.nearestKSearch(pt, K, pointIdxNKNSearch,
                              pointNKNSquaredDistance) > 0) {
      // 距离约束：避免跨 voxel 最近邻
      if (pointNKNSquaredDistance[0] > dist_limit) continue;

      /* way 1
      auto updated = pt;
      updated.intensity = (*src)[pointIdxNKNSearch[0]].intensity;
      ptr_reassigned->points.emplace_back(updated);
      */
      /* way 2 C17
      auto& updated     = ptr_reassigned->points.emplace_back(pt);
      updated.intensity = (*src)[pointIdxNKNSearch[0]].intensity;
      */
      // way 3
      ptr_reassigned->points.emplace_back(pt);
      auto& updated = ptr_reassigned->points.back();
      // Update meaned intensity to original intensity
      updated.intensity = (*src)[pointIdxNKNSearch[0]].intensity;
    }
  }

  dst->width    = ptr_reassigned->points.size();
  dst->height   = 1;
  dst->is_dense = true;

  // dst = ptr_reassigned;
}

void VoxelizePreserveLabel(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src,
                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr& dst,
                           float leaf_size) {
  // clang-format off
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_voxelized(new pcl::PointCloud<pcl::PointXYZRGB>);
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_reassigned(new pcl::PointCloud<pcl::PointXYZRGB>);
  auto& ptr_reassigned = dst;
  // clang-format on

  pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
  voxel_filter.setInputCloud(src);
  voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel_filter.filter(*ptr_voxelized);

  pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
  kdtree.setInputCloud(src);

  ptr_reassigned->clear();
  ptr_reassigned->points.reserve(ptr_voxelized->points.size());

  int K = 1;
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

  float dist_limit = leaf_size * leaf_size;

  for (const auto& pt : ptr_voxelized->points) {
    if (!pcl::isFinite(pt)) continue;

    if (kdtree.nearestKSearch(pt, K, pointIdxNKNSearch,
                              pointNKNSquaredDistance) > 0) {
      if (pointNKNSquaredDistance[0] > dist_limit) continue;

      ptr_reassigned->points.emplace_back(pt);
      auto& updated = ptr_reassigned->points.back();
      updated.rgb   = (*src)[pointIdxNKNSearch[0]].rgb;
    }
  }

  dst->width    = ptr_reassigned->points.size();
  dst->height   = 1;
  dst->is_dense = true;
}

void VoxelizeLargeScalePreserveLabel(pcl::PointCloud<pcl::PointXYZI>::Ptr& src,
                                     pcl::PointCloud<pcl::PointXYZI>::Ptr& dst,
                                     float leaf_size) {
  // std::cout << "VoxelizeLargeScalePreserveLabel start ... " << std::endl;

  // clang-format off
  pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_voxelized(new pcl::PointCloud<pcl::PointXYZI>);
  // pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_reassigned(new pcl::PointCloud<pcl::PointXYZI>);
  auto& ptr_reassigned = dst;
  // clang-format on

  // std::cout << "VoxelizeLargeScalePreserveLabel running ... " << std::endl;
  // std::cout << ptr_voxelized->size() << " points!" << std::endl;

  // 1. Voxelization
  // pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
  // voxel_filter.setInputCloud(src);
  // voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  // voxel_filter.filter(*ptr_voxelized);

  std::unordered_map<utils::VOXEL_LOC, utils::M_POINT> feature_map;
  VoxelizeLargeScale<pcl::PointXYZI>(src, ptr_voxelized, feature_map,
                                     leaf_size);

  // std::cout << "VoxelizeLargeScalePreserveLabel create kdtree ... " << std::endl;
  // std::cout << ptr_voxelized->size() << " points!" << std::endl;

  // 2. Find nearest point to update intensity (index and id)
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud(src);

  ptr_reassigned->clear();
  ptr_reassigned->points.reserve(ptr_voxelized->points.size());
  // std::cout << "ptr_reassigned: " << ptr_reassigned->size() << " points!" << std::endl;

  int K = 1;
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

  // std::cout << "VoxelizeLargeScalePreserveLabel find nearest pts ... " << std::endl;

  float dist_limit = leaf_size * leaf_size;

  // Set dst <- output
  // for (const auto& pt : ptr_voxelized->points) {
  for (size_t i = 0; i < ptr_voxelized->size(); i++) {
    if (i % 1000000 == 0) {
      std::cout << i << " / " << ptr_voxelized->size() << std::endl;
    }

    const auto& pt = ptr_voxelized->points[i];
    if (!pcl::isFinite(pt)) continue;

    if (kdtree.nearestKSearch(pt, K, pointIdxNKNSearch,
                              pointNKNSquaredDistance) > 0) {
      if (pointNKNSquaredDistance[0] > dist_limit) continue;

      ptr_reassigned->points.emplace_back(pt);
      auto& updated     = ptr_reassigned->points.back();
      updated.intensity = (*src)[pointIdxNKNSearch[0]].intensity;
    }
  }

  // std::cout << "VoxelizeLargeScalePreserveLabel end . " << std::endl;

  dst->width    = ptr_reassigned->points.size();
  dst->height   = 1;
  dst->is_dense = true;

  // dst = ptr_reassigned;

  // std::cout << "dst: " << dst->size() << " points!" << std::endl;
}

void VoxelizeLargeScalePreserveLabel(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& dst, float leaf_size) {
  // clang-format off
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_voxelized(new pcl::PointCloud<pcl::PointXYZRGB>);
  auto& ptr_reassigned = dst;
  // clang-format on

  // 1. Voxelization
  std::unordered_map<utils::VOXEL_LOC, utils::M_POINT> feature_map;
  VoxelizeLargeScale<pcl::PointXYZRGB>(src, ptr_voxelized, feature_map,
                                       leaf_size);

  // 2. Find nearest point to update intensity (index and id)
  pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
  kdtree.setInputCloud(src);

  ptr_reassigned->clear();
  ptr_reassigned->points.reserve(ptr_voxelized->points.size());

  int K = 1;
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

  float dist_limit = leaf_size * leaf_size;

  for (size_t i = 0; i < ptr_voxelized->size(); i++) {
    if (i % 1000000 == 0) {
      std::cout << i << " / " << ptr_voxelized->size() << std::endl;
    }

    const auto& pt = ptr_voxelized->points[i];
    if (!pcl::isFinite(pt)) continue;

    if (kdtree.nearestKSearch(pt, K, pointIdxNKNSearch,
                              pointNKNSquaredDistance) > 0) {
      if (pointNKNSquaredDistance[0] > dist_limit) continue;

      ptr_reassigned->points.emplace_back(pt);
      auto& updated = ptr_reassigned->points.back();
      updated.rgb   = (*src)[pointIdxNKNSearch[0]].rgb;
    }
  }

  dst->width    = ptr_reassigned->points.size();
  dst->height   = 1;
  dst->is_dense = true;
}
