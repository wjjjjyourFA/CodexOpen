#pragma once

#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "modules/mapping/map_processing/utils.h"

template <typename PointT>
void VoxelizeLargeScale(
    typename pcl::PointCloud<PointT>::Ptr& src,
    typename pcl::PointCloud<PointT>::Ptr& dst,
    std::unordered_map<utils::VOXEL_LOC, utils::M_POINT>& feature_map,
    float leaf_size) {
  // std::cout << "VoxelizeLargeScale start ... " << std::endl;

  // pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_voxelized(new pcl::PointCloud<pcl::PointXYZ>);
  auto& ptr_voxelized = dst;

  feature_map.reserve(src->size());

  size_t pt_size = src->size();
  for (size_t i = 0; i < pt_size; i++) {
    auto& pt = src->points[i];
    if (!pcl::isFinite(pt)) continue;
    /* 
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
      continue;
    */

    /* way 1
    float loc_xyz[3];
    for (int j = 0; j < 3; j++) {
      loc_xyz[j] = pt.data[j] / leaf_size;
      if (loc_xyz[j] < 0) loc_xyz[j] -= 1.0;
    }
    utils::VOXEL_LOC key((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
                         (int64_t)loc_xyz[2]);
    */
    // /* way 2
    int64_t ix = static_cast<int64_t>(std::floor(pt.x / leaf_size));
    int64_t iy = static_cast<int64_t>(std::floor(pt.y / leaf_size));
    int64_t iz = static_cast<int64_t>(std::floor(pt.z / leaf_size));
    utils::VOXEL_LOC key(ix, iy, iz);
    // */

    /* way 1
    auto iter = feature_map.find(key);
    if (iter != feature_map.end()) {
      iter->second.xyz[0] += pt.x;
      iter->second.xyz[1] += pt.y;
      iter->second.xyz[2] += pt.z;
      iter->second.count++;
    } else {
      utils::M_POINT anp;
      anp.xyz[0] = pt.x;
      anp.xyz[1] = pt.y;
      anp.xyz[2] = pt.z;
      anp.count  = 1;

      feature_map[key] = anp;
    }
    */
    // /* way 2
    auto& voxel = feature_map[key];
    voxel.xyz[0] += pt.x;
    voxel.xyz[1] += pt.y;
    voxel.xyz[2] += pt.z;
    voxel.count++;
    // */
  }

  // std::cout << "VoxelizeLargeScale running ... " << std::endl;

  pt_size = feature_map.size();
  ptr_voxelized->clear();
  ptr_voxelized->points.resize(pt_size);
  // std::cout << "feature_map: " << pt_size << " points!" << std::endl;

  size_t i = 0;
  /* way 1
  for (auto iter = feature_map.begin(); iter != feature_map.end(); ++iter) {
    ptr_voxelized->points[i].x = iter->second.xyz[0] / iter->second.count;
    ptr_voxelized->points[i].y = iter->second.xyz[1] / iter->second.count;
    ptr_voxelized->points[i].z = iter->second.xyz[2] / iter->second.count;
    i++;
  }
  */
  // /* way 2
  auto& pts = ptr_voxelized->points;
  for (const auto& kv : feature_map) {
    const auto& v = kv.second;

    const float inv = 1.0f / v.count;

    pts[i].x = v.xyz[0] * inv;
    pts[i].y = v.xyz[1] * inv;
    pts[i].z = v.xyz[2] * inv;
    ++i;
  }
  // */
  // std::cout << "ptr_voxelized: " << ptr_voxelized->size() << " points!" << std::endl;

  // std::cout << "VoxelizeLargeScale running ...... " << std::endl;

  dst->width    = ptr_voxelized->points.size();
  dst->height   = 1;
  dst->is_dense = false;

  // dst = ptr_voxelized;
}

void VoxelizePreserveLabel(pcl::PointCloud<pcl::PointXYZI>::Ptr& src,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr& dst,
                           float leaf_size);

void VoxelizePreserveLabel(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src,
                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr& dst,
                           float leaf_size);

void VoxelizeLargeScalePreserveLabel(pcl::PointCloud<pcl::PointXYZI>::Ptr& src,
                                     pcl::PointCloud<pcl::PointXYZI>::Ptr& dst,
                                     float leaf_size);

void VoxelizeLargeScalePreserveLabel(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& dst, float leaf_size);