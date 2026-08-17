/**
 * This file is based on ROG-Map (HKU MaRS Lab) and retains its LGPL notice
 * from the repository distribution. Middleware integration has been removed
 * from this translation unit.
 */

#include "rog_map/rog_map.h"

#include <algorithm>
#include <stdexcept>

#include <pcl/io/pcd_io.h>

namespace rog_map {

ROGMap::ROGMap(const Config& config) {
  cfg_ = config;
  cfg_.Finalize();
  initProbMap();

  robot_state_.p = cfg_.fix_map_origin;
  robot_state_.q = Quatf::Identity();
  if (cfg_.map_sliding_en) {
    mapSliding(Vec3f::Zero());
    inf_map_->mapSliding(Vec3f::Zero());
  } else {
    local_map_bound_min_d_ = -cfg_.half_map_size_d + cfg_.fix_map_origin;
    local_map_bound_max_d_ = cfg_.half_map_size_d + cfg_.fix_map_origin;
    mapSliding(cfg_.fix_map_origin);
    inf_map_->mapSliding(cfg_.fix_map_origin);
  }

  if (cfg_.load_pcd_en) {
    PointCloud pcd_map;
    if (pcl::io::loadPCDFile(cfg_.pcd_name, pcd_map) == -1) {
      throw std::runtime_error("failed to load ROG-Map PCD: " +
                               cfg_.pcd_name);
    }
    updateOccPointCloud(pcd_map);
    if (cfg_.esdf_en) {
      esdf_map_->updateESDF3D(robot_state_.p);
    }
    map_empty_ = false;
  }
}

bool ROGMap::isLineFree(const Vec3f& start_pt, const Vec3f& end_pt,
                        const bool& use_inf_map,
                        const bool& use_unk_as_occ) const {
  if (start_pt.array().isNaN().any() || end_pt.array().isNaN().any()) {
    return false;
  }
  raycaster::RayCaster raycaster;
  raycaster.setResolution(use_inf_map ? cfg_.inflation_resolution
                                      : cfg_.resolution);
  Vec3f ray_pt;
  raycaster.setInput(start_pt, end_pt);
  while (raycaster.step(ray_pt)) {
    if (!use_unk_as_occ) {
      if (use_inf_map ? isOccupiedInflate(ray_pt) : isOccupied(ray_pt)) {
        return false;
      }
    } else if (use_inf_map
                   ? (isUnknownInflate(ray_pt) ||
                      isOccupiedInflate(ray_pt))
                   : !isKnownFree(ray_pt)) {
      return false;
    }
  }
  return true;
}

bool ROGMap::isLineFree(const Vec3f& start_pt, const Vec3f& end_pt,
                        const double& max_dis,
                        const vec_Vec3i& neighbor_list) const {
  raycaster::RayCaster raycaster;
  raycaster.setResolution(cfg_.resolution);
  Vec3f ray_pt;
  raycaster.setInput(start_pt, end_pt);
  while (raycaster.step(ray_pt)) {
    if (max_dis > 0 && (ray_pt - start_pt).norm() > max_dis) {
      return false;
    }
    if (neighbor_list.empty()) {
      if (isOccupied(ray_pt)) {
        return false;
      }
      continue;
    }
    Vec3i ray_pt_id_g;
    posToGlobalIndex(ray_pt, ray_pt_id_g);
    for (const Vec3i& neighbor : neighbor_list) {
      Vec3i shifted = ray_pt_id_g + neighbor;
      if (isOccupied(shifted)) {
        return false;
      }
    }
  }
  return true;
}

bool ROGMap::isLineFree(const Vec3f& start_pt, const Vec3f& end_pt,
                        Vec3f& free_local_goal, const double& max_dis,
                        const vec_Vec3i& neighbor_list) const {
  raycaster::RayCaster raycaster;
  raycaster.setResolution(cfg_.resolution);
  Vec3f ray_pt;
  raycaster.setInput(start_pt, end_pt);
  free_local_goal = start_pt;
  while (raycaster.step(ray_pt)) {
    free_local_goal = ray_pt;
    if (max_dis > 0 && (ray_pt - start_pt).norm() > max_dis) {
      return false;
    }
    if (neighbor_list.empty()) {
      if (isOccupied(ray_pt)) {
        return false;
      }
      continue;
    }
    Vec3i ray_pt_id_g;
    posToGlobalIndex(ray_pt, ray_pt_id_g);
    for (const Vec3i& neighbor : neighbor_list) {
      Vec3i shifted = ray_pt_id_g + neighbor;
      if (isOccupied(shifted)) {
        return false;
      }
    }
  }
  free_local_goal = end_pt;
  return true;
}

void ROGMap::SetRobotState(const Pose& pose, double receive_time_seconds) {
  robot_state_.p = pose.first;
  robot_state_.q = pose.second;
  robot_state_.rcv_time = receive_time_seconds;
  robot_state_.rcv = true;
  robot_state_.yaw = get_yaw_from_quaternion<double>(pose.second);
  updateLocalBox(pose.first);
}

bool ROGMap::UpdateFromLatestPose(const PointCloud& cloud) {
  if (!robot_state_.rcv || cloud.empty()) {
    return false;
  }
  const Pose pose(robot_state_.p, robot_state_.q);
  updateProbMap(cloud, pose);
  map_empty_ = false;
  return true;
}

void ROGMap::updateMap(const PointCloud& cloud, const Pose& pose,
                       double receive_time_seconds) {
  if (cloud.empty()) {
    return;
  }
  SetRobotState(pose, receive_time_seconds);
  updateProbMap(cloud, pose);
  map_empty_ = false;
}

RobotState ROGMap::getRobotState() const {
  return robot_state_;
}

pcl::PointCloud<pcl::PointXYZ> ROGMap::ToPointCloud(
    const vec_E<Vec3f>& points) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.reserve(points.size());
  for (const Vec3f& position : points) {
    cloud.emplace_back(static_cast<float>(position.x()),
                       static_cast<float>(position.y()),
                       static_cast<float>(position.z()));
  }
  cloud.width = cloud.size();
  cloud.height = 1;
  cloud.is_dense = true;
  return cloud;
}

VisualizationOutput ROGMap::BuildVisualization(
    const VisualizationRequest& request) const {
  VisualizationOutput output;
  if (map_empty_ || !robot_state_.rcv) {
    return output;
  }

  Vec3f box_min;
  Vec3f box_max;
  if (request.use_explicit_bounds) {
    const Vec3f offset = request.bounds_relative_to_robot
                             ? robot_state_.p
                             : Vec3f::Zero();
    box_min = request.minimum + offset;
    box_max = request.maximum + offset;
  } else {
    box_min = robot_state_.p - request.range / 2.0;
    box_max = robot_state_.p + request.range / 2.0;
  }
  boundBoxByLocalMap(box_min, box_max);
  if ((box_max - box_min).minCoeff() <= 0.0) {
    return output;
  }

  vec_E<Vec3f> points;
  if (request.include_occupied) {
    boxSearch(box_min, box_max, OCCUPIED, points);
    output.occupied = ToPointCloud(points);
  }
  if (request.include_unknown) {
    points.clear();
    boxSearch(box_min, box_max, UNKNOWN, points);
    output.unknown = ToPointCloud(points);
  }
  if (request.include_inflated_occupied) {
    points.clear();
    boxSearchInflate(box_min, box_max, OCCUPIED, points);
    output.inflated_occupied = ToPointCloud(points);
  }
  if (request.include_inflated_unknown && cfg_.unk_inflation_en) {
    points.clear();
    boxSearchInflate(box_min, box_max, UNKNOWN, points);
    output.inflated_unknown = ToPointCloud(points);
  }
  if (request.include_frontier && cfg_.frontier_extraction_en) {
    points.clear();
    boxSearch(box_min, box_max, FRONTIER, points);
    output.frontier = ToPointCloud(points);
  }
  if (request.include_esdf && cfg_.esdf_en) {
    esdf_map_->getESDFOccCloud(box_min, box_max,
                               output.occupied_esdf);
    esdf_map_->getPositiveESDFCloud(box_min, box_max,
                                    robot_state_.p.z() - 0.5,
                                    output.positive_esdf);
    esdf_map_->getNegativeESDFCloud(box_min, box_max,
                                    robot_state_.p.z() - 0.5,
                                    output.negative_esdf);
  }

  output.visualization_bounds.minimum = box_min;
  output.visualization_bounds.maximum = box_max;
  Vec3f local_min(-999.0, -999.0, -999.0);
  Vec3f local_max(999.0, 999.0, 999.0);
  boundBoxByLocalMap(local_min, local_max);
  output.local_map_bounds.minimum = local_min;
  output.local_map_bounds.maximum = local_max;
  output.update_bounds.minimum = raycast_data_.cache_box_min;
  output.update_bounds.maximum = raycast_data_.cache_box_max;
  output.local_map_origin = local_map_origin_d_;
  output.ready = true;
  return output;
}

}  // namespace rog_map
