/*
 * LiDAR target detection adapted from hku-mars/FAST-Calib (GPLv2).
 */

#include "calibration_internal.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace jojo {
namespace tools {
namespace fast_calib {
namespace {

using RingCloud = pcl::PointCloud<PointXYZRing>;
using XyzCloud = pcl::PointCloud<pcl::PointXYZ>;

bool SetError(const std::string& message, std::string* error) {
  if (error != nullptr) {
    *error = message;
  }
  return false;
}

void ApplyRangeFilter(const Params& params,
                      const RingCloud::ConstPtr& input,
                      RingCloud::Ptr output) {
  RingCloud::Ptr first(new RingCloud);
  RingCloud::Ptr second(new RingCloud);

  pcl::PassThrough<PointXYZRing> pass;
  pass.setInputCloud(input);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(params.x_min, params.x_max);
  pass.filter(*first);

  pass.setInputCloud(first);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(params.y_min, params.y_max);
  pass.filter(*second);

  pass.setInputCloud(second);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(params.z_min, params.z_max);
  pass.filter(*output);
}

bool SegmentPlane(const RingCloud::ConstPtr& input,
                  RingCloud::Ptr plane,
                  pcl::ModelCoefficients::Ptr coefficients,
                  std::string* error) {
  if (input->size() < 3) {
    return SetError("fewer than three points remain after range filtering",
                    error);
  }
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<PointXYZRing> segmentation;
  segmentation.setModelType(pcl::SACMODEL_PLANE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setDistanceThreshold(0.01);
  segmentation.setInputCloud(input);
  segmentation.segment(*inliers, *coefficients);
  if (inliers->indices.empty() || coefficients->values.size() < 4) {
    return SetError("unable to segment a calibration-target plane", error);
  }

  pcl::ExtractIndices<PointXYZRing> extract;
  extract.setInputCloud(input);
  extract.setIndices(inliers);
  extract.filter(*plane);
  if (plane->empty()) {
    return SetError("segmented calibration-target plane is empty", error);
  }
  return true;
}

Eigen::Matrix3d OriginalPlaneAlignment(const Eigen::Vector3d& normal) {
  const Eigen::Vector3d z_axis(0.0, 0.0, 1.0);
  const Eigen::Vector3d axis = normal.cross(z_axis);
  const double cosine = std::max(-1.0, std::min(1.0, normal.dot(z_axis)));
  const double angle = std::acos(cosine);
  if (axis.squaredNorm() < 1e-16) {
    if (cosine >= 0.0) {
      return Eigen::Matrix3d::Identity();
    }
    return Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())
        .toRotationMatrix();
  }
  // The source algorithm passes this cross product directly to AngleAxisd.
  return Eigen::AngleAxisd(angle, axis).toRotationMatrix();
}

}  // namespace

void LidarDetector::ClearDebugClouds() {
  debug_clouds_.filtered->clear();
  debug_clouds_.plane->clear();
  debug_clouds_.aligned->clear();
  debug_clouds_.edge->clear();
  debug_clouds_.centers_z0->clear();
}

bool LidarDetector::DetectMechanical(
    const RingCloud::ConstPtr& cloud,
    XyzCloud::Ptr centers,
    std::string* error) {
  centers->clear();
  ClearDebugClouds();
  if (cloud == nullptr || cloud->empty()) {
    return SetError("input LiDAR cloud is empty", error);
  }

  ApplyRangeFilter(params_, cloud, debug_clouds_.filtered);
  std::cout << "[LiDAR] Depth filtered cloud size: "
            << debug_clouds_.filtered->size() << std::endl;

  pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients);
  if (!SegmentPlane(debug_clouds_.filtered, debug_clouds_.plane,
                    plane_coefficients, error)) {
    return false;
  }
  std::cout << "[LiDAR] Plane cloud size: " << debug_clouds_.plane->size()
            << std::endl;

  std::unordered_map<unsigned int, std::vector<int>> ring_indices;
  ring_indices.reserve(64);
  for (int index = 0;
       index < static_cast<int>(debug_clouds_.filtered->size()); ++index) {
    ring_indices[debug_clouds_.filtered->points[static_cast<std::size_t>(index)]
                     .ring]
        .push_back(index);
  }

  const auto& coefficients = plane_coefficients->values;
  const Eigen::Vector3d raw_normal(coefficients[0], coefficients[1],
                                   coefficients[2]);
  const double normal_norm = raw_normal.norm();
  if (normal_norm <= 0.0) {
    return SetError("calibration-target plane normal is invalid", error);
  }
  const Eigen::Vector3d normal = raw_normal / normal_norm;
  constexpr double kNeighborGapThreshold = 0.10;
  constexpr int kMinPointsPerRing = 10;
  debug_clouds_.edge->reserve(debug_clouds_.filtered->size());
  for (auto& entry : ring_indices) {
    const auto& indices = entry.second;
    if (static_cast<int>(indices.size()) < kMinPointsPerRing) {
      continue;
    }
    for (std::size_t index = 1; index + 1 < indices.size(); ++index) {
      const auto& previous = debug_clouds_.filtered->points[
          static_cast<std::size_t>(indices[index - 1])];
      const auto& current = debug_clouds_.filtered->points[
          static_cast<std::size_t>(indices[index])];
      const auto& next = debug_clouds_.filtered->points[
          static_cast<std::size_t>(indices[index + 1])];
      const double plane_distance =
          std::fabs(coefficients[0] * current.x +
                    coefficients[1] * current.y +
                    coefficients[2] * current.z + coefficients[3]) /
          normal_norm;
      if (plane_distance >= 0.03) {
        continue;
      }
      const double previous_distance =
          std::sqrt(std::pow(current.x - previous.x, 2) +
                    std::pow(current.y - previous.y, 2) +
                    std::pow(current.z - previous.z, 2));
      const double next_distance =
          std::sqrt(std::pow(current.x - next.x, 2) +
                    std::pow(current.y - next.y, 2) +
                    std::pow(current.z - next.z, 2));
      if (previous_distance > kNeighborGapThreshold ||
          next_distance > kNeighborGapThreshold) {
        debug_clouds_.edge->push_back(
            pcl::PointXYZ(current.x, current.y, current.z));
      }
    }
  }
  std::cout << "[LiDAR] Extracted " << debug_clouds_.edge->size()
            << " edge points (mechanical LiDAR by neighbor distance)."
            << std::endl;
  if (debug_clouds_.edge->empty()) {
    return SetError("mechanical LiDAR edge cloud is empty", error);
  }

  const Eigen::Matrix3d alignment = OriginalPlaneAlignment(normal);
  double average_z = 0.0;
  debug_clouds_.aligned->reserve(debug_clouds_.edge->size());
  for (const auto& point : *debug_clouds_.edge) {
    const Eigen::Vector3d aligned =
        alignment * Eigen::Vector3d(point.x, point.y, point.z);
    debug_clouds_.aligned->push_back(
        pcl::PointXYZ(aligned.x(), aligned.y(), 0.0));
    average_z += aligned.z();
  }
  average_z /= static_cast<double>(debug_clouds_.edge->size());

  XyzCloud::Ptr remaining(new XyzCloud(*debug_clouds_.aligned));
  pcl::SACSegmentation<pcl::PointXYZ> circle_segmentation;
  circle_segmentation.setModelType(pcl::SACMODEL_CIRCLE2D);
  circle_segmentation.setMethodType(pcl::SAC_RANSAC);
  circle_segmentation.setDistanceThreshold(0.02);
  circle_segmentation.setOptimizeCoefficients(true);
  circle_segmentation.setMaxIterations(1000);
  circle_segmentation.setRadiusLimits(params_.circle_radius - 0.03,
                                      params_.circle_radius + 0.03);
  pcl::ModelCoefficients::Ptr circle_coefficients(
      new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  std::cout << "[LiDAR] Start circle detection, initial cloud size = "
            << remaining->size() << std::endl;
  while (remaining->size() > 3) {
    circle_segmentation.setInputCloud(remaining);
    circle_segmentation.segment(*inliers, *circle_coefficients);
    if (inliers->indices.empty()) {
      break;
    }
    if (inliers->indices.size() < 5 ||
        circle_coefficients->values.size() < 3) {
      break;
    }
    debug_clouds_.centers_z0->push_back(pcl::PointXYZ(
        circle_coefficients->values[0], circle_coefficients->values[1], 0.0));

    extract.setInputCloud(remaining);
    extract.setIndices(inliers);
    extract.setNegative(true);
    XyzCloud::Ptr next_remaining(new XyzCloud);
    extract.filter(*next_remaining);
    remaining.swap(next_remaining);
    inliers->indices.clear();
  }

  const auto groups = BuildCombinations(
      static_cast<int>(debug_clouds_.centers_z0->size()),
      kTargetCircleCount);
  int best_group = -1;
  double best_score = -1.0;
  for (std::size_t group_index = 0; group_index < groups.size();
       ++group_index) {
    std::vector<pcl::PointXYZ> candidate_set;
    for (const int candidate_index : groups[group_index]) {
      candidate_set.push_back(debug_clouds_.centers_z0->points[
          static_cast<std::size_t>(candidate_index)]);
    }
    const double score = IsTargetRectangle(
                             candidate_set, params_.delta_width_circles,
                             params_.delta_height_circles)
                             ? 1.0
                             : -1.0;
    if (best_score == 1.0 && score == 1.0) {
      return SetError(
          "more than one mechanical-LiDAR candidate set fits target geometry",
          error);
    }
    if (score > best_score) {
      best_score = score;
      best_group = static_cast<int>(group_index);
    }
  }
  if (best_group < 0) {
    return SetError(
        "no mechanical-LiDAR candidate set matches target geometry", error);
  }

  const Eigen::Matrix3d inverse_alignment = alignment.inverse();
  for (const int candidate_index :
       groups[static_cast<std::size_t>(best_group)]) {
    const auto& center = debug_clouds_.centers_z0->points[
        static_cast<std::size_t>(candidate_index)];
    const Eigen::Vector3d original =
        inverse_alignment *
        Eigen::Vector3d(center.x, center.y, center.z + average_z);
    centers->push_back(
        pcl::PointXYZ(original.x(), original.y(), original.z()));
  }
  return true;
}

bool LidarDetector::DetectSolidState(
    const RingCloud::ConstPtr& cloud,
    XyzCloud::Ptr centers,
    std::string* error) {
  centers->clear();
  ClearDebugClouds();
  if (cloud == nullptr || cloud->empty()) {
    return SetError("input LiDAR cloud is empty", error);
  }

  ApplyRangeFilter(params_, cloud, debug_clouds_.filtered);
  std::cout << "[LiDAR] Filtered cloud size: "
            << debug_clouds_.filtered->size() << std::endl;
  if (debug_clouds_.filtered->empty()) {
    return SetError("solid-state LiDAR range filter produced no points",
                    error);
  }
  pcl::VoxelGrid<PointXYZRing> voxel_filter;
  voxel_filter.setInputCloud(debug_clouds_.filtered);
  voxel_filter.setLeafSize(0.005F, 0.005F, 0.005F);
  RingCloud::Ptr downsampled(new RingCloud);
  voxel_filter.filter(*downsampled);
  debug_clouds_.filtered.swap(downsampled);
  std::cout << "[LiDAR] Downsampled cloud size: "
            << debug_clouds_.filtered->size() << std::endl;

  pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients);
  if (!SegmentPlane(debug_clouds_.filtered, debug_clouds_.plane,
                    plane_coefficients, error)) {
    return false;
  }
  std::cout << "[LiDAR] Plane cloud size: " << debug_clouds_.plane->size()
            << std::endl;

  Eigen::Vector3d normal(plane_coefficients->values[0],
                         plane_coefficients->values[1],
                         plane_coefficients->values[2]);
  if (normal.norm() <= 0.0) {
    return SetError("calibration-target plane normal is invalid", error);
  }
  normal.normalize();
  const Eigen::Matrix3d alignment = OriginalPlaneAlignment(normal);
  double average_z = 0.0;
  debug_clouds_.aligned->reserve(debug_clouds_.plane->size());
  for (const auto& point : *debug_clouds_.plane) {
    const Eigen::Vector3d aligned =
        alignment * Eigen::Vector3d(point.x, point.y, point.z);
    debug_clouds_.aligned->push_back(
        pcl::PointXYZ(aligned.x(), aligned.y(), 0.0));
    average_z += aligned.z();
  }
  average_z /= static_cast<double>(debug_clouds_.plane->size());

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  normal_estimator.setInputCloud(debug_clouds_.aligned);
  normal_estimator.setRadiusSearch(0.03);
  normal_estimator.compute(*normals);

  pcl::PointCloud<pcl::Boundary> boundaries;
  pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary>
      boundary_estimator;
  boundary_estimator.setInputCloud(debug_clouds_.aligned);
  boundary_estimator.setInputNormals(normals);
  boundary_estimator.setRadiusSearch(0.03);
  boundary_estimator.setAngleThreshold(M_PI / 4.0);
  boundary_estimator.compute(boundaries);
  debug_clouds_.edge->reserve(debug_clouds_.aligned->size());
  for (std::size_t index = 0;
       index < debug_clouds_.aligned->size() && index < boundaries.size();
       ++index) {
    if (boundaries.points[index].boundary_point > 0) {
      debug_clouds_.edge->push_back(debug_clouds_.aligned->points[index]);
    }
  }
  std::cout << "[LiDAR] Extracted " << debug_clouds_.edge->size()
            << " edge points." << std::endl;
  if (debug_clouds_.edge->empty()) {
    return SetError("solid-state LiDAR edge cloud is empty", error);
  }

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(debug_clouds_.edge);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> clustering;
  clustering.setClusterTolerance(0.05);
  clustering.setMinClusterSize(50);
  clustering.setMaxClusterSize(1000);
  clustering.setSearchMethod(tree);
  clustering.setInputCloud(debug_clouds_.edge);
  clustering.extract(cluster_indices);
  std::cout << "[LiDAR] Number of edge clusters: " << cluster_indices.size()
            << std::endl;

  const Eigen::Matrix3d inverse_alignment = alignment.inverse();
  debug_clouds_.centers_z0->reserve(kTargetCircleCount);
  for (const auto& indices : cluster_indices) {
    XyzCloud::Ptr cluster(new XyzCloud);
    for (const int point_index : indices.indices) {
      cluster->push_back(
          debug_clouds_.edge->points[static_cast<std::size_t>(point_index)]);
    }

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> segmentation;
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_CIRCLE2D);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setDistanceThreshold(0.01);
    segmentation.setMaxIterations(1000);
    segmentation.setInputCloud(cluster);
    segmentation.segment(*inliers, *coefficients);
    if (inliers->indices.empty() || coefficients->values.size() < 3) {
      continue;
    }

    double fitting_error = 0.0;
    for (const int point_index : inliers->indices) {
      const auto& point =
          cluster->points[static_cast<std::size_t>(point_index)];
      const double dx = point.x - coefficients->values[0];
      const double dy = point.y - coefficients->values[1];
      fitting_error +=
          std::fabs(std::sqrt(dx * dx + dy * dy) - params_.circle_radius);
    }
    fitting_error /= static_cast<double>(inliers->indices.size());
    if (fitting_error >= 0.025) {
      continue;
    }

    const pcl::PointXYZ center(coefficients->values[0],
                               coefficients->values[1], 0.0);
    debug_clouds_.centers_z0->push_back(center);
    const Eigen::Vector3d original =
        inverse_alignment *
        Eigen::Vector3d(center.x, center.y, center.z + average_z);
    centers->push_back(
        pcl::PointXYZ(original.x(), original.y(), original.z()));
  }

  if (centers->size() != kTargetCircleCount) {
    std::ostringstream stream;
    stream << "solid-state LiDAR detected " << centers->size()
           << " circle center(s); expected " << kTargetCircleCount;
    return SetError(stream.str(), error);
  }
  return true;
}

}  // namespace fast_calib
}  // namespace tools
}  // namespace jojo
