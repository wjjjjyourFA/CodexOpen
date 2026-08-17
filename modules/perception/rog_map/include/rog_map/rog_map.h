/**
 * Pure ROG-Map facade.
 *
 * ROS subscription, TF, timers, dynamic reconfigure, and message conversion
 * live in ros1/src/perception/rog_map.
 */
#pragma once

#include <memory>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rog_map/prob_map.h>

namespace rog_map {

struct MapBounds {
  Vec3f minimum{0.0, 0.0, 0.0};
  Vec3f maximum{0.0, 0.0, 0.0};
};

struct VisualizationRequest {
  Vec3f range{10.0, 10.0, 5.0};
  bool use_explicit_bounds{false};
  bool bounds_relative_to_robot{false};
  Vec3f minimum{-5.0, -5.0, -2.5};
  Vec3f maximum{5.0, 5.0, 2.5};
  bool include_unknown{false};
  bool include_inflated_unknown{false};
  bool include_frontier{false};
  bool include_occupied{true};
  bool include_inflated_occupied{true};
  bool include_esdf{false};
};

struct VisualizationOutput {
  bool ready{false};
  pcl::PointCloud<pcl::PointXYZ> occupied;
  pcl::PointCloud<pcl::PointXYZ> unknown;
  pcl::PointCloud<pcl::PointXYZ> inflated_occupied;
  pcl::PointCloud<pcl::PointXYZ> inflated_unknown;
  pcl::PointCloud<pcl::PointXYZ> frontier;
  pcl::PointCloud<pcl::PointXYZI> positive_esdf;
  pcl::PointCloud<pcl::PointXYZI> negative_esdf;
  pcl::PointCloud<pcl::PointXYZI> occupied_esdf;
  MapBounds visualization_bounds;
  MapBounds local_map_bounds;
  MapBounds update_bounds;
  Vec3f local_map_origin{0.0, 0.0, 0.0};
};

class ROGMap : public ProbMap {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<ROGMap>;

  explicit ROGMap(const Config& config);
  ~ROGMap() = default;

  bool isLineFree(const Vec3f& start_pt, const Vec3f& end_pt,
                  const double& max_dis = 999999,
                  const vec_Vec3i& neighbor_list = vec_Vec3i{}) const;
  bool isLineFree(const Vec3f& start_pt, const Vec3f& end_pt,
                  Vec3f& free_local_goal, const double& max_dis = 999999,
                  const vec_Vec3i& neighbor_list = vec_Vec3i{}) const;
  bool isLineFree(const Vec3f& start_pt, const Vec3f& end_pt,
                  const bool& use_inf_map,
                  const bool& use_unk_as_occ) const;

  void SetRobotState(const Pose& pose, double receive_time_seconds);
  bool UpdateFromLatestPose(const PointCloud& cloud);
  void updateMap(const PointCloud& cloud, const Pose& pose,
                 double receive_time_seconds);
  RobotState getRobotState() const;
  bool empty() const { return map_empty_; }
  VisualizationOutput BuildVisualization(
      const VisualizationRequest& request) const;

 private:
  static pcl::PointCloud<pcl::PointXYZ> ToPointCloud(
      const vec_E<Vec3f>& points);

  RobotState robot_state_;
};

}  // namespace rog_map
