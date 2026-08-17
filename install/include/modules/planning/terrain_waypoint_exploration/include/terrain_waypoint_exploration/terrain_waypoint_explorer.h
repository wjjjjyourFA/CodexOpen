#pragma once

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "modules/common_struct/basic_msgs/Pose.h"

namespace terrain_waypoint_exploration {

struct TerrainWaypointExplorerConfig {
  double grid_resolution{0.20};
  double height_layer_resolution{0.10};
  int occupied_min_layers{4};
  int ground_max_layers{2};
  double ground_below_sensor{0.10};
  int scan_point_stride{4};
  double scan_max_range{20.0};
  double planning_frequency{2.0};
  double min_goal_distance{0.8};
  double reachable_search_radius{20.0};
  double waypoint_lookahead_distance{1.8};
  double goal_reached_distance{0.8};
  double goal_timeout{8.0};
  double goal_publish_frequency{5.0};
  double obstacle_clearance{0.45};
  double frontier_cluster_radius{0.8};
  int min_frontier_cluster_cells{3};
  double information_radius{3.0};
  double gain_weight{1.0};
  double distance_weight{2.0};
  double heading_weight{0.3};
  double continuation_angle_deg{65.0};
  double continuation_bonus{8.0};
  double continuation_target_radius{3.0};
  double saved_branch_match_radius{3.0};
  double saved_branch_merge_radius{1.5};
  double blacklist_radius{1.0};
  double blacklist_duration{20.0};
  int finish_no_frontier_cycles{10};
};

enum class ExplorerStatus : std::uint8_t {
  kWaitingForInputs = 0,
  kNoTraversableStart = 1,
  kActive = 2,
  kWaypointTimedOut = 3,
  kNoFrontier = 4,
  kFinished = 5
};

struct TerrainWaypointExplorerOutput {
  ExplorerStatus status{ExplorerStatus::kWaitingForInputs};
  bool goal_valid{false};
  bool publish_goal{false};
  bool exploration_finished{false};
  bool returning_to_saved_branch{false};
  std::size_t deferred_branch_count{0};
  jojo::common_struct::Pose goal;
  double selected_x{0.0};
  double selected_y{0.0};
  double selected_path_distance{0.0};
  pcl::PointCloud<pcl::PointXYZI> debug_map;
  pcl::PointCloud<pcl::PointXYZ> debug_frontiers;
  pcl::PointCloud<pcl::PointXYZI> debug_candidates;
};

// Frontier exploration core. It consumes native algorithm data and an
// explicit timestamp; middleware timers, messages, publishers, and logging
// are intentionally owned by adapters.
class TerrainWaypointExplorer {
 public:
  explicit TerrainWaypointExplorer(
      const TerrainWaypointExplorerConfig& config);

  void SetOdometry(const jojo::common_struct::Pose& pose);
  void AddRegisteredScan(
      double timestamp_seconds,
      const pcl::PointCloud<pcl::PointXYZ>& registered_scan);
  void AddTerrain(double timestamp_seconds,
                  const pcl::PointCloud<pcl::PointXYZ>& terrain);
  TerrainWaypointExplorerOutput Plan(double now_seconds);

  double planning_frequency() const { return planning_frequency_; }

 private:
  enum class CellState : std::uint8_t {
    UNKNOWN = 0,
    FREE = 1,
    OCCUPIED = 2
  };

  struct GridIndex {
    int x = 0;
    int y = 0;

    GridIndex() = default;
    GridIndex(int x_value, int y_value) : x(x_value), y(y_value) {}

    bool operator==(const GridIndex& other) const {
      return x == other.x && y == other.y;
    }
  };

  struct GridIndexHash {
    std::size_t operator()(const GridIndex& index) const {
      const std::uint64_t ux = static_cast<std::uint32_t>(index.x);
      const std::uint64_t uy = static_cast<std::uint32_t>(index.y);
      return std::hash<std::uint64_t>()((ux << 32U) | uy);
    }
  };

  struct Cell {
    CellState state = CellState::UNKNOWN;
    double ground_z = 0.0;
    double last_update_seconds = 0.0;
    std::uint32_t ground_observations = 0;
    std::uint32_t occupied_observations = 0;
  };

  struct HeightObservation {
    std::unordered_set<int> layers;
    double z_sum = 0.0;
    double max_z = -1.0e9;
    int point_count = 0;
  };

  struct FrontierCluster {
    std::vector<GridIndex> cells;
    GridIndex representative;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double distance = 0.0;
    double information_gain = 0.0;
    double score = -1.0e9;
  };

  struct SavedBranch {
    GridIndex anchor;
    double information_gain = 0.0;
    double last_seen_seconds = 0.0;
  };

  GridIndex worldToGrid(double x, double y) const;
  void gridToWorld(const GridIndex& index, double* x, double* y) const;
  CellState stateAt(const GridIndex& index) const;
  double groundHeightAt(const GridIndex& index, double fallback) const;
  void raycastFree(const GridIndex& start, const GridIndex& end,
                   double timestamp_seconds);
  bool obstacleNearby(const GridIndex& index) const;
  bool isTraversable(const GridIndex& index) const;
  bool isFrontier(const GridIndex& index) const;
  bool isBlacklisted(const GridIndex& index) const;
  double countUnknownAround(const GridIndex& index) const;

  bool findTraversableStart(GridIndex* start) const;
  void buildReachableRegion(
      const GridIndex& start,
      std::unordered_set<GridIndex, GridIndexHash>* reachable,
      std::unordered_map<GridIndex, GridIndex, GridIndexHash>* parent,
      std::unordered_map<GridIndex, int, GridIndexHash>* path_steps,
      double search_radius) const;
  std::vector<GridIndex> extractReachableFrontiers(
      const std::unordered_set<GridIndex, GridIndexHash>& reachable) const;
  std::vector<FrontierCluster> clusterFrontiers(
      const std::vector<GridIndex>& frontiers,
      const std::unordered_map<GridIndex, int, GridIndexHash>&
          path_steps) const;
  bool selectExplorationTarget(
      const std::vector<FrontierCluster>& clusters,
      FrontierCluster* selected);
  bool isContinuationCandidate(const FrontierCluster& cluster) const;
  void rememberDeferredBranches(
      const std::vector<FrontierCluster>& clusters,
      const FrontierCluster* selected);
  bool selectSavedBranch(const std::vector<FrontierCluster>& clusters,
                         FrontierCluster* selected);
  bool recoverPath(
      const GridIndex& start, const GridIndex& target,
      const std::unordered_map<GridIndex, GridIndex, GridIndexHash>& parent,
      std::vector<GridIndex>* path) const;
  bool selectTrackingWaypoint(const std::vector<GridIndex>& path,
                              GridIndex* waypoint) const;
  void updateTrackingGoal(const GridIndex& waypoint,
                          const GridIndex& exploration_target);
  void BuildDebug(const std::vector<GridIndex>& frontiers,
                  const std::vector<FrontierCluster>& clusters,
                  TerrainWaypointExplorerOutput* output) const;

  std::unordered_map<GridIndex, Cell, GridIndexHash> map_;
  std::unordered_map<GridIndex, double, GridIndexHash> blacklist_;
  std::unordered_map<GridIndex, SavedBranch, GridIndexHash> saved_branches_;

  bool has_odometry_ = false;
  bool has_scan_ = false;
  bool has_terrain_ = false;
  bool has_goal_ = false;
  bool has_active_direction_ = false;
  double robot_x_ = 0.0;
  double robot_y_ = 0.0;
  double robot_z_ = 0.0;
  double robot_yaw_ = 0.0;
  jojo::common_struct::Pose current_goal_;
  GridIndex current_goal_index_;
  GridIndex active_target_index_;
  double active_direction_x_ = 0.0;
  double active_direction_y_ = 0.0;
  double goal_created_time_seconds_ = 0.0;
  double last_goal_publish_time_seconds_ = 0.0;
  double current_time_seconds_ = 0.0;
  int no_frontier_cycles_ = 0;

  double grid_resolution_ = 0.20;
  double height_layer_resolution_ = 0.10;
  int occupied_min_layers_ = 4;
  int ground_max_layers_ = 2;
  double ground_below_sensor_ = 0.10;
  int scan_point_stride_ = 4;
  double scan_max_range_ = 20.0;
  double planning_frequency_ = 2.0;
  double min_goal_distance_ = 0.8;
  double reachable_search_radius_ = 20.0;
  double waypoint_lookahead_distance_ = 1.8;
  double goal_reached_distance_ = 0.8;
  double goal_timeout_ = 8.0;
  double goal_publish_frequency_ = 5.0;
  double obstacle_clearance_ = 0.45;
  double frontier_cluster_radius_ = 0.8;
  int min_frontier_cluster_cells_ = 3;
  double information_radius_ = 3.0;
  double gain_weight_ = 1.0;
  double distance_weight_ = 2.0;
  double heading_weight_ = 0.3;
  double continuation_angle_deg_ = 65.0;
  double continuation_bonus_ = 8.0;
  double continuation_target_radius_ = 3.0;
  double saved_branch_match_radius_ = 3.0;
  double saved_branch_merge_radius_ = 1.5;
  double blacklist_radius_ = 1.0;
  double blacklist_duration_ = 20.0;
  int finish_no_frontier_cycles_ = 10;
};

}  // namespace terrain_waypoint_exploration
