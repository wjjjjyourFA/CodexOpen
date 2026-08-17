#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "modules/common_struct/basic_msgs/Pose.h"
#include "modules/common_struct/planning_msgs/OccupancyGrid.h"
#include "modules/common_struct/planning_msgs/Path.h"

namespace jojo {
namespace planning {

struct WorldPlannerConfig {
  double grid_resolution{0.4};
  double map_extra_margin{10.0};
  double robot_width{0.8};
  double robot_length{1.5};
  double inflation_margin{0.15};
  double obstacle_height_threshold{0.3};
  double path_point_spacing{0.5};
  double goal_reached_xy{0.3};
  double path_check_step{0.15};
  double reuse_search_radius{1.2};
  double goal_change_replan_distance{0.6};
  double heuristic_weight{1.0};
  std::string waypoint_file;
  std::string world_frame{"map"};
};

struct WorldPlannerOutput {
  bool ready{false};
  bool has_obstacle_grid{false};
  common_struct::Path path;
  common_struct::OccupancyGrid obstacle_grid;
};

// Pure planning core.  It owns no middleware handles and performs no publish,
// subscribe, parameter-server, logging-macro, or wall-clock calls.
class WorldPlanner {
 public:
  explicit WorldPlanner(const WorldPlannerConfig& config);

  void SetTerrain(const pcl::PointCloud<pcl::PointXYZI>& terrain);
  void SetOdometry(const common_struct::Pose& pose);
  void SetGoal(const common_struct::Pose& goal);
  WorldPlannerOutput Step(std::uint64_t timestamp_ns);

 private:
  struct GridSpec {
    double min_x{0.0};
    double min_y{0.0};
    int width{0};
    int height{0};
  };

  struct SearchNode {
    int index{0};
    double score{0.0};
    SearchNode() = default;
    SearchNode(int node_index, double node_score)
        : index(node_index), score(node_score) {}
    bool operator<(const SearchNode& other) const {
      return score > other.score;
    }
  };

  int ToIndex(int x, int y, int width) const;
  bool InBounds(int x, int y, int width, int height) const;
  double CellCenterX(const GridSpec& grid, int x) const;
  double CellCenterY(const GridSpec& grid, int y) const;
  bool WorldToCell(const GridSpec& grid, double world_x, double world_y,
                   int* cell_x, int* cell_y) const;
  void ResetCurrentPath(std::uint64_t timestamp_ns);
  void ReadWaypointBounds();
  GridSpec BuildRequiredMapSpec() const;
  void EnsureMapCoverage();
  void UpdateObstacleMapFromTerrain();
  bool FindNearestFreeCell(const GridSpec& grid,
                           const std::vector<std::uint8_t>& occupancy,
                           int* cell_x, int* cell_y) const;
  bool EstimateGoalCell(const GridSpec& grid,
                        const std::vector<std::uint8_t>& occupancy,
                        int start_x, int start_y, int* goal_x,
                        int* goal_y) const;
  bool RunAStar(const GridSpec& grid,
                const std::vector<std::uint8_t>& occupancy, int start_x,
                int start_y, int goal_x, int goal_y,
                std::vector<int>* parents) const;
  bool IsCellBlocked(const GridSpec& grid,
                     const std::vector<std::uint8_t>& occupancy,
                     double world_x, double world_y) const;
  bool IsSegmentBlocked(const GridSpec& grid,
                        const std::vector<std::uint8_t>& occupancy,
                        double x0, double y0, double x1, double y1) const;
  common_struct::Path BuildPath(const GridSpec& grid,
                                const std::vector<int>& parents, int goal_x,
                                int goal_y, std::uint64_t timestamp_ns) const;
  common_struct::Path BuildStraightPath(double end_x, double end_y,
                                        std::uint64_t timestamp_ns) const;
  common_struct::Path ShortcutPath(
      const GridSpec& grid, const std::vector<std::uint8_t>& occupancy,
      const common_struct::Path& raw_path) const;
  common_struct::Path SparsifyPath(
      const common_struct::Path& raw_path) const;
  bool GoalChangedMeaningfully() const;
  bool UpdatePathAnchorAndCheckConnectivity(
      const GridSpec& grid, const std::vector<std::uint8_t>& occupancy);
  common_struct::Path BuildDisplayPath(
      const GridSpec& grid, const std::vector<std::uint8_t>& occupancy,
      std::uint64_t timestamp_ns) const;
  common_struct::OccupancyGrid BuildOccupancyGrid(
      std::uint64_t timestamp_ns) const;
  common_struct::Header MakeHeader(std::uint64_t timestamp_ns) const;
  common_struct::PoseStamped MakePoseStamped(
      double x, double y, double z, const common_struct::Quaternion& orientation,
      std::uint64_t timestamp_ns) const;

  WorldPlannerConfig config_;
  bool has_pose_{false};
  bool has_goal_{false};
  bool has_terrain_{false};
  bool has_map_{false};
  bool has_waypoint_bounds_{false};
  double vehicle_x_{0.0};
  double vehicle_y_{0.0};
  common_struct::Pose goal_;
  pcl::PointCloud<pcl::PointXYZI> terrain_;

  double waypoint_min_x_{0.0};
  double waypoint_max_x_{0.0};
  double waypoint_min_y_{0.0};
  double waypoint_max_y_{0.0};

  GridSpec global_map_spec_;
  std::vector<std::uint8_t> global_obstacle_map_;
  common_struct::Path current_global_path_;
  std::size_t current_path_anchor_index_{0};
  double planned_goal_x_{std::numeric_limits<double>::quiet_NaN()};
  double planned_goal_y_{std::numeric_limits<double>::quiet_NaN()};
};

}  // namespace planning
}  // namespace jojo
