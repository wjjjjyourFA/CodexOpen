#pragma once

#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>

namespace terrain_waypoint_exploration {

class TerrainWaypointExplorer {
 public:
  TerrainWaypointExplorer(const ros::NodeHandle& nh,
                          const ros::NodeHandle& pnh);

 private:
  enum class CellState : uint8_t {
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
      const uint64_t ux = static_cast<uint32_t>(index.x);
      const uint64_t uy = static_cast<uint32_t>(index.y);
      return std::hash<uint64_t>()((ux << 32U) | uy);
    }
  };

  struct Cell {
    CellState state = CellState::UNKNOWN;
    double ground_z = 0.0;
    ros::Time last_update;
    uint32_t ground_observations = 0;
    uint32_t occupied_observations = 0;
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
    ros::Time last_seen;
  };

  void loadParameters();
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
  void scanCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
  void terrainCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
  void planningTimerCallback(const ros::TimerEvent&);

  GridIndex worldToGrid(double x, double y) const;
  void gridToWorld(const GridIndex& index, double* x, double* y) const;
  CellState stateAt(const GridIndex& index) const;
  double groundHeightAt(const GridIndex& index, double fallback) const;
  void raycastFree(const GridIndex& start, const GridIndex& end,
                   const ros::Time& stamp);
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
  void publishGoal();
  void publishGoalValidity(bool valid);
  void publishDebug(const std::vector<GridIndex>& frontiers,
                    const std::vector<FrontierCluster>& clusters);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber odometry_sub_;
  ros::Subscriber scan_sub_;
  ros::Subscriber terrain_sub_;
  ros::Publisher waypoint_pub_;
  ros::Publisher goal_valid_pub_;
  ros::Publisher map_pub_;
  ros::Publisher frontier_pub_;
  ros::Publisher candidate_pub_;
  ros::Publisher goal_marker_pub_;
  ros::Publisher finished_pub_;
  ros::Timer planning_timer_;

  mutable std::mutex mutex_;
  std::unordered_map<GridIndex, Cell, GridIndexHash> map_;
  std::unordered_map<GridIndex, ros::Time, GridIndexHash> blacklist_;
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
  geometry_msgs::PoseStamped current_goal_;
  GridIndex current_goal_index_;
  GridIndex active_target_index_;
  double active_direction_x_ = 0.0;
  double active_direction_y_ = 0.0;
  ros::Time goal_created_time_;
  ros::Time last_goal_publish_time_;
  int no_frontier_cycles_ = 0;

  std::string world_frame_;
  std::string odometry_topic_;
  std::string registered_scan_topic_;
  std::string terrain_topic_;
  std::string waypoint_topic_;
  std::string goal_valid_topic_;
  std::string finished_topic_;

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
