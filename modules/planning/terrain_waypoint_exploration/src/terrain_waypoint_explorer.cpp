#include "terrain_waypoint_exploration/terrain_waypoint_explorer.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_datatypes.h>

namespace terrain_waypoint_exploration {
namespace {

constexpr double kPi = 3.14159265358979323846;

double normalizeAngle(double angle) {
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

}  // namespace

TerrainWaypointExplorer::TerrainWaypointExplorer(
    const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh) {
  loadParameters();

  odometry_sub_ = nh_.subscribe(
      odometry_topic_, 20, &TerrainWaypointExplorer::odometryCallback, this);
  scan_sub_ = nh_.subscribe(
      registered_scan_topic_, 2, &TerrainWaypointExplorer::scanCallback, this);
  terrain_sub_ = nh_.subscribe(
      terrain_topic_, 2, &TerrainWaypointExplorer::terrainCallback, this);

  waypoint_pub_ =
      nh_.advertise<geometry_msgs::PoseStamped>(waypoint_topic_, 2, true);
  goal_valid_pub_ =
      nh_.advertise<std_msgs::Bool>(goal_valid_topic_, 5, true);
  map_pub_ =
      pnh_.advertise<sensor_msgs::PointCloud2>("debug_map", 1, true);
  frontier_pub_ =
      pnh_.advertise<sensor_msgs::PointCloud2>("debug_frontiers", 1, true);
  candidate_pub_ =
      pnh_.advertise<sensor_msgs::PointCloud2>("debug_candidates", 1, true);
  goal_marker_pub_ =
      pnh_.advertise<visualization_msgs::Marker>("debug_goal", 1, true);
  finished_pub_ = nh_.advertise<std_msgs::Bool>(finished_topic_, 1, true);

  planning_timer_ = nh_.createTimer(
      ros::Duration(1.0 / std::max(0.1, planning_frequency_)),
      &TerrainWaypointExplorer::planningTimerCallback, this);

  // Latching the initial invalid state prevents a downstream planner from
  // treating a stale waypoint from an earlier node instance as active.
  publishGoalValidity(false);

  ROS_INFO_STREAM("terrain_waypoint_exploration started. Inputs: "
                  << odometry_topic_ << ", " << registered_scan_topic_ << ", "
                  << terrain_topic_ << "; output: " << waypoint_topic_);
}

void TerrainWaypointExplorer::loadParameters() {
  pnh_.param("world_frame", world_frame_, std::string("map"));
  pnh_.param("odometry_topic", odometry_topic_,
             std::string("/state_estimation"));
  pnh_.param("registered_scan_topic", registered_scan_topic_,
             std::string("/registered_scan"));
  pnh_.param("terrain_topic", terrain_topic_,
             std::string("/terrain_map"));
  pnh_.param("waypoint_topic", waypoint_topic_,
             std::string("/way_point"));
  pnh_.param("goal_valid_topic", goal_valid_topic_,
             std::string("/isgoal_vaild"));
  pnh_.param("finished_topic", finished_topic_,
             std::string("/exploration_finished"));

  pnh_.param("grid_resolution", grid_resolution_, grid_resolution_);
  pnh_.param("height_layer_resolution", height_layer_resolution_,
             height_layer_resolution_);
  pnh_.param("occupied_min_layers", occupied_min_layers_,
             occupied_min_layers_);
  pnh_.param("ground_max_layers", ground_max_layers_, ground_max_layers_);
  pnh_.param("ground_below_sensor", ground_below_sensor_,
             ground_below_sensor_);
  pnh_.param("scan_point_stride", scan_point_stride_, scan_point_stride_);
  pnh_.param("scan_max_range", scan_max_range_, scan_max_range_);

  pnh_.param("planning_frequency", planning_frequency_, planning_frequency_);
  pnh_.param("min_goal_distance", min_goal_distance_, min_goal_distance_);
  pnh_.param("reachable_search_radius", reachable_search_radius_,
             reachable_search_radius_);
  pnh_.param("waypoint_lookahead_distance", waypoint_lookahead_distance_,
             waypoint_lookahead_distance_);
  pnh_.param("goal_reached_distance", goal_reached_distance_,
             goal_reached_distance_);
  pnh_.param("goal_timeout", goal_timeout_, goal_timeout_);
  pnh_.param("goal_publish_frequency", goal_publish_frequency_,
             goal_publish_frequency_);
  pnh_.param("obstacle_clearance", obstacle_clearance_, obstacle_clearance_);
  pnh_.param("frontier_cluster_radius", frontier_cluster_radius_,
             frontier_cluster_radius_);
  pnh_.param("min_frontier_cluster_cells", min_frontier_cluster_cells_,
             min_frontier_cluster_cells_);
  pnh_.param("information_radius", information_radius_, information_radius_);
  pnh_.param("gain_weight", gain_weight_, gain_weight_);
  pnh_.param("distance_weight", distance_weight_, distance_weight_);
  pnh_.param("heading_weight", heading_weight_, heading_weight_);
  pnh_.param("continuation_angle_deg", continuation_angle_deg_,
             continuation_angle_deg_);
  pnh_.param("continuation_bonus", continuation_bonus_,
             continuation_bonus_);
  pnh_.param("continuation_target_radius", continuation_target_radius_,
             continuation_target_radius_);
  pnh_.param("saved_branch_match_radius", saved_branch_match_radius_,
             saved_branch_match_radius_);
  pnh_.param("saved_branch_merge_radius", saved_branch_merge_radius_,
             saved_branch_merge_radius_);
  pnh_.param("blacklist_radius", blacklist_radius_, blacklist_radius_);
  pnh_.param("blacklist_duration", blacklist_duration_, blacklist_duration_);
  pnh_.param("finish_no_frontier_cycles", finish_no_frontier_cycles_,
             finish_no_frontier_cycles_);
}

void TerrainWaypointExplorer::odometryCallback(
    const nav_msgs::OdometryConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  robot_z_ = msg->pose.pose.position.z;
  robot_yaw_ = tf::getYaw(msg->pose.pose.orientation);
  has_odometry_ = true;
}

void TerrainWaypointExplorer::scanCallback(
    const sensor_msgs::PointCloud2ConstPtr& msg) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg, cloud);

  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_odometry_) {
    return;
  }

  const GridIndex start = worldToGrid(robot_x_, robot_y_);
  const int stride = std::max(1, scan_point_stride_);
  for (std::size_t i = 0; i < cloud.size(); i += stride) {
    const pcl::PointXYZ& point = cloud.points[i];
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }

    const double dx = point.x - robot_x_;
    const double dy = point.y - robot_y_;
    const double range = std::hypot(dx, dy);
    if (range < grid_resolution_ || range > scan_max_range_) {
      continue;
    }
    raycastFree(start, worldToGrid(point.x, point.y), msg->header.stamp);
  }
  has_scan_ = true;
}

void TerrainWaypointExplorer::terrainCallback(
    const sensor_msgs::PointCloud2ConstPtr& msg) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg, cloud);

  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_odometry_) {
    return;
  }

  std::unordered_map<GridIndex, HeightObservation, GridIndexHash> observations;
  for (const pcl::PointXYZ& point : cloud.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }
    GridIndex index = worldToGrid(point.x, point.y);
    HeightObservation& observation = observations[index];
    const int layer =
        static_cast<int>(std::floor(point.z / height_layer_resolution_));
    observation.layers.insert(layer);
    observation.z_sum += point.z;
    observation.max_z = std::max(observation.max_z,
                                 static_cast<double>(point.z));
    ++observation.point_count;
  }

  for (const auto& entry : observations) {
    const GridIndex& index = entry.first;
    const HeightObservation& observation = entry.second;
    const int layer_count = static_cast<int>(observation.layers.size());
    Cell& cell = map_[index];

    const bool ground =
        layer_count <= ground_max_layers_ &&
        observation.max_z < robot_z_ - ground_below_sensor_;
    const bool occupied = layer_count >= occupied_min_layers_;

    if (ground) {
      // A later, compact low ground observation deliberately clears an older
      // occupied state. This is the dynamic-trail removal rule requested for
      // moving objects.
      cell.state = CellState::FREE;
      cell.ground_z =
          observation.z_sum / std::max(1, observation.point_count);
      ++cell.ground_observations;
      cell.last_update = msg->header.stamp;
    } else if (occupied) {
      cell.state = CellState::OCCUPIED;
      ++cell.occupied_observations;
      cell.last_update = msg->header.stamp;
    }
  }
  has_terrain_ = true;
}

TerrainWaypointExplorer::GridIndex
TerrainWaypointExplorer::worldToGrid(double x, double y) const {
  GridIndex index;
  index.x = static_cast<int>(std::floor(x / grid_resolution_));
  index.y = static_cast<int>(std::floor(y / grid_resolution_));
  return index;
}

void TerrainWaypointExplorer::gridToWorld(
    const GridIndex& index, double* x, double* y) const {
  *x = (static_cast<double>(index.x) + 0.5) * grid_resolution_;
  *y = (static_cast<double>(index.y) + 0.5) * grid_resolution_;
}

TerrainWaypointExplorer::CellState TerrainWaypointExplorer::stateAt(
    const GridIndex& index) const {
  const auto it = map_.find(index);
  return it == map_.end() ? CellState::UNKNOWN : it->second.state;
}

double TerrainWaypointExplorer::groundHeightAt(
    const GridIndex& index, double fallback) const {
  const auto it = map_.find(index);
  if (it == map_.end() || it->second.state != CellState::FREE ||
      it->second.ground_observations == 0) {
    return fallback;
  }
  return it->second.ground_z;
}

void TerrainWaypointExplorer::raycastFree(
    const GridIndex& start, const GridIndex& end, const ros::Time& stamp) {
  int x0 = start.x;
  int y0 = start.y;
  const int x1 = end.x;
  const int y1 = end.y;
  const int dx = std::abs(x1 - x0);
  const int sx = x0 < x1 ? 1 : -1;
  const int dy = -std::abs(y1 - y0);
  const int sy = y0 < y1 ? 1 : -1;
  int error = dx + dy;

  while (!(x0 == x1 && y0 == y1)) {
    GridIndex index{x0, y0};
    if (!(index == start) &&
        stateAt(index) == CellState::OCCUPIED) {
      // Never carve free space through an already confirmed wall/obstacle.
      // This also prevents frontiers from being created behind that wall.
      break;
    }
    Cell& cell = map_[index];
    // Ray tracing expands observed free space but never clears a terrain
    // obstacle. Only the explicit low-ground rule in terrainCallback can do
    // that.
    if (cell.state == CellState::UNKNOWN) {
      cell.state = CellState::FREE;
      cell.ground_z = robot_z_ - ground_below_sensor_;
      cell.last_update = stamp;
    }
    const int twice_error = 2 * error;
    if (twice_error >= dy) {
      error += dy;
      x0 += sx;
    }
    if (twice_error <= dx) {
      error += dx;
      y0 += sy;
    }
  }
}

bool TerrainWaypointExplorer::obstacleNearby(
    const GridIndex& index) const {
  const int radius =
      static_cast<int>(std::ceil(obstacle_clearance_ / grid_resolution_));
  for (int dx = -radius; dx <= radius; ++dx) {
    for (int dy = -radius; dy <= radius; ++dy) {
      if (dx * dx + dy * dy > radius * radius) {
        continue;
      }
      if (stateAt(GridIndex{index.x + dx, index.y + dy}) ==
          CellState::OCCUPIED) {
        return true;
      }
    }
  }
  return false;
}

bool TerrainWaypointExplorer::isTraversable(
    const GridIndex& index) const {
  return stateAt(index) == CellState::FREE && !obstacleNearby(index);
}

bool TerrainWaypointExplorer::isFrontier(
    const GridIndex& index) const {
  if (!isTraversable(index)) {
    return false;
  }
  static const int kNeighbors[8][2] = {
      {-1, -1}, {-1, 0}, {-1, 1}, {0, -1},
      {0, 1},   {1, -1}, {1, 0},  {1, 1}};
  for (const auto& offset : kNeighbors) {
    if (stateAt(GridIndex{index.x + offset[0], index.y + offset[1]}) ==
        CellState::UNKNOWN) {
      return true;
    }
  }
  return false;
}

bool TerrainWaypointExplorer::isBlacklisted(
    const GridIndex& index) const {
  const ros::Time now = ros::Time::now();
  const int radius =
      static_cast<int>(std::ceil(blacklist_radius_ / grid_resolution_));
  for (const auto& entry : blacklist_) {
    if ((now - entry.second).toSec() > blacklist_duration_) {
      continue;
    }
    const int dx = entry.first.x - index.x;
    const int dy = entry.first.y - index.y;
    if (dx * dx + dy * dy <= radius * radius) {
      return true;
    }
  }
  return false;
}

double TerrainWaypointExplorer::countUnknownAround(
    const GridIndex& index) const {
  const int radius =
      static_cast<int>(std::ceil(information_radius_ / grid_resolution_));
  double unknown_count = 0.0;
  for (int dx = -radius; dx <= radius; ++dx) {
    for (int dy = -radius; dy <= radius; ++dy) {
      if (dx * dx + dy * dy > radius * radius) {
        continue;
      }
      if (stateAt(GridIndex{index.x + dx, index.y + dy}) ==
          CellState::UNKNOWN) {
        unknown_count += 1.0;
      }
    }
  }
  return unknown_count;
}

std::vector<TerrainWaypointExplorer::GridIndex>
TerrainWaypointExplorer::extractReachableFrontiers(
    const std::unordered_set<GridIndex, GridIndexHash>& reachable) const {
  std::vector<GridIndex> frontiers;
  frontiers.reserve(reachable.size() / 8U + 1U);
  for (const GridIndex& index : reachable) {
    if (isFrontier(index) && !isBlacklisted(index)) {
      frontiers.push_back(index);
    }
  }
  return frontiers;
}

bool TerrainWaypointExplorer::findTraversableStart(
    GridIndex* start) const {
  const GridIndex robot = worldToGrid(robot_x_, robot_y_);
  if (isTraversable(robot)) {
    *start = robot;
    return true;
  }

  const int search_radius =
      std::max(1, static_cast<int>(std::ceil(1.0 / grid_resolution_)));
  double best_distance = std::numeric_limits<double>::max();
  bool found = false;
  for (int dx = -search_radius; dx <= search_radius; ++dx) {
    for (int dy = -search_radius; dy <= search_radius; ++dy) {
      const GridIndex candidate{robot.x + dx, robot.y + dy};
      if (!isTraversable(candidate)) {
        continue;
      }
      const double distance = std::hypot(dx, dy);
      if (distance < best_distance) {
        best_distance = distance;
        *start = candidate;
        found = true;
      }
    }
  }
  return found;
}

void TerrainWaypointExplorer::buildReachableRegion(
    const GridIndex& start,
    std::unordered_set<GridIndex, GridIndexHash>* reachable,
    std::unordered_map<GridIndex, GridIndex, GridIndexHash>* parent,
    std::unordered_map<GridIndex, int, GridIndexHash>* path_steps,
    double search_radius) const {
  reachable->clear();
  parent->clear();
  path_steps->clear();

  std::queue<GridIndex> pending;
  pending.push(start);
  reachable->insert(start);
  (*parent)[start] = start;
  (*path_steps)[start] = 0;

  const int max_steps = search_radius > 0.0
                            ? std::max(1, static_cast<int>(std::ceil(
                                              search_radius /
                                              grid_resolution_)))
                            : std::numeric_limits<int>::max();
  static const int kNeighbors[8][2] = {
      {-1, -1}, {-1, 0}, {-1, 1}, {0, -1},
      {0, 1},   {1, -1}, {1, 0},  {1, 1}};

  while (!pending.empty()) {
    const GridIndex current = pending.front();
    pending.pop();
    const int current_steps = path_steps->at(current);
    if (current_steps >= max_steps) {
      continue;
    }

    for (const auto& offset : kNeighbors) {
      const GridIndex next{current.x + offset[0],
                           current.y + offset[1]};
      if (reachable->count(next) != 0U || !isTraversable(next)) {
        continue;
      }

      // Prevent diagonal corner cutting. A point behind a wall must not become
      // reachable through the diagonal gap between two occupied cells.
      if (offset[0] != 0 && offset[1] != 0) {
        const GridIndex side_x{current.x + offset[0], current.y};
        const GridIndex side_y{current.x, current.y + offset[1]};
        if (!isTraversable(side_x) || !isTraversable(side_y)) {
          continue;
        }
      }

      reachable->insert(next);
      (*parent)[next] = current;
      (*path_steps)[next] = current_steps + 1;
      pending.push(next);
    }
  }
}

std::vector<TerrainWaypointExplorer::FrontierCluster>
TerrainWaypointExplorer::clusterFrontiers(
    const std::vector<GridIndex>& frontiers,
    const std::unordered_map<GridIndex, int, GridIndexHash>&
        path_steps) const {
  std::vector<FrontierCluster> clusters;
  if (frontiers.empty()) {
    return clusters;
  }

  std::unordered_set<GridIndex, GridIndexHash> frontier_set(
      frontiers.begin(), frontiers.end());
  std::unordered_set<GridIndex, GridIndexHash> visited;
  const int cluster_radius =
      std::max(1, static_cast<int>(
          std::ceil(frontier_cluster_radius_ / grid_resolution_)));

  for (const GridIndex& seed : frontiers) {
    if (visited.count(seed) != 0U) {
      continue;
    }

    FrontierCluster cluster;
    std::queue<GridIndex> pending;
    pending.push(seed);
    visited.insert(seed);

    while (!pending.empty()) {
      const GridIndex current = pending.front();
      pending.pop();
      cluster.cells.push_back(current);

      for (int dx = -cluster_radius; dx <= cluster_radius; ++dx) {
        for (int dy = -cluster_radius; dy <= cluster_radius; ++dy) {
          if (dx == 0 && dy == 0) {
            continue;
          }
          if (dx * dx + dy * dy > cluster_radius * cluster_radius) {
            continue;
          }
          const GridIndex neighbor{current.x + dx, current.y + dy};
          if (frontier_set.count(neighbor) != 0U &&
              visited.insert(neighbor).second) {
            pending.push(neighbor);
          }
        }
      }
    }

    if (static_cast<int>(cluster.cells.size()) <
        min_frontier_cluster_cells_) {
      continue;
    }

    double mean_x = 0.0;
    double mean_y = 0.0;
    for (const GridIndex& index : cluster.cells) {
      double x = 0.0;
      double y = 0.0;
      gridToWorld(index, &x, &y);
      mean_x += x;
      mean_y += y;
    }
    mean_x /= cluster.cells.size();
    mean_y /= cluster.cells.size();

    double best_cost = std::numeric_limits<double>::max();
    for (const GridIndex& index : cluster.cells) {
      double x = 0.0;
      double y = 0.0;
      gridToWorld(index, &x, &y);
      const double center_distance = std::hypot(x - mean_x, y - mean_y);
      const auto step_it = path_steps.find(index);
      if (step_it == path_steps.end()) {
        continue;
      }
      const double path_distance =
          step_it->second * grid_resolution_;
      const double cost = center_distance + 0.15 * path_distance;
      if (cost < best_cost && isTraversable(index)) {
        best_cost = cost;
        cluster.representative = index;
        cluster.x = x;
        cluster.y = y;
        cluster.distance = path_distance;
      }
    }
    if (!std::isfinite(best_cost)) {
      continue;
    }
    cluster.z =
        groundHeightAt(cluster.representative, robot_z_ - ground_below_sensor_);
    cluster.information_gain =
        countUnknownAround(cluster.representative);
    clusters.push_back(cluster);
  }
  return clusters;
}

bool TerrainWaypointExplorer::selectExplorationTarget(
    const std::vector<FrontierCluster>& clusters,
    FrontierCluster* selected) {
  bool continuation_available = false;
  if (has_active_direction_) {
    for (const FrontierCluster& cluster : clusters) {
      if (cluster.distance >= min_goal_distance_ &&
          isContinuationCandidate(cluster)) {
        continuation_available = true;
        break;
      }
    }
  }

  // Finishing a corridor is a state transition. Do not immediately choose an
  // arbitrary nearby frontier: the saved branch pool must decide where to
  // backtrack next.
  if (has_active_direction_ && !continuation_available) {
    return false;
  }

  bool found = false;
  double best_score = -std::numeric_limits<double>::max();
  for (FrontierCluster cluster : clusters) {
    if (cluster.distance < min_goal_distance_) {
      continue;
    }
    const bool continuation = isContinuationCandidate(cluster);
    if (continuation_available && !continuation) {
      continue;
    }

    const double bearing =
        std::atan2(cluster.y - robot_y_, cluster.x - robot_x_);
    const double heading_error =
        std::abs(normalizeAngle(bearing - robot_yaw_));

    // Geodesic distance is deliberately dominant. Information gain only
    // breaks ties between nearby reachable frontiers.
    cluster.score =
        gain_weight_ * std::log1p(cluster.information_gain) -
        distance_weight_ * cluster.distance -
        heading_weight_ * heading_error +
        (continuation ? continuation_bonus_ : 0.0);

    if (cluster.score > best_score) {
      best_score = cluster.score;
      *selected = cluster;
      found = true;
    }
  }
  return found;
}

bool TerrainWaypointExplorer::isContinuationCandidate(
    const FrontierCluster& cluster) const {
  if (!has_active_direction_) {
    return false;
  }
  const double direction_norm =
      std::hypot(active_direction_x_, active_direction_y_);
  const double candidate_x = cluster.x - robot_x_;
  const double candidate_y = cluster.y - robot_y_;
  const double candidate_norm = std::hypot(candidate_x, candidate_y);
  if (direction_norm <= 1.0e-3 || candidate_norm <= 1.0e-3) {
    return false;
  }
  const double cosine = std::max(
      -1.0, std::min(1.0,
                     (active_direction_x_ * candidate_x +
                      active_direction_y_ * candidate_y) /
                         (direction_norm * candidate_norm)));
  const double direction_error = std::acos(cosine) * 180.0 / kPi;
  double target_x = 0.0;
  double target_y = 0.0;
  gridToWorld(active_target_index_, &target_x, &target_y);
  return direction_error <= continuation_angle_deg_ ||
         std::hypot(cluster.x - target_x, cluster.y - target_y) <=
             continuation_target_radius_;
}

void TerrainWaypointExplorer::rememberDeferredBranches(
    const std::vector<FrontierCluster>& clusters,
    const FrontierCluster* selected) {
  const int merge_cells = std::max(
      1, static_cast<int>(std::ceil(saved_branch_merge_radius_ /
                                    grid_resolution_)));

  for (const FrontierCluster& cluster : clusters) {
    // Distinct clusters are distinct branches, even when a Y junction has a
    // shallow angle. Only the selected cluster (and fragments immediately
    // around it) are excluded. If the corridor has just ended without a
    // selection, exclude its forward continuation using the direction lock.
    const bool same_corridor =
        selected != nullptr
            ? std::hypot(cluster.x - selected->x,
                         cluster.y - selected->y) <=
                  continuation_target_radius_
            : isContinuationCandidate(cluster);
    if (same_corridor || cluster.distance < min_goal_distance_) {
      continue;
    }

    GridIndex existing_key;
    bool has_existing = false;
    for (const auto& entry : saved_branches_) {
      const int dx = entry.first.x - cluster.representative.x;
      const int dy = entry.first.y - cluster.representative.y;
      if (dx * dx + dy * dy <= merge_cells * merge_cells) {
        existing_key = entry.first;
        has_existing = true;
        break;
      }
    }
    if (has_existing) {
      SavedBranch& branch = saved_branches_[existing_key];
      branch.information_gain = std::max(branch.information_gain,
                                         cluster.information_gain);
      branch.last_seen = ros::Time::now();
    } else {
      SavedBranch branch;
      branch.anchor = cluster.representative;
      branch.information_gain = cluster.information_gain;
      branch.last_seen = ros::Time::now();
      saved_branches_[branch.anchor] = branch;
    }
  }
}

bool TerrainWaypointExplorer::selectSavedBranch(
    const std::vector<FrontierCluster>& clusters,
    FrontierCluster* selected) {
  const double match_radius = std::max(saved_branch_match_radius_,
                                       saved_branch_merge_radius_);
  bool found = false;
  double best_score = -std::numeric_limits<double>::max();

  for (auto it = saved_branches_.begin(); it != saved_branches_.end();) {
    double anchor_x = 0.0;
    double anchor_y = 0.0;
    gridToWorld(it->second.anchor, &anchor_x, &anchor_y);
    const FrontierCluster* match = nullptr;
    double best_match_distance = match_radius;
    for (const FrontierCluster& cluster : clusters) {
      const double distance =
          std::hypot(cluster.x - anchor_x, cluster.y - anchor_y);
      if (distance <= best_match_distance &&
          cluster.distance >= min_goal_distance_) {
        best_match_distance = distance;
        match = &cluster;
      }
    }
    if (match == nullptr) {
      // No frontier remains near this anchor: it has already been observed,
      // became unreachable, or was invalidated by an obstacle update.
      it = saved_branches_.erase(it);
      continue;
    }

    const double score =
        gain_weight_ * std::log1p(match->information_gain) -
        distance_weight_ * match->distance;
    if (score > best_score) {
      best_score = score;
      *selected = *match;
      found = true;
    }
    ++it;
  }
  // Keep the chosen branch until the robot reaches it. The global path is
  // intentionally emitted as short tracking waypoints, so removing it here
  // would lose a far-away branch after the first return step. It is erased by
  // the no-match path above once its frontier is observed or invalidated.
  return found;
}

bool TerrainWaypointExplorer::recoverPath(
    const GridIndex& start, const GridIndex& target,
    const std::unordered_map<GridIndex, GridIndex, GridIndexHash>& parent,
    std::vector<GridIndex>* path) const {
  path->clear();
  if (parent.find(target) == parent.end()) {
    return false;
  }
  GridIndex current = target;
  path->push_back(current);
  while (!(current == start)) {
    const auto it = parent.find(current);
    if (it == parent.end() || it->second == current) {
      return false;
    }
    current = it->second;
    path->push_back(current);
  }
  std::reverse(path->begin(), path->end());
  return true;
}

bool TerrainWaypointExplorer::selectTrackingWaypoint(
    const std::vector<GridIndex>& path, GridIndex* waypoint) const {
  if (path.empty()) {
    return false;
  }
  const int lookahead_steps = std::max(
      1, static_cast<int>(std::ceil(waypoint_lookahead_distance_ /
                                    grid_resolution_)));
  const std::size_t index = std::min(
      path.size() - 1U, static_cast<std::size_t>(lookahead_steps));
  *waypoint = path[index];
  return isTraversable(*waypoint);
}

void TerrainWaypointExplorer::updateTrackingGoal(
    const GridIndex& waypoint, const GridIndex& exploration_target) {
  double waypoint_x = 0.0;
  double waypoint_y = 0.0;
  gridToWorld(waypoint, &waypoint_x, &waypoint_y);

  const bool waypoint_changed =
      !has_goal_ || !(current_goal_index_ == waypoint);
  current_goal_.header.frame_id = world_frame_;
  current_goal_.header.stamp = ros::Time::now();
  current_goal_.pose.position.x = waypoint_x;
  current_goal_.pose.position.y = waypoint_y;
  current_goal_.pose.position.z =
      groundHeightAt(waypoint, robot_z_ - ground_below_sensor_);
  const double yaw = std::atan2(waypoint_y - robot_y_,
                                waypoint_x - robot_x_);
  current_goal_.pose.orientation =
      tf::createQuaternionMsgFromYaw(yaw);
  current_goal_index_ = waypoint;
  active_target_index_ = exploration_target;
  active_direction_x_ =
      (exploration_target.x + 0.5) * grid_resolution_ - robot_x_;
  active_direction_y_ =
      (exploration_target.y + 0.5) * grid_resolution_ - robot_y_;
  has_active_direction_ = true;
  if (waypoint_changed) {
    goal_created_time_ = ros::Time::now();
  }
  has_goal_ = true;
}

void TerrainWaypointExplorer::publishGoal() {
  if (!has_goal_) {
    return;
  }
  const ros::Time now = ros::Time::now();
  if (!last_goal_publish_time_.isZero() &&
      (now - last_goal_publish_time_).toSec() <
          1.0 / std::max(0.1, goal_publish_frequency_)) {
    return;
  }
  current_goal_.header.stamp = now;
  waypoint_pub_.publish(current_goal_);
  last_goal_publish_time_ = now;
}

void TerrainWaypointExplorer::publishGoalValidity(bool valid) {
  std_msgs::Bool message;
  message.data = valid;
  goal_valid_pub_.publish(message);
}

void TerrainWaypointExplorer::publishDebug(
    const std::vector<GridIndex>& frontiers,
    const std::vector<FrontierCluster>& clusters) {
  pcl::PointCloud<pcl::PointXYZI> map_cloud;
  map_cloud.reserve(map_.size());
  for (const auto& entry : map_) {
    if (entry.second.state == CellState::UNKNOWN) {
      continue;
    }
    pcl::PointXYZI point;
    double x = 0.0;
    double y = 0.0;
    gridToWorld(entry.first, &x, &y);
    point.x = static_cast<float>(x);
    point.y = static_cast<float>(y);
    point.z = static_cast<float>(
        groundHeightAt(entry.first, robot_z_ - ground_below_sensor_));
    point.intensity =
        entry.second.state == CellState::FREE ? 1.0F : 100.0F;
    map_cloud.push_back(point);
  }

  pcl::PointCloud<pcl::PointXYZ> frontier_cloud;
  frontier_cloud.reserve(frontiers.size());
  for (const GridIndex& index : frontiers) {
    pcl::PointXYZ point;
    double x = 0.0;
    double y = 0.0;
    gridToWorld(index, &x, &y);
    point.x = static_cast<float>(x);
    point.y = static_cast<float>(y);
    point.z = static_cast<float>(
        groundHeightAt(index, robot_z_ - ground_below_sensor_));
    frontier_cloud.push_back(point);
  }

  pcl::PointCloud<pcl::PointXYZI> candidate_cloud;
  candidate_cloud.reserve(clusters.size());
  for (const FrontierCluster& cluster : clusters) {
    pcl::PointXYZI point;
    point.x = static_cast<float>(cluster.x);
    point.y = static_cast<float>(cluster.y);
    point.z = static_cast<float>(cluster.z);
    point.intensity = static_cast<float>(cluster.information_gain);
    candidate_cloud.push_back(point);
  }

  sensor_msgs::PointCloud2 map_msg;
  pcl::toROSMsg(map_cloud, map_msg);
  map_msg.header.frame_id = world_frame_;
  map_msg.header.stamp = ros::Time::now();
  map_pub_.publish(map_msg);

  sensor_msgs::PointCloud2 frontier_msg;
  pcl::toROSMsg(frontier_cloud, frontier_msg);
  frontier_msg.header = map_msg.header;
  frontier_pub_.publish(frontier_msg);

  sensor_msgs::PointCloud2 candidate_msg;
  pcl::toROSMsg(candidate_cloud, candidate_msg);
  candidate_msg.header = map_msg.header;
  candidate_pub_.publish(candidate_msg);

  visualization_msgs::Marker marker;
  marker.header = map_msg.header;
  marker.ns = "terrain_waypoint_exploration";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = has_goal_ ? visualization_msgs::Marker::ADD
                            : visualization_msgs::Marker::DELETE;
  marker.pose.orientation.w = 1.0;
  marker.pose.position = current_goal_.pose.position;
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;
  marker.color.r = 1.0;
  marker.color.g = 0.2;
  marker.color.b = 0.1;
  marker.color.a = 1.0;
  goal_marker_pub_.publish(marker);
}

void TerrainWaypointExplorer::planningTimerCallback(
    const ros::TimerEvent&) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_odometry_ || !has_scan_ || !has_terrain_) {
    has_goal_ = false;
    publishGoalValidity(false);
    ROS_WARN_THROTTLE(
        5.0, "Waiting for odometry, registered scan, and terrain map.");
    return;
  }

  GridIndex start;
  if (!findTraversableStart(&start)) {
    has_goal_ = false;
    publishGoalValidity(false);
    ROS_WARN_THROTTLE(
        2.0, "No traversable cell found around the current odometry.");
    return;
  }

  std::unordered_set<GridIndex, GridIndexHash> reachable;
  std::unordered_map<GridIndex, GridIndex, GridIndexHash> parent;
  std::unordered_map<GridIndex, int, GridIndexHash> path_steps;
  buildReachableRegion(start, &reachable, &parent, &path_steps,
                       reachable_search_radius_);

  const std::vector<GridIndex> frontiers =
      extractReachableFrontiers(reachable);
  const std::vector<FrontierCluster> clusters =
      clusterFrontiers(frontiers, path_steps);

  FrontierCluster selected;
  std::vector<GridIndex> path;
  GridIndex tracking_waypoint;
  const bool target_found =
      selectExplorationTarget(clusters, &selected);
  rememberDeferredBranches(clusters, target_found ? &selected : nullptr);
  const bool path_found =
      target_found &&
      recoverPath(start, selected.representative, parent, &path);
  bool waypoint_found =
      path_found && selectTrackingWaypoint(path, &tracking_waypoint);

  bool returning_to_saved_branch = false;
  if (!waypoint_found &&
      (has_active_direction_ || !saved_branches_.empty())) {
    // The current corridor ended. Search the complete connected known-free
    // graph so an old junction remains reachable even after it has fallen
    // outside the normal local planning radius.
    has_active_direction_ = false;
    std::unordered_set<GridIndex, GridIndexHash> global_reachable;
    std::unordered_map<GridIndex, GridIndex, GridIndexHash> global_parent;
    std::unordered_map<GridIndex, int, GridIndexHash> global_path_steps;
    buildReachableRegion(start, &global_reachable, &global_parent,
                         &global_path_steps, 0.0);
    const std::vector<GridIndex> global_frontiers =
        extractReachableFrontiers(global_reachable);
    const std::vector<FrontierCluster> global_clusters =
        clusterFrontiers(global_frontiers, global_path_steps);

    FrontierCluster saved_target;
    std::vector<GridIndex> saved_path;
    if (selectSavedBranch(global_clusters, &saved_target) &&
        recoverPath(start, saved_target.representative, global_parent,
                    &saved_path) &&
        selectTrackingWaypoint(saved_path, &tracking_waypoint)) {
      selected = saved_target;
      path = std::move(saved_path);
      waypoint_found = true;
      returning_to_saved_branch = true;
    }
  }

  if (waypoint_found) {
    const bool same_waypoint =
        has_goal_ && current_goal_index_ == tracking_waypoint;
    const bool timed_out =
        same_waypoint &&
        (ros::Time::now() - goal_created_time_).toSec() >= goal_timeout_;

    if (timed_out) {
      blacklist_[selected.representative] = ros::Time::now();
      has_goal_ = false;
      has_active_direction_ = false;
      ROS_WARN_STREAM("Tracking waypoint timed out. Frontier temporarily "
                      "blacklisted at [" << selected.x << ", "
                      << selected.y << "].");
    } else {
      updateTrackingGoal(tracking_waypoint, selected.representative);
      no_frontier_cycles_ = 0;
      ROS_INFO_STREAM_THROTTLE(
          1.0, "Reachable frontier target=[" << selected.x << ", "
          << selected.y << "], path_distance=" << selected.distance
          << (returning_to_saved_branch ? ", returning_to_saved_branch" : "")
          << ", deferred_branches=" << saved_branches_.size()
          << ", tracking waypoint=["
          << current_goal_.pose.position.x << ", "
          << current_goal_.pose.position.y << "]");
    }
  } else {
    ++no_frontier_cycles_;
    has_goal_ = false;

    // Exploration is complete only after both current frontiers and the
    // persistent deferred-branch pool have been exhausted.
    has_active_direction_ = false;
    if (saved_branches_.empty() &&
        no_frontier_cycles_ >= finish_no_frontier_cycles_) {
      std_msgs::Bool finished;
      finished.data = true;
      finished_pub_.publish(finished);
      ROS_INFO_THROTTLE(5.0, "No reachable frontier. Exploration finished.");
    }
  }

  publishGoal();
  publishGoalValidity(has_goal_);
  publishDebug(frontiers, clusters);

  const ros::Time now = ros::Time::now();
  for (auto it = blacklist_.begin(); it != blacklist_.end();) {
    if ((now - it->second).toSec() > blacklist_duration_) {
      it = blacklist_.erase(it);
    } else {
      ++it;
    }
  }
}

}  // namespace terrain_waypoint_exploration

#ifndef ROBOT_PLAN_EXPV2_NO_MAIN
int main(int argc, char** argv) {
  ros::init(argc, argv, "terrain_waypoint_explorer");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  terrain_waypoint_exploration::TerrainWaypointExplorer explorer(nh, pnh);
  ros::spin();
  return 0;
}
#endif
