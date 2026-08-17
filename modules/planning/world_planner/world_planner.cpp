#include "modules/planning/world_planner/world_planner.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <queue>
#include <stdexcept>

namespace jojo {
namespace planning {

WorldPlanner::WorldPlanner(const WorldPlannerConfig& config) : config_(config) {
  if (config_.grid_resolution <= 0.0 || config_.map_extra_margin < 0.0 ||
      config_.path_point_spacing <= 0.0 || config_.path_check_step <= 0.0 ||
      config_.reuse_search_radius < 0.0 ||
      config_.goal_change_replan_distance < 0.0 ||
      config_.heuristic_weight <= 0.0 || config_.world_frame.empty()) {
    throw std::invalid_argument("Invalid world planner configuration");
  }
  ReadWaypointBounds();
}

void WorldPlanner::SetTerrain(
    const pcl::PointCloud<pcl::PointXYZI>& terrain) {
  terrain_ = terrain;
  has_terrain_ = true;
}

void WorldPlanner::SetOdometry(const common_struct::Pose& pose) {
  vehicle_x_ = pose.position.x;
  vehicle_y_ = pose.position.y;
  has_pose_ = true;
}

void WorldPlanner::SetGoal(const common_struct::Pose& goal) {
  goal_ = goal;
  has_goal_ = true;
}

int WorldPlanner::ToIndex(int x, int y, int width) const {
  return y * width + x;
}

bool WorldPlanner::InBounds(int x, int y, int width, int height) const {
  return x >= 0 && x < width && y >= 0 && y < height;
}

double WorldPlanner::CellCenterX(const GridSpec& grid, int x) const {
  return grid.min_x + (x + 0.5) * config_.grid_resolution;
}

double WorldPlanner::CellCenterY(const GridSpec& grid, int y) const {
  return grid.min_y + (y + 0.5) * config_.grid_resolution;
}

bool WorldPlanner::WorldToCell(const GridSpec& grid, double world_x,
                               double world_y, int* cell_x,
                               int* cell_y) const {
  *cell_x = static_cast<int>(
      std::floor((world_x - grid.min_x) / config_.grid_resolution));
  *cell_y = static_cast<int>(
      std::floor((world_y - grid.min_y) / config_.grid_resolution));
  return InBounds(*cell_x, *cell_y, grid.width, grid.height);
}

common_struct::Header WorldPlanner::MakeHeader(
    std::uint64_t timestamp_ns) const {
  return common_struct::Header(timestamp_ns, config_.world_frame);
}

common_struct::PoseStamped WorldPlanner::MakePoseStamped(
    double x, double y, double z,
    const common_struct::Quaternion& orientation,
    std::uint64_t timestamp_ns) const {
  common_struct::PoseStamped pose;
  pose.header = MakeHeader(timestamp_ns);
  pose.pose.position = common_struct::Vector3d(x, y, z);
  pose.pose.orientation = orientation;
  return pose;
}

void WorldPlanner::ResetCurrentPath(std::uint64_t timestamp_ns) {
  current_global_path_.poses.clear();
  current_global_path_.header = MakeHeader(timestamp_ns);
  current_path_anchor_index_ = 0;
  planned_goal_x_ = std::numeric_limits<double>::quiet_NaN();
  planned_goal_y_ = std::numeric_limits<double>::quiet_NaN();
}

void WorldPlanner::ReadWaypointBounds() {
  if (config_.waypoint_file.empty()) {
    return;
  }
  FILE* file = std::fopen(config_.waypoint_file.c_str(), "r");
  if (file == nullptr) {
    return;
  }

  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  float roll = 0.0F;
  float pitch = 0.0F;
  float yaw = 0.0F;
  bool found = false;
  while (std::fscanf(file, "%f %f %f %f %f %f", &x, &y, &z, &roll,
                     &pitch, &yaw) == 6) {
    if (!found) {
      waypoint_min_x_ = waypoint_max_x_ = x;
      waypoint_min_y_ = waypoint_max_y_ = y;
      found = true;
    } else {
      waypoint_min_x_ = std::min(waypoint_min_x_, static_cast<double>(x));
      waypoint_max_x_ = std::max(waypoint_max_x_, static_cast<double>(x));
      waypoint_min_y_ = std::min(waypoint_min_y_, static_cast<double>(y));
      waypoint_max_y_ = std::max(waypoint_max_y_, static_cast<double>(y));
    }
  }
  std::fclose(file);
  has_waypoint_bounds_ = found;
}

WorldPlanner::GridSpec WorldPlanner::BuildRequiredMapSpec() const {
  double min_x = std::min(vehicle_x_, goal_.position.x);
  double max_x = std::max(vehicle_x_, goal_.position.x);
  double min_y = std::min(vehicle_y_, goal_.position.y);
  double max_y = std::max(vehicle_y_, goal_.position.y);
  if (has_waypoint_bounds_) {
    min_x = std::min(min_x, waypoint_min_x_);
    max_x = std::max(max_x, waypoint_max_x_);
    min_y = std::min(min_y, waypoint_min_y_);
    max_y = std::max(max_y, waypoint_max_y_);
  }
  min_x -= config_.map_extra_margin;
  max_x += config_.map_extra_margin;
  min_y -= config_.map_extra_margin;
  max_y += config_.map_extra_margin;

  GridSpec grid;
  grid.min_x = min_x;
  grid.min_y = min_y;
  grid.width = std::max(
      10, static_cast<int>(std::ceil((max_x - min_x) /
                                     config_.grid_resolution)) +
              1);
  grid.height = std::max(
      10, static_cast<int>(std::ceil((max_y - min_y) /
                                     config_.grid_resolution)) +
              1);
  return grid;
}

void WorldPlanner::EnsureMapCoverage() {
  const GridSpec required = BuildRequiredMapSpec();
  if (!has_map_) {
    global_map_spec_ = required;
    global_obstacle_map_.assign(
        global_map_spec_.width * global_map_spec_.height, 0);
    has_map_ = true;
    return;
  }

  const double current_max_x =
      global_map_spec_.min_x +
      global_map_spec_.width * config_.grid_resolution;
  const double current_max_y =
      global_map_spec_.min_y +
      global_map_spec_.height * config_.grid_resolution;
  const double required_max_x =
      required.min_x + required.width * config_.grid_resolution;
  const double required_max_y =
      required.min_y + required.height * config_.grid_resolution;
  const bool resize = required.min_x < global_map_spec_.min_x ||
                      required.min_y < global_map_spec_.min_y ||
                      required_max_x > current_max_x ||
                      required_max_y > current_max_y;
  if (!resize) {
    return;
  }

  GridSpec next;
  next.min_x = std::min(global_map_spec_.min_x, required.min_x);
  next.min_y = std::min(global_map_spec_.min_y, required.min_y);
  const double next_max_x = std::max(current_max_x, required_max_x);
  const double next_max_y = std::max(current_max_y, required_max_y);
  next.width = std::max(
      10, static_cast<int>(std::ceil((next_max_x - next.min_x) /
                                     config_.grid_resolution)) +
              1);
  next.height = std::max(
      10, static_cast<int>(std::ceil((next_max_y - next.min_y) /
                                     config_.grid_resolution)) +
              1);

  std::vector<std::uint8_t> next_map(next.width * next.height, 0);
  const int offset_x = static_cast<int>(std::llround(
      (global_map_spec_.min_x - next.min_x) / config_.grid_resolution));
  const int offset_y = static_cast<int>(std::llround(
      (global_map_spec_.min_y - next.min_y) / config_.grid_resolution));
  for (int y = 0; y < global_map_spec_.height; ++y) {
    for (int x = 0; x < global_map_spec_.width; ++x) {
      if (!global_obstacle_map_[ToIndex(x, y, global_map_spec_.width)]) {
        continue;
      }
      const int next_x = x + offset_x;
      const int next_y = y + offset_y;
      if (InBounds(next_x, next_y, next.width, next.height)) {
        next_map[ToIndex(next_x, next_y, next.width)] = 255;
      }
    }
  }
  global_map_spec_ = next;
  global_obstacle_map_.swap(next_map);
}

void WorldPlanner::UpdateObstacleMapFromTerrain() {
  if (!has_map_) {
    return;
  }
  std::vector<std::uint8_t> frame_state(global_obstacle_map_.size(), 0);
  std::vector<int> touched;
  touched.reserve(terrain_.size());
  for (const pcl::PointXYZI& point : terrain_.points) {
    int cell_x = 0;
    int cell_y = 0;
    if (!WorldToCell(global_map_spec_, point.x, point.y, &cell_x, &cell_y)) {
      continue;
    }
    const int index = ToIndex(cell_x, cell_y, global_map_spec_.width);
    if (frame_state[index] == 0) {
      frame_state[index] = 1;
      touched.push_back(index);
    }
    if (point.intensity > config_.obstacle_height_threshold) {
      frame_state[index] = 2;
    }
  }
  for (int index : touched) {
    global_obstacle_map_[index] = frame_state[index] == 2 ? 255 : 0;
  }
}

bool WorldPlanner::FindNearestFreeCell(
    const GridSpec& grid, const std::vector<std::uint8_t>& occupancy,
    int* cell_x, int* cell_y) const {
  if (InBounds(*cell_x, *cell_y, grid.width, grid.height) &&
      !occupancy[ToIndex(*cell_x, *cell_y, grid.width)]) {
    return true;
  }
  const int max_radius = std::max(grid.width, grid.height);
  for (int radius = 1; radius < max_radius; ++radius) {
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        if (std::abs(dx) != radius && std::abs(dy) != radius) {
          continue;
        }
        const int x = *cell_x + dx;
        const int y = *cell_y + dy;
        if (InBounds(x, y, grid.width, grid.height) &&
            !occupancy[ToIndex(x, y, grid.width)]) {
          *cell_x = x;
          *cell_y = y;
          return true;
        }
      }
    }
  }
  return false;
}

bool WorldPlanner::EstimateGoalCell(
    const GridSpec& grid, const std::vector<std::uint8_t>& occupancy,
    int start_x, int start_y, int* goal_x, int* goal_y) const {
  if (!WorldToCell(grid, goal_.position.x, goal_.position.y, goal_x, goal_y)) {
    return false;
  }
  if (FindNearestFreeCell(grid, occupancy, goal_x, goal_y)) {
    return true;
  }
  *goal_x = start_x;
  *goal_y = start_y;
  return false;
}

bool WorldPlanner::RunAStar(
    const GridSpec& grid, const std::vector<std::uint8_t>& occupancy,
    int start_x, int start_y, int goal_x, int goal_y,
    std::vector<int>* parents) const {
  const int total = grid.width * grid.height;
  const int start_index = ToIndex(start_x, start_y, grid.width);
  const int goal_index = ToIndex(goal_x, goal_y, grid.width);
  parents->assign(total, -1);
  std::vector<double> costs(total,
                            std::numeric_limits<double>::infinity());
  std::vector<std::uint8_t> closed(total, 0);
  std::priority_queue<SearchNode> open;
  const int delta_x[8] = {1, 1, 0, -1, -1, -1, 0, 1};
  const int delta_y[8] = {0, 1, 1, 1, 0, -1, -1, -1};

  costs[start_index] = 0.0;
  open.push({start_index,
             config_.heuristic_weight *
                 std::hypot(goal_x - start_x, goal_y - start_y) *
                 config_.grid_resolution});
  while (!open.empty()) {
    const SearchNode current = open.top();
    open.pop();
    if (closed[current.index]) {
      continue;
    }
    closed[current.index] = 1;
    if (current.index == goal_index) {
      return true;
    }
    const int current_x = current.index % grid.width;
    const int current_y = current.index / grid.width;
    for (int direction = 0; direction < 8; ++direction) {
      const int x = current_x + delta_x[direction];
      const int y = current_y + delta_y[direction];
      if (!InBounds(x, y, grid.width, grid.height)) {
        continue;
      }
      const int index = ToIndex(x, y, grid.width);
      if (occupancy[index] || closed[index]) {
        continue;
      }
      const double step = direction % 2 == 0
                              ? config_.grid_resolution
                              : config_.grid_resolution * 1.41421356237;
      const double tentative = costs[current.index] + step;
      if (tentative < costs[index]) {
        costs[index] = tentative;
        (*parents)[index] = current.index;
        const double heuristic =
            config_.heuristic_weight * std::hypot(goal_x - x, goal_y - y) *
            config_.grid_resolution;
        open.push({index, tentative + heuristic});
      }
    }
  }
  return false;
}

bool WorldPlanner::IsCellBlocked(
    const GridSpec& grid, const std::vector<std::uint8_t>& occupancy,
    double world_x, double world_y) const {
  int cell_x = 0;
  int cell_y = 0;
  if (!WorldToCell(grid, world_x, world_y, &cell_x, &cell_y)) {
    return true;
  }
  return occupancy[ToIndex(cell_x, cell_y, grid.width)] != 0;
}

bool WorldPlanner::IsSegmentBlocked(
    const GridSpec& grid, const std::vector<std::uint8_t>& occupancy,
    double x0, double y0, double x1, double y1) const {
  const double dx = x1 - x0;
  const double dy = y1 - y0;
  const double length = std::hypot(dx, dy);
  if (length < 1.0e-6) {
    return IsCellBlocked(grid, occupancy, x0, y0);
  }
  const int steps = std::max(
      1, static_cast<int>(std::ceil(length / config_.path_check_step)));
  for (int step = 0; step <= steps; ++step) {
    const double ratio = static_cast<double>(step) / steps;
    if (IsCellBlocked(grid, occupancy, x0 + dx * ratio, y0 + dy * ratio)) {
      return true;
    }
  }
  return false;
}

common_struct::Path WorldPlanner::BuildPath(
    const GridSpec& grid, const std::vector<int>& parents, int goal_x,
    int goal_y, std::uint64_t timestamp_ns) const {
  common_struct::Path path;
  path.header = MakeHeader(timestamp_ns);
  std::vector<common_struct::PoseStamped> reversed;
  int index = ToIndex(goal_x, goal_y, grid.width);
  const common_struct::Quaternion identity;
  while (index >= 0) {
    const int x = index % grid.width;
    const int y = index / grid.width;
    reversed.push_back(MakePoseStamped(CellCenterX(grid, x),
                                       CellCenterY(grid, y), 0.0, identity,
                                       timestamp_ns));
    index = parents[index];
  }
  std::reverse(reversed.begin(), reversed.end());
  path.poses.swap(reversed);
  return path;
}

common_struct::Path WorldPlanner::BuildStraightPath(
    double end_x, double end_y, std::uint64_t timestamp_ns) const {
  common_struct::Path path;
  path.header = MakeHeader(timestamp_ns);
  path.poses.push_back(MakePoseStamped(vehicle_x_, vehicle_y_, 0.0,
                                       common_struct::Quaternion(),
                                       timestamp_ns));
  path.poses.push_back(
      MakePoseStamped(end_x, end_y, 0.0, goal_.orientation, timestamp_ns));
  return path;
}

common_struct::Path WorldPlanner::ShortcutPath(
    const GridSpec& grid, const std::vector<std::uint8_t>& occupancy,
    const common_struct::Path& raw_path) const {
  common_struct::Path result = raw_path;
  if (raw_path.poses.size() < 3) {
    return result;
  }
  result.poses.clear();
  std::size_t anchor = 0;
  result.poses.push_back(raw_path.poses.front());
  while (anchor + 1 < raw_path.poses.size()) {
    std::size_t next = anchor + 1;
    for (std::size_t candidate = raw_path.poses.size() - 1;
         candidate > anchor + 1; --candidate) {
      const auto& begin = raw_path.poses[anchor].pose.position;
      const auto& end = raw_path.poses[candidate].pose.position;
      if (!IsSegmentBlocked(grid, occupancy, begin.x, begin.y, end.x, end.y)) {
        next = candidate;
        break;
      }
    }
    result.poses.push_back(raw_path.poses[next]);
    anchor = next;
  }
  return result;
}

common_struct::Path WorldPlanner::SparsifyPath(
    const common_struct::Path& raw_path) const {
  common_struct::Path result = raw_path;
  if (raw_path.poses.empty()) {
    return result;
  }
  result.poses.clear();
  result.poses.push_back(raw_path.poses.front());
  double accumulated_distance = 0.0;
  for (std::size_t index = 1; index < raw_path.poses.size(); ++index) {
    const auto& current = raw_path.poses[index].pose.position;
    const auto& previous = raw_path.poses[index - 1].pose.position;
    accumulated_distance +=
        std::hypot(current.x - previous.x, current.y - previous.y);
    if (accumulated_distance >= config_.path_point_spacing ||
        index + 1 == raw_path.poses.size()) {
      result.poses.push_back(raw_path.poses[index]);
      accumulated_distance = 0.0;
    }
  }
  const auto& result_end = result.poses.back().pose.position;
  const auto& raw_end = raw_path.poses.back().pose.position;
  if (result_end.x != raw_end.x || result_end.y != raw_end.y) {
    result.poses.push_back(raw_path.poses.back());
  }
  return result;
}

bool WorldPlanner::GoalChangedMeaningfully() const {
  if (!std::isfinite(planned_goal_x_) || !std::isfinite(planned_goal_y_)) {
    return true;
  }
  return std::hypot(goal_.position.x - planned_goal_x_,
                    goal_.position.y - planned_goal_y_) >
         config_.goal_change_replan_distance;
}

bool WorldPlanner::UpdatePathAnchorAndCheckConnectivity(
    const GridSpec& grid, const std::vector<std::uint8_t>& occupancy) {
  if (current_global_path_.poses.size() < 2) {
    return false;
  }
  double best_distance = std::numeric_limits<double>::infinity();
  std::size_t best_index = current_path_anchor_index_;
  const std::size_t begin = std::min(current_path_anchor_index_,
                                     current_global_path_.poses.size() - 1);
  for (std::size_t index = begin; index < current_global_path_.poses.size();
       ++index) {
    const auto& position = current_global_path_.poses[index].pose.position;
    const double distance =
        std::hypot(position.x - vehicle_x_, position.y - vehicle_y_);
    if (distance > config_.reuse_search_radius ||
        IsSegmentBlocked(grid, occupancy, vehicle_x_, vehicle_y_, position.x,
                         position.y)) {
      continue;
    }
    if (distance < best_distance) {
      best_distance = distance;
      best_index = index;
    }
  }
  if (!std::isfinite(best_distance)) {
    return false;
  }
  current_path_anchor_index_ = best_index;
  for (std::size_t index = current_path_anchor_index_;
       index + 1 < current_global_path_.poses.size(); ++index) {
    const auto& begin_position =
        current_global_path_.poses[index].pose.position;
    const auto& end_position =
        current_global_path_.poses[index + 1].pose.position;
    if (IsSegmentBlocked(grid, occupancy, begin_position.x, begin_position.y,
                         end_position.x, end_position.y)) {
      return false;
    }
  }
  return true;
}

common_struct::Path WorldPlanner::BuildDisplayPath(
    const GridSpec& grid, const std::vector<std::uint8_t>& occupancy,
    std::uint64_t timestamp_ns) const {
  common_struct::Path display;
  display.header = MakeHeader(timestamp_ns);
  display.poses.push_back(MakePoseStamped(vehicle_x_, vehicle_y_, 0.0,
                                          common_struct::Quaternion(),
                                          timestamp_ns));
  if (!current_global_path_.poses.empty()) {
    const std::size_t anchor = std::min(
        current_path_anchor_index_, current_global_path_.poses.size() - 1);
    for (std::size_t index = anchor;
         index < current_global_path_.poses.size(); ++index) {
      common_struct::PoseStamped pose = current_global_path_.poses[index];
      pose.header = display.header;
      display.poses.push_back(pose);
    }
  }
  if (!display.poses.empty()) {
    const auto& last = display.poses.back().pose.position;
    if (!IsSegmentBlocked(grid, occupancy, last.x, last.y, goal_.position.x,
                          goal_.position.y)) {
      display.poses.push_back(MakePoseStamped(
          goal_.position.x, goal_.position.y, goal_.position.z,
          goal_.orientation, timestamp_ns));
    }
  }
  return display;
}

common_struct::OccupancyGrid WorldPlanner::BuildOccupancyGrid(
    std::uint64_t timestamp_ns) const {
  common_struct::OccupancyGrid grid;
  grid.header = MakeHeader(timestamp_ns);
  grid.info.map_load_time = timestamp_ns;
  grid.info.resolution = static_cast<float>(config_.grid_resolution);
  grid.info.width = static_cast<std::uint32_t>(global_map_spec_.width);
  grid.info.height = static_cast<std::uint32_t>(global_map_spec_.height);
  grid.info.origin.position = common_struct::Vector3d(
      global_map_spec_.min_x, global_map_spec_.min_y, 0.0);
  grid.data.resize(global_obstacle_map_.size(), 0);
  for (std::size_t index = 0; index < global_obstacle_map_.size(); ++index) {
    grid.data[index] = global_obstacle_map_[index] ? 100 : 0;
  }
  return grid;
}

WorldPlannerOutput WorldPlanner::Step(std::uint64_t timestamp_ns) {
  WorldPlannerOutput output;
  if (!has_pose_ || !has_goal_ || !has_terrain_) {
    return output;
  }
  output.ready = true;
  const double goal_distance = std::hypot(goal_.position.x - vehicle_x_,
                                          goal_.position.y - vehicle_y_);
  if (goal_distance < config_.goal_reached_xy) {
    ResetCurrentPath(timestamp_ns);
    output.path = current_global_path_;
    if (has_map_) {
      output.has_obstacle_grid = true;
      output.obstacle_grid = BuildOccupancyGrid(timestamp_ns);
    }
    return output;
  }

  EnsureMapCoverage();
  UpdateObstacleMapFromTerrain();
  output.has_obstacle_grid = true;
  output.obstacle_grid = BuildOccupancyGrid(timestamp_ns);
  const std::vector<std::uint8_t>& occupancy = global_obstacle_map_;

  if (!GoalChangedMeaningfully() &&
      UpdatePathAnchorAndCheckConnectivity(global_map_spec_, occupancy)) {
    output.path = BuildDisplayPath(global_map_spec_, occupancy, timestamp_ns);
    return output;
  }

  int start_x = 0;
  int start_y = 0;
  if (!WorldToCell(global_map_spec_, vehicle_x_, vehicle_y_, &start_x,
                   &start_y) ||
      !FindNearestFreeCell(global_map_spec_, occupancy, &start_x, &start_y)) {
    ResetCurrentPath(timestamp_ns);
    output.path = current_global_path_;
    return output;
  }

  int goal_x = 0;
  int goal_y = 0;
  if (!EstimateGoalCell(global_map_spec_, occupancy, start_x, start_y, &goal_x,
                        &goal_y)) {
    ResetCurrentPath(timestamp_ns);
    output.path = current_global_path_;
    return output;
  }

  if (!IsSegmentBlocked(global_map_spec_, occupancy, vehicle_x_, vehicle_y_,
                        goal_.position.x, goal_.position.y)) {
    current_global_path_ = BuildStraightPath(
        goal_.position.x, goal_.position.y, timestamp_ns);
  } else {
    const double goal_cell_x = CellCenterX(global_map_spec_, goal_x);
    const double goal_cell_y = CellCenterY(global_map_spec_, goal_y);
    if (!IsSegmentBlocked(global_map_spec_, occupancy, vehicle_x_, vehicle_y_,
                          goal_cell_x, goal_cell_y)) {
      current_global_path_ =
          BuildStraightPath(goal_cell_x, goal_cell_y, timestamp_ns);
    } else {
      std::vector<int> parents;
      if (!RunAStar(global_map_spec_, occupancy, start_x, start_y, goal_x,
                    goal_y, &parents)) {
        ResetCurrentPath(timestamp_ns);
        output.path = current_global_path_;
        return output;
      }
      const common_struct::Path raw = BuildPath(
          global_map_spec_, parents, goal_x, goal_y, timestamp_ns);
      current_global_path_ =
          SparsifyPath(ShortcutPath(global_map_spec_, occupancy, raw));
      current_global_path_.header = MakeHeader(timestamp_ns);
    }
  }

  current_path_anchor_index_ = 0;
  planned_goal_x_ = goal_.position.x;
  planned_goal_y_ = goal_.position.y;
  output.path = BuildDisplayPath(global_map_spec_, occupancy, timestamp_ns);
  return output;
}

}  // namespace planning
}  // namespace jojo
