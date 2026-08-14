#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <limits>
#include <queue>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;

namespace {

double gridResolution = 0.4;
double mapExtraMargin = 10.0;
double robotWidth = 0.8;
double robotLength = 1.5;
double inflationMargin = 0.15;
double obstacleHeightThre = 0.3;
double pathPointSpacing = 0.5;
double goalReachedXY = 0.3;
double pathCheckStep = 0.15;
double reuseSearchRadius = 1.2;
double goalChangeReplanDis = 0.6;
double heuristicWeight = 1.0;
string waypointFileDir;

bool hasPose = false;
bool hasGoal = false;
bool hasTerrain = false;
bool hasMap = false;
bool hasWaypointBounds = false;

double vehicleX = 0.0;
double vehicleY = 0.0;
geometry_msgs::PoseStamped goalPose;
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud(new pcl::PointCloud<pcl::PointXYZI>());

double waypointMinX = 0.0;
double waypointMaxX = 0.0;
double waypointMinY = 0.0;
double waypointMaxY = 0.0;

struct GridSpec {
  double minX;
  double minY;
  int width;
  int height;
};

struct Node {
  int idx;
  double f;
  bool operator<(const Node& other) const {
    return f > other.f;
  }
};

GridSpec globalMapSpec;
vector<uint8_t> globalObstacleMap;
nav_msgs::Path currentGlobalPath;
size_t currentPathAnchorIdx = 0;
double plannedGoalX = numeric_limits<double>::quiet_NaN();
double plannedGoalY = numeric_limits<double>::quiet_NaN();

int toIndex(int x, int y, int width) {
  return y * width + x;
}

bool inBounds(int x, int y, int width, int height) {
  return x >= 0 && x < width && y >= 0 && y < height;
}

double cellCenterX(const GridSpec& grid, int x) {
  return grid.minX + (x + 0.5) * gridResolution;
}

double cellCenterY(const GridSpec& grid, int y) {
  return grid.minY + (y + 0.5) * gridResolution;
}

bool worldToCell(const GridSpec& grid, double wx, double wy, int& cx, int& cy) {
  cx = static_cast<int>(floor((wx - grid.minX) / gridResolution));
  cy = static_cast<int>(floor((wy - grid.minY) / gridResolution));
  return inBounds(cx, cy, grid.width, grid.height);
}

void resetCurrentPath() {
  currentGlobalPath.poses.clear();
  currentGlobalPath.header.stamp = ros::Time::now();
  currentGlobalPath.header.frame_id = "map";
  currentPathAnchorIdx = 0;
  plannedGoalX = numeric_limits<double>::quiet_NaN();
  plannedGoalY = numeric_limits<double>::quiet_NaN();
}

void readWaypointBounds() {
  if (waypointFileDir.empty()) {
    return;
  }

  FILE* waypointFile = fopen(waypointFileDir.c_str(), "r");
  if (waypointFile == NULL) {
    ROS_WARN_THROTTLE(5.0, "worldPlanner cannot read waypoint file: %s", waypointFileDir.c_str());
    return;
  }

  float tx, ty, tz, roll, pitch, yaw;
  bool found = false;
  while (fscanf(waypointFile, "%f %f %f %f %f %f", &tx, &ty, &tz, &roll, &pitch, &yaw) == 6) {
    if (!found) {
      waypointMinX = waypointMaxX = tx;
      waypointMinY = waypointMaxY = ty;
      found = true;
      continue;
    }
    waypointMinX = std::min(waypointMinX, static_cast<double>(tx));
    waypointMaxX = std::max(waypointMaxX, static_cast<double>(tx));
    waypointMinY = std::min(waypointMinY, static_cast<double>(ty));
    waypointMaxY = std::max(waypointMaxY, static_cast<double>(ty));
  }
  fclose(waypointFile);

  hasWaypointBounds = found;
}

GridSpec buildRequiredMapSpec() {
  double minX = std::min(vehicleX, goalPose.pose.position.x);
  double maxX = std::max(vehicleX, goalPose.pose.position.x);
  double minY = std::min(vehicleY, goalPose.pose.position.y);
  double maxY = std::max(vehicleY, goalPose.pose.position.y);

  if (hasWaypointBounds) {
    minX = std::min(minX, waypointMinX);
    maxX = std::max(maxX, waypointMaxX);
    minY = std::min(minY, waypointMinY);
    maxY = std::max(maxY, waypointMaxY);
  }

  minX -= mapExtraMargin;
  maxX += mapExtraMargin;
  minY -= mapExtraMargin;
  maxY += mapExtraMargin;

  GridSpec grid;
  grid.minX = minX;
  grid.minY = minY;
  grid.width = std::max(10, static_cast<int>(ceil((maxX - minX) / gridResolution)) + 1);
  grid.height = std::max(10, static_cast<int>(ceil((maxY - minY) / gridResolution)) + 1);
  return grid;
}

void ensureMapCoverage() {
  GridSpec required = buildRequiredMapSpec();
  if (!hasMap) {
    globalMapSpec = required;
    globalObstacleMap.assign(globalMapSpec.width * globalMapSpec.height, 0);
    hasMap = true;
    return;
  }

  double currentMaxX = globalMapSpec.minX + globalMapSpec.width * gridResolution;
  double currentMaxY = globalMapSpec.minY + globalMapSpec.height * gridResolution;
  double requiredMaxX = required.minX + required.width * gridResolution;
  double requiredMaxY = required.minY + required.height * gridResolution;

  bool needResize = required.minX < globalMapSpec.minX || required.minY < globalMapSpec.minY ||
                    requiredMaxX > currentMaxX || requiredMaxY > currentMaxY;
  if (!needResize) {
    return;
  }

  GridSpec resized;
  resized.minX = std::min(globalMapSpec.minX, required.minX);
  resized.minY = std::min(globalMapSpec.minY, required.minY);
  double resizedMaxX = std::max(currentMaxX, requiredMaxX);
  double resizedMaxY = std::max(currentMaxY, requiredMaxY);
  resized.width = std::max(10, static_cast<int>(ceil((resizedMaxX - resized.minX) / gridResolution)) + 1);
  resized.height = std::max(10, static_cast<int>(ceil((resizedMaxY - resized.minY) / gridResolution)) + 1);

  vector<uint8_t> resizedMap(resized.width * resized.height, 0);
  int offsetX = static_cast<int>(llround((globalMapSpec.minX - resized.minX) / gridResolution));
  int offsetY = static_cast<int>(llround((globalMapSpec.minY - resized.minY) / gridResolution));
  for (int y = 0; y < globalMapSpec.height; ++y) {
    for (int x = 0; x < globalMapSpec.width; ++x) {
      int oldIdx = toIndex(x, y, globalMapSpec.width);
      if (!globalObstacleMap[oldIdx]) {
        continue;
      }
      int nx = x + offsetX;
      int ny = y + offsetY;
      if (inBounds(nx, ny, resized.width, resized.height)) {
        resizedMap[toIndex(nx, ny, resized.width)] = 255;
      }
    }
  }

  globalMapSpec = resized;
  globalObstacleMap.swap(resizedMap);
}

void updateObstacleMapFromTerrain() {
  if (!hasMap) {
    return;
  }

  vector<uint8_t> frameState(globalObstacleMap.size(), 0);
  vector<int> touchedIndices;
  touchedIndices.reserve(terrainCloud->points.size());

  int terrainSize = terrainCloud->points.size();
  for (int i = 0; i < terrainSize; ++i) {
    const auto& pt = terrainCloud->points[i];
    int cx, cy;
    if (!worldToCell(globalMapSpec, pt.x, pt.y, cx, cy)) {
      continue;
    }
    int idx = toIndex(cx, cy, globalMapSpec.width);
    if (frameState[idx] == 0) {
      frameState[idx] = 1;
      touchedIndices.push_back(idx);
    }
    if (pt.intensity > obstacleHeightThre) {
      frameState[idx] = 2;
    }
  }

  for (int idx : touchedIndices) {
    globalObstacleMap[idx] = (frameState[idx] == 2) ? 255 : 0;
  }
}

const vector<uint8_t>& getPlanningOccupancy() {
  return globalObstacleMap;
}

bool findNearestFreeCell(const GridSpec& grid, const vector<uint8_t>& occ, int& cx, int& cy) {
  if (inBounds(cx, cy, grid.width, grid.height) && !occ[toIndex(cx, cy, grid.width)]) {
    return true;
  }

  int maxRadius = std::max(grid.width, grid.height);
  for (int radius = 1; radius < maxRadius; ++radius) {
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        if (abs(dx) != radius && abs(dy) != radius) {
          continue;
        }
        int nx = cx + dx;
        int ny = cy + dy;
        if (inBounds(nx, ny, grid.width, grid.height) && !occ[toIndex(nx, ny, grid.width)]) {
          cx = nx;
          cy = ny;
          return true;
        }
      }
    }
  }
  return false;
}

bool estimateGoalCell(const GridSpec& grid, const vector<uint8_t>& occ, int startX, int startY, int& goalX, int& goalY) {
  if (!worldToCell(grid, goalPose.pose.position.x, goalPose.pose.position.y, goalX, goalY)) {
    return false;
  }
  if (findNearestFreeCell(grid, occ, goalX, goalY)) {
    return true;
  }
  goalX = startX;
  goalY = startY;
  return false;
}

bool runAStar(const GridSpec& grid,
              const vector<uint8_t>& occ,
              int startX,
              int startY,
              int goalX,
              int goalY,
              vector<int>& parents) {
  const int total = grid.width * grid.height;
  const int startIdx = toIndex(startX, startY, grid.width);
  const int goalIdx = toIndex(goalX, goalY, grid.width);

  parents.assign(total, -1);
  vector<double> g(total, numeric_limits<double>::infinity());
  vector<uint8_t> closed(total, 0);
  priority_queue<Node> open;

  const int nx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
  const int ny[8] = {0, 1, 1, 1, 0, -1, -1, -1};

  g[startIdx] = 0.0;
  open.push({startIdx, heuristicWeight * hypot(goalX - startX, goalY - startY) * gridResolution});

  while (!open.empty()) {
    Node cur = open.top();
    open.pop();
    if (closed[cur.idx]) {
      continue;
    }
    closed[cur.idx] = 1;

    if (cur.idx == goalIdx) {
      return true;
    }

    int cx = cur.idx % grid.width;
    int cy = cur.idx / grid.width;
    for (int i = 0; i < 8; ++i) {
      int x = cx + nx[i];
      int y = cy + ny[i];
      if (!inBounds(x, y, grid.width, grid.height)) {
        continue;
      }

      int nidx = toIndex(x, y, grid.width);
      if (occ[nidx] || closed[nidx]) {
        continue;
      }

      double stepCost = (i % 2 == 0) ? gridResolution : gridResolution * 1.41421356237;
      double tentative = g[cur.idx] + stepCost;
      if (tentative < g[nidx]) {
        g[nidx] = tentative;
        parents[nidx] = cur.idx;
        double h = heuristicWeight * hypot(goalX - x, goalY - y) * gridResolution;
        open.push({nidx, tentative + h});
      }
    }
  }

  return false;
}

bool isCellBlocked(const GridSpec& grid, const vector<uint8_t>& occ, double wx, double wy) {
  int cx, cy;
  if (!worldToCell(grid, wx, wy, cx, cy)) {
    return true;
  }
  return occ[toIndex(cx, cy, grid.width)] != 0;
}

bool isSegmentBlocked(const GridSpec& grid, const vector<uint8_t>& occ, double x0, double y0, double x1, double y1) {
  double dx = x1 - x0;
  double dy = y1 - y0;
  double len = hypot(dx, dy);
  if (len < 1e-6) {
    return isCellBlocked(grid, occ, x0, y0);
  }

  int steps = std::max(1, static_cast<int>(ceil(len / pathCheckStep)));
  for (int i = 0; i <= steps; ++i) {
    double ratio = static_cast<double>(i) / static_cast<double>(steps);
    double wx = x0 + dx * ratio;
    double wy = y0 + dy * ratio;
    if (isCellBlocked(grid, occ, wx, wy)) {
      return true;
    }
  }
  return false;
}

nav_msgs::Path buildPathMessage(const GridSpec& grid, const vector<int>& parents, int goalX, int goalY) {
  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "map";

  int goalIdx = toIndex(goalX, goalY, grid.width);
  vector<geometry_msgs::PoseStamped> reversed;
  int idx = goalIdx;
  while (idx >= 0) {
    int x = idx % grid.width;
    int y = idx / grid.width;
    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = cellCenterX(grid, x);
    pose.pose.position.y = cellCenterY(grid, y);
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    reversed.push_back(pose);
    idx = parents[idx];
  }
  reverse(reversed.begin(), reversed.end());
  path.poses = reversed;
  return path;
}

nav_msgs::Path buildStraightPath(double endX, double endY) {
  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "map";

  geometry_msgs::PoseStamped startPose;
  startPose.header = path.header;
  startPose.pose.position.x = vehicleX;
  startPose.pose.position.y = vehicleY;
  startPose.pose.position.z = 0.0;
  startPose.pose.orientation.w = 1.0;
  path.poses.push_back(startPose);

  geometry_msgs::PoseStamped endPose;
  endPose.header = path.header;
  endPose.pose.position.x = endX;
  endPose.pose.position.y = endY;
  endPose.pose.position.z = 0.0;
  endPose.pose.orientation = goalPose.pose.orientation;
  path.poses.push_back(endPose);
  return path;
}

nav_msgs::Path shortcutPath(const GridSpec& grid, const vector<uint8_t>& occ, const nav_msgs::Path& rawPath) {
  nav_msgs::Path smoothed = rawPath;
  if (rawPath.poses.size() < 3) {
    return smoothed;
  }

  smoothed.poses.clear();
  size_t anchor = 0;
  smoothed.poses.push_back(rawPath.poses.front());
  while (anchor + 1 < rawPath.poses.size()) {
    size_t next = anchor + 1;
    for (size_t candidate = rawPath.poses.size() - 1; candidate > anchor + 1; --candidate) {
      double x0 = rawPath.poses[anchor].pose.position.x;
      double y0 = rawPath.poses[anchor].pose.position.y;
      double x1 = rawPath.poses[candidate].pose.position.x;
      double y1 = rawPath.poses[candidate].pose.position.y;
      if (!isSegmentBlocked(grid, occ, x0, y0, x1, y1)) {
        next = candidate;
        break;
      }
    }
    smoothed.poses.push_back(rawPath.poses[next]);
    anchor = next;
  }

  return smoothed;
}

nav_msgs::Path sparsifyPath(const nav_msgs::Path& rawPath) {
  nav_msgs::Path sparse = rawPath;
  if (rawPath.poses.empty()) {
    return sparse;
  }

  sparse.poses.clear();
  sparse.poses.push_back(rawPath.poses.front());
  double accumDis = 0.0;
  for (size_t i = 1; i < rawPath.poses.size(); ++i) {
    double dx = rawPath.poses[i].pose.position.x - rawPath.poses[i - 1].pose.position.x;
    double dy = rawPath.poses[i].pose.position.y - rawPath.poses[i - 1].pose.position.y;
    accumDis += hypot(dx, dy);
    if (accumDis >= pathPointSpacing || i + 1 == rawPath.poses.size()) {
      sparse.poses.push_back(rawPath.poses[i]);
      accumDis = 0.0;
    }
  }

  if (sparse.poses.back().pose.position.x != rawPath.poses.back().pose.position.x ||
      sparse.poses.back().pose.position.y != rawPath.poses.back().pose.position.y) {
    sparse.poses.push_back(rawPath.poses.back());
  }

  return sparse;
}

bool goalChangedMeaningfully() {
  if (!std::isfinite(plannedGoalX) || !std::isfinite(plannedGoalY)) {
    return true;
  }
  return hypot(goalPose.pose.position.x - plannedGoalX, goalPose.pose.position.y - plannedGoalY) > goalChangeReplanDis;
}

bool updatePathAnchorAndCheckConnectivity(const GridSpec& grid, const vector<uint8_t>& occ) {
  if (currentGlobalPath.poses.size() < 2) {
    return false;
  }

  double bestDis = numeric_limits<double>::infinity();
  size_t bestIdx = currentPathAnchorIdx;
  size_t searchBegin = std::min(currentPathAnchorIdx, currentGlobalPath.poses.size() - 1);
  for (size_t i = searchBegin; i < currentGlobalPath.poses.size(); ++i) {
    double px = currentGlobalPath.poses[i].pose.position.x;
    double py = currentGlobalPath.poses[i].pose.position.y;
    double dis = hypot(px - vehicleX, py - vehicleY);
    if (dis > reuseSearchRadius) {
      continue;
    }
    if (isSegmentBlocked(grid, occ, vehicleX, vehicleY, px, py)) {
      continue;
    }
    if (dis < bestDis) {
      bestDis = dis;
      bestIdx = i;
    }
  }

  if (!std::isfinite(bestDis)) {
    return false;
  }

  currentPathAnchorIdx = bestIdx;
  for (size_t i = currentPathAnchorIdx; i + 1 < currentGlobalPath.poses.size(); ++i) {
    double x0 = currentGlobalPath.poses[i].pose.position.x;
    double y0 = currentGlobalPath.poses[i].pose.position.y;
    double x1 = currentGlobalPath.poses[i + 1].pose.position.x;
    double y1 = currentGlobalPath.poses[i + 1].pose.position.y;
    if (isSegmentBlocked(grid, occ, x0, y0, x1, y1)) {
      return false;
    }
  }

  return true;
}

nav_msgs::Path buildDisplayPath(const GridSpec& grid, const vector<uint8_t>& occ) {
  nav_msgs::Path displayPath;
  displayPath.header.stamp = ros::Time::now();
  displayPath.header.frame_id = "map";

  geometry_msgs::PoseStamped startPose;
  startPose.header = displayPath.header;
  startPose.pose.position.x = vehicleX;
  startPose.pose.position.y = vehicleY;
  startPose.pose.position.z = 0.0;
  startPose.pose.orientation.w = 1.0;
  displayPath.poses.push_back(startPose);

  if (!currentGlobalPath.poses.empty()) {
    size_t anchorIdx = std::min(currentPathAnchorIdx, currentGlobalPath.poses.size() - 1);
    for (size_t i = anchorIdx; i < currentGlobalPath.poses.size(); ++i) {
      geometry_msgs::PoseStamped pose = currentGlobalPath.poses[i];
      pose.header = displayPath.header;
      displayPath.poses.push_back(pose);
    }
  }

  if (!displayPath.poses.empty()) {
    double lastX = displayPath.poses.back().pose.position.x;
    double lastY = displayPath.poses.back().pose.position.y;
    if (!isSegmentBlocked(grid, occ, lastX, lastY, goalPose.pose.position.x, goalPose.pose.position.y)) {
      geometry_msgs::PoseStamped finalPose = goalPose;
      finalPose.header = displayPath.header;
      displayPath.poses.push_back(finalPose);
    }
  }

  return displayPath;
}

nav_msgs::OccupancyGrid buildOccupancyGridMsg() {
  nav_msgs::OccupancyGrid gridMsg;
  gridMsg.header.stamp = ros::Time::now();
  gridMsg.header.frame_id = "map";
  gridMsg.info.map_load_time = gridMsg.header.stamp;
  gridMsg.info.resolution = gridResolution;
  gridMsg.info.width = globalMapSpec.width;
  gridMsg.info.height = globalMapSpec.height;
  gridMsg.info.origin.position.x = globalMapSpec.minX;
  gridMsg.info.origin.position.y = globalMapSpec.minY;
  gridMsg.info.origin.position.z = 0.0;
  gridMsg.info.origin.orientation.w = 1.0;
  gridMsg.data.resize(globalObstacleMap.size(), 0);

  for (size_t i = 0; i < globalObstacleMap.size(); ++i) {
    gridMsg.data[i] = globalObstacleMap[i] ? 100 : 0;
  }

  return gridMsg;
}

void terrainHandler(const sensor_msgs::PointCloud2ConstPtr& msg) {
  terrainCloud->clear();
  pcl::fromROSMsg(*msg, *terrainCloud);
  hasTerrain = true;
}

void odomHandler(const nav_msgs::Odometry::ConstPtr& msg) {
  vehicleX = msg->pose.pose.position.x;
  vehicleY = msg->pose.pose.position.y;
  hasPose = true;
}

void goalHandler(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  goalPose = *msg;
  hasGoal = true;
}

}  // namespace

int RunRobotPlanWorldPlanner(ros::NodeHandle& nh,
                             ros::NodeHandle& nhPrivate) {
  nhPrivate.getParam("grid_resolution", gridResolution);
  nhPrivate.getParam("map_extra_margin", mapExtraMargin);
  nhPrivate.getParam("robot_width", robotWidth);
  nhPrivate.getParam("robot_length", robotLength);
  nhPrivate.getParam("inflation_margin", inflationMargin);
  nhPrivate.getParam("obstacle_height_thre", obstacleHeightThre);
  nhPrivate.getParam("path_point_spacing", pathPointSpacing);
  nhPrivate.getParam("goal_reached_xy", goalReachedXY);
  nhPrivate.getParam("path_check_step", pathCheckStep);
  nhPrivate.getParam("reuse_search_radius", reuseSearchRadius);
  nhPrivate.getParam("goal_change_replan_dis", goalChangeReplanDis);
  nhPrivate.getParam("heuristic_weight", heuristicWeight);
  nhPrivate.getParam("waypoint_file_dir", waypointFileDir);

  string terrainTopic = "/terrain_map";
  string odometryTopic = "/state_estimation";
  string goalTopic = "/way_point";
  string pathTopic = "/global_reference_path";
  string obstacleGridTopic = "/global_obstacle_grid";
  nhPrivate.getParam("terrain_topic", terrainTopic);
  nhPrivate.getParam("odometry_topic", odometryTopic);
  nhPrivate.getParam("goal_topic", goalTopic);
  nhPrivate.getParam("path_topic", pathTopic);
  nhPrivate.getParam("obstacle_grid_topic", obstacleGridTopic);

  readWaypointBounds();

  ros::Subscriber subTerrain = nh.subscribe<sensor_msgs::PointCloud2>(terrainTopic, 2, terrainHandler);
  ros::Subscriber subOdom = nh.subscribe<nav_msgs::Odometry>(odometryTopic, 5, odomHandler);
  ros::Subscriber subGoal = nh.subscribe<geometry_msgs::PoseStamped>(goalTopic, 5, goalHandler);
  ros::Publisher pubPath = nh.advertise<nav_msgs::Path>(pathTopic, 2, true);
  ros::Publisher pubGrid = nh.advertise<nav_msgs::OccupancyGrid>(obstacleGridTopic, 1, true);
  

  ros::Rate rate(5.0);
  while (ros::ok()) {
    ros::spinOnce();

    if (!hasPose || !hasGoal || !hasTerrain) {
      rate.sleep();
      continue;
    }

    double goalDis = hypot(goalPose.pose.position.x - vehicleX, goalPose.pose.position.y - vehicleY);
    if (goalDis < goalReachedXY) {
      resetCurrentPath();
      pubPath.publish(currentGlobalPath);
      if (hasMap) {
        pubGrid.publish(buildOccupancyGridMsg());
      }
      rate.sleep();
      continue;
    }

    ensureMapCoverage();
    updateObstacleMapFromTerrain();
    pubGrid.publish(buildOccupancyGridMsg());
    const vector<uint8_t>& planningOcc = getPlanningOccupancy();

    if (!goalChangedMeaningfully() && updatePathAnchorAndCheckConnectivity(globalMapSpec, planningOcc)) {
      pubPath.publish(buildDisplayPath(globalMapSpec, planningOcc));
      rate.sleep();
      continue;
    }

    int startX, startY;
    if (!worldToCell(globalMapSpec, vehicleX, vehicleY, startX, startY) ||
        !findNearestFreeCell(globalMapSpec, planningOcc, startX, startY)) {
      resetCurrentPath();
      pubPath.publish(currentGlobalPath);
      rate.sleep();
      continue;
    }

    int goalX, goalY;
    if (!estimateGoalCell(globalMapSpec, planningOcc, startX, startY, goalX, goalY)) {
      resetCurrentPath();
      pubPath.publish(currentGlobalPath);
      rate.sleep();
      continue;
    }

    if (!isSegmentBlocked(globalMapSpec, planningOcc, vehicleX, vehicleY,
                          goalPose.pose.position.x, goalPose.pose.position.y)) {
      currentGlobalPath = buildStraightPath(goalPose.pose.position.x, goalPose.pose.position.y);
      currentPathAnchorIdx = 0;
      plannedGoalX = goalPose.pose.position.x;
      plannedGoalY = goalPose.pose.position.y;
      pubPath.publish(buildDisplayPath(globalMapSpec, planningOcc));
      rate.sleep();
      continue;
    }

    double goalCellX = cellCenterX(globalMapSpec, goalX);
    double goalCellY = cellCenterY(globalMapSpec, goalY);
    if (!isSegmentBlocked(globalMapSpec, planningOcc, vehicleX, vehicleY, goalCellX, goalCellY)) {
      currentGlobalPath = buildStraightPath(goalCellX, goalCellY);
      currentPathAnchorIdx = 0;
      plannedGoalX = goalPose.pose.position.x;
      plannedGoalY = goalPose.pose.position.y;
      pubPath.publish(buildDisplayPath(globalMapSpec, planningOcc));
      rate.sleep();
      continue;
    }

    vector<int> parents;
    if (runAStar(globalMapSpec, planningOcc, startX, startY, goalX, goalY, parents)) {
      nav_msgs::Path rawPath = buildPathMessage(globalMapSpec, parents, goalX, goalY);
      nav_msgs::Path shortcut = shortcutPath(globalMapSpec, planningOcc, rawPath);
      currentGlobalPath = sparsifyPath(shortcut);
      currentGlobalPath.header.stamp = ros::Time::now();
      currentGlobalPath.header.frame_id = "map";
      currentPathAnchorIdx = 0;
      plannedGoalX = goalPose.pose.position.x;
      plannedGoalY = goalPose.pose.position.y;
      pubPath.publish(buildDisplayPath(globalMapSpec, planningOcc));
    } else {
      resetCurrentPath();
      pubPath.publish(currentGlobalPath);
    }

    rate.sleep();
  }

  return 0;
}

#ifndef ROBOT_PLAN_EXPV2_NO_MAIN
int main(int argc, char** argv) {
  ros::init(argc, argv, "worldPlanner");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate("~");
  return RunRobotPlanWorldPlanner(nh, nhPrivate);
}
#endif
