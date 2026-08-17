#include "modules/planning/local_planner/local_planner.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

namespace jojo {
namespace planning {

using namespace std;

constexpr double PI = 3.1415926;

#define PLOTPATHSET 1

struct LocalPlanner::Impl {
string pathFolder;
double vehicleLength = 0.6;
double vehicleWidth = 0.6;
double sensorOffsetX = 0;
double sensorOffsetY = 0;
bool twoWayDrive = true;
double laserVoxelSize = 0.05;
double terrainVoxelSize = 0.2;
bool useTerrainAnalysis = false;
bool checkObstacle = true;
bool checkRotObstacle = false;
double adjacentRange = 3.5;
double obstacleHeightThre = 0.2;
double groundHeightThre = 0.1;
double costHeightThre = 0.1;
double costScore = 0.02;
double obstacleInflationRadius = 0.12;
double inflatedObstaclePenalty = 0.35;
double centerPathBias = 0.25;
double pathContinuityWeight = 0.35;
double groupContinuityWeight = 0.15;
double sideSwitchPenalty = 0.4;
double largeSwitchAngleDeg = 50.0;
double holdPathScoreRatio = 0.92;
bool useCost = false;
static constexpr int laserCloudStackNum = 1;
int laserCloudCount = 0;
int pointPerPathThre = 2;
double minRelZ = -0.5;
double maxRelZ = 0.25;
double maxSpeed = 1.0;
double dirWeight = 0.02;
double dirThre = 90.0;
bool dirToVehicle = false;
double pathScale = 1.0;
double minPathScale = 0.75;
double pathScaleStep = 0.25;
bool pathScaleBySpeed = true;
double minPathRange = 1.0;
double pathRangeStep = 0.5;
bool pathRangeBySpeed = true;
bool pathCropByGoal = true;
bool autonomyMode = false;
double autonomySpeed = 1.0;
double goalCloseDis = 0.4;
double goalClearRange = 0.5;
double goalX = 0;
double goalY = 0;
double goalYaw = 0;
double globalPathLookAhead = 2.0;
double globalPathGoalSwitchDis = 1.5;
double controlLookAheadDis = 0.5;
double forwardAlignAngle = 8.0;
double yawRateGain = 1.8;
double maxYawRate = 20.0;
double maxAccel = 0.3;
double maxYawAccel = 40.0;
double goalStopDistance = 0.2;
double goalSlowDistance = 1.0;
double planTimeout = 0.5;
double controlFrequency = 100.0;
double nearGoalEnterDistance = 0.5;
double nearGoalExitDistance = 0.8;
double nearGoalXYTolerance = 0.2;
double nearGoalYawToleranceDeg = 5.0;
double nearGoalMinSpeed = 0.08;
double nearGoalMinYawRateDeg = 3.0;
double nearGoalPositionGain = 0.8;
double nearGoalYawGain = 1.2;
double nearGoalObstacleCheckRange = 0.2;
double nearGoalObstacleCheckMargin = 0.1;

float speedRatio = 0;
float desiredDir = 0;
float commandedSpeed = 0;
float commandedSideSpeed = 0;
float commandedYawRate = 0;
int safetyStop = 0;
bool goalReceived = false;
bool goalValid = false;
bool selectedPathValid = false;
bool selectedPathReverseMode = false;
double selectedPathTime = 0;
float selectedPathVehicleX = 0;
float selectedPathVehicleY = 0;
float selectedPathVehicleYaw = 0;
bool nearGoalControlMode = false;

static constexpr int pathNum = 343;
static constexpr int groupNum = 7;
float gridVoxelSize = 0.02;
float searchRadius = 0.45;
float gridVoxelOffsetX = 3.2;
float gridVoxelOffsetY = 4.5;
static constexpr int gridVoxelNumX = 161;
static constexpr int gridVoxelNumY = 451;
static constexpr int gridVoxelNum = gridVoxelNumX * gridVoxelNumY;

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud{
    new pcl::PointCloud<pcl::PointXYZI>()};
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCrop{
    new pcl::PointCloud<pcl::PointXYZI>()};
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudDwz{
    new pcl::PointCloud<pcl::PointXYZI>()};
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud{
    new pcl::PointCloud<pcl::PointXYZI>()};
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudCrop{
    new pcl::PointCloud<pcl::PointXYZI>()};
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudDwz{
    new pcl::PointCloud<pcl::PointXYZI>()};
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudStack[laserCloudStackNum];
pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloud{
    new pcl::PointCloud<pcl::PointXYZI>()};
pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloudCrop{
    new pcl::PointCloud<pcl::PointXYZI>()};
pcl::PointCloud<pcl::PointXYZI>::Ptr boundaryCloud{
    new pcl::PointCloud<pcl::PointXYZI>()};
pcl::PointCloud<pcl::PointXYZI>::Ptr addedObstacles{
    new pcl::PointCloud<pcl::PointXYZI>()};
pcl::PointCloud<pcl::PointXYZ>::Ptr startPaths[groupNum];
common_struct::Path globalReferencePath;
common_struct::Path selectedPath;
bool hasGlobalReferencePath = false;
#if PLOTPATHSET == 1
pcl::PointCloud<pcl::PointXYZI>::Ptr paths[pathNum];
pcl::PointCloud<pcl::PointXYZI>::Ptr freePaths{
    new pcl::PointCloud<pcl::PointXYZI>()};
#endif

int pathList[pathNum] = {0};
float endDirPathList[pathNum] = {0};
int clearPathList[36 * pathNum] = {0};
int inflatedPathList[36 * pathNum] = {0};
float pathPenaltyList[36 * pathNum] = {0};
float clearPathPerGroupScore[36 * groupNum] = {0};
std::vector<int> correspondences[gridVoxelNum];

bool newLaserCloud = false;
bool newTerrainCloud = false;

double odomTime = 0;

float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
int lastSelectedRotDir = -1;
int lastSelectedPathGroup = -1;
bool hasLastSelection = false;

pcl::VoxelGrid<pcl::PointXYZI> laserDwzFilter, terrainDwzFilter;

explicit Impl(const LocalPlannerConfig& config)
{
  pathFolder = config.path_folder;
  vehicleLength = config.vehicle_length;
  vehicleWidth = config.vehicle_width;
  sensorOffsetX = config.sensor_offset_x;
  sensorOffsetY = config.sensor_offset_y;
  twoWayDrive = config.two_way_drive;
  laserVoxelSize = config.laser_voxel_size;
  terrainVoxelSize = config.terrain_voxel_size;
  useTerrainAnalysis = config.use_terrain_analysis;
  checkObstacle = config.check_obstacle;
  checkRotObstacle = config.check_rotation_obstacle;
  adjacentRange = config.adjacent_range;
  obstacleHeightThre = config.obstacle_height_threshold;
  groundHeightThre = config.ground_height_threshold;
  costHeightThre = config.cost_height_threshold;
  costScore = config.cost_score;
  obstacleInflationRadius = config.obstacle_inflation_radius;
  inflatedObstaclePenalty = config.inflated_obstacle_penalty;
  centerPathBias = config.center_path_bias;
  pathContinuityWeight = config.path_continuity_weight;
  groupContinuityWeight = config.group_continuity_weight;
  sideSwitchPenalty = config.side_switch_penalty;
  largeSwitchAngleDeg = config.large_switch_angle_degrees;
  holdPathScoreRatio = config.hold_path_score_ratio;
  useCost = config.use_cost;
  pointPerPathThre = config.point_per_path_threshold;
  minRelZ = config.min_relative_z;
  maxRelZ = config.max_relative_z;
  maxSpeed = config.max_speed;
  dirWeight = config.direction_weight;
  dirThre = config.direction_threshold;
  dirToVehicle = config.direction_to_vehicle;
  pathScale = config.path_scale;
  minPathScale = config.min_path_scale;
  pathScaleStep = config.path_scale_step;
  pathScaleBySpeed = config.path_scale_by_speed;
  minPathRange = config.min_path_range;
  pathRangeStep = config.path_range_step;
  pathRangeBySpeed = config.path_range_by_speed;
  pathCropByGoal = config.path_crop_by_goal;
  autonomyMode = config.autonomy_mode;
  autonomySpeed = config.autonomy_speed;
  goalClearRange = config.goal_clear_range;
  goalX = config.goal_x;
  goalY = config.goal_y;
  globalPathLookAhead = config.global_path_look_ahead;
  globalPathGoalSwitchDis = config.global_path_goal_switch_distance;
  controlLookAheadDis = config.control_look_ahead_distance;
  forwardAlignAngle = config.forward_align_angle_degrees;
  yawRateGain = config.yaw_rate_gain;
  maxYawRate = config.max_yaw_rate_degrees;
  maxAccel = config.max_acceleration;
  maxYawAccel = config.max_yaw_acceleration_degrees;
  goalStopDistance = config.goal_stop_distance;
  goalSlowDistance = config.goal_slow_distance;
  planTimeout = config.plan_timeout;
  controlFrequency = config.control_frequency;
  nearGoalEnterDistance = config.near_goal_enter_distance;
  nearGoalExitDistance = config.near_goal_exit_distance;
  nearGoalXYTolerance = config.near_goal_xy_tolerance;
  nearGoalYawToleranceDeg = config.near_goal_yaw_tolerance_degrees;
  nearGoalMinSpeed = config.near_goal_min_speed;
  nearGoalMinYawRateDeg = config.near_goal_min_yaw_rate_degrees;
  nearGoalPositionGain = config.near_goal_position_gain;
  nearGoalYawGain = config.near_goal_yaw_gain;
  nearGoalObstacleCheckRange = config.near_goal_obstacle_check_range;
  nearGoalObstacleCheckMargin = config.near_goal_obstacle_check_margin;

  goalStopDistance = std::max(0.2, goalStopDistance);
  goalSlowDistance = std::max(goalStopDistance + 0.05, goalSlowDistance);
  controlFrequency = std::max(10.0, controlFrequency);
  nearGoalExitDistance = std::max(nearGoalEnterDistance,
                                  nearGoalExitDistance);
  nearGoalYawToleranceDeg = std::max(0.1, nearGoalYawToleranceDeg);
  nearGoalMinSpeed = std::max(0.0, nearGoalMinSpeed);
  nearGoalMinYawRateDeg = std::max(0.0, nearGoalMinYawRateDeg);
  nearGoalObstacleCheckRange = std::max(0.0, nearGoalObstacleCheckRange);
  nearGoalObstacleCheckMargin = std::max(0.0,
                                         nearGoalObstacleCheckMargin);
  if (pathFolder.empty()) {
    throw std::invalid_argument("local planner path_folder is empty");
  }
  initializePathData();
}

static double yawFromQuaternion(const common_struct::Quaternion& quaternion)
{
  return atan2(2.0 * (quaternion.w * quaternion.z +
                      quaternion.x * quaternion.y),
               1.0 - 2.0 * (quaternion.y * quaternion.y +
                            quaternion.z * quaternion.z));
}

static void quaternionToRpy(const common_struct::Quaternion& quaternion,
                            double& roll, double& pitch, double& yaw)
{
  roll = atan2(2.0 * (quaternion.w * quaternion.x +
                      quaternion.y * quaternion.z),
               1.0 - 2.0 * (quaternion.x * quaternion.x +
                            quaternion.y * quaternion.y));
  const double sinPitch = 2.0 * (quaternion.w * quaternion.y -
                                 quaternion.z * quaternion.x);
  pitch = fabs(sinPitch) >= 1.0
              ? copysign(PI / 2.0, sinPitch)
              : asin(sinPitch);
  yaw = yawFromQuaternion(quaternion);
}

struct PathSelectionDecision
{
  int selectedGroupIndex;
  bool reverseMode;

  PathSelectionDecision()
      : selectedGroupIndex(-1), reverseMode(false)
  {
  }
};

float normalizeAngleDeg(float angle)
{
  while (angle > 180.0) angle -= 360.0;
  while (angle < -180.0) angle += 360.0;
  return angle;
}

float angleDiffDeg(float a, float b)
{
  return fabs(normalizeAngleDeg(a - b));
}

bool hasDirectionSign(float angleDeg)
{
  return fabs(angleDeg) > 10.0;
}

bool isReverseRotation(int rotDir)
{
  const float rotDeg = normalizeAngleDeg(10.0 * rotDir - 180.0);
  return fabs(rotDeg) > 90.0;
}





void SetOdometry(double timestampSeconds, const common_struct::Pose& pose)
{
  odomTime = timestampSeconds;

  double roll, pitch, yaw;
  quaternionToRpy(pose.orientation, roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY;
  vehicleY = pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY;
  vehicleZ = pose.position.z;
}

void SetRegisteredScan(
    const pcl::PointCloud<pcl::PointXYZI>& registeredScan)
{
  if (!useTerrainAnalysis) {
    *laserCloud = registeredScan;

    pcl::PointXYZI point;
    laserCloudCrop->clear();
    int laserCloudSize = laserCloud->points.size();
    for (int i = 0; i < laserCloudSize; i++) {
      point = laserCloud->points[i];

      float pointX = point.x;
      float pointY = point.y;
      float pointZ = point.z;

      float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) + (pointY - vehicleY) * (pointY - vehicleY));
      if (dis < adjacentRange) {
        point.x = pointX;
        point.y = pointY;
        point.z = pointZ;
        laserCloudCrop->push_back(point);
      }
    }

    laserCloudDwz->clear();
    laserDwzFilter.setInputCloud(laserCloudCrop);
    laserDwzFilter.filter(*laserCloudDwz);

    newLaserCloud = true;
  }
}

void SetTerrain(const pcl::PointCloud<pcl::PointXYZI>& terrain)
{
  if (useTerrainAnalysis) {
    *terrainCloud = terrain;

    pcl::PointXYZI point;
    terrainCloudCrop->clear();
    int terrainCloudSize = terrainCloud->points.size();
    for (int i = 0; i < terrainCloudSize; i++) {
      point = terrainCloud->points[i];

      float pointX = point.x;
      float pointY = point.y;
      float pointZ = point.z;

      float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) + (pointY - vehicleY) * (pointY - vehicleY));
      if (dis < adjacentRange && (point.intensity > obstacleHeightThre || useCost)) {
        point.x = pointX;
        point.y = pointY;
        point.z = pointZ;
        terrainCloudCrop->push_back(point);
      }
    }

    terrainCloudDwz->clear();
    terrainDwzFilter.setInputCloud(terrainCloudCrop);
    terrainDwzFilter.filter(*terrainCloudDwz);

    newTerrainCloud = true;
  }
}

void SetGoal(const common_struct::Pose& goal)
{
  goalX = goal.position.x;
  goalY = goal.position.y;
  goalYaw = yawFromQuaternion(goal.orientation);
  goalReceived = true;
}

void SetGlobalPath(const common_struct::Path& pathIn)
{
  globalReferencePath = pathIn;
  hasGlobalReferencePath = !globalReferencePath.poses.empty();
}

void SetSpeed(double speed)
{
  speedRatio = speed / maxSpeed;
  if (speedRatio < 0) speedRatio = 0;
  else if (speedRatio > 1.0) speedRatio = 1.0;
}

void SetBoundary(const common_struct::PolygonStamped& boundary)
{
  boundaryCloud->clear();
  pcl::PointXYZI point, point1, point2;
  int boundarySize = boundary.points.size();

  if (boundarySize >= 1) {
    point2.x = boundary.points[0].x;
    point2.y = boundary.points[0].y;
    point2.z = boundary.points[0].z;
  }

  for (int i = 0; i < boundarySize; i++) {
    point1 = point2;

    point2.x = boundary.points[i].x;
    point2.y = boundary.points[i].y;
    point2.z = boundary.points[i].z;

    if (point1.z == point2.z) {
      float disX = point1.x - point2.x;
      float disY = point1.y - point2.y;
      float dis = sqrt(disX * disX + disY * disY);

      int pointNum = int(dis / terrainVoxelSize) + 1;
      for (int pointID = 0; pointID < pointNum; pointID++) {
        point.x = float(pointID) / float(pointNum) * point1.x + (1.0 - float(pointID) / float(pointNum)) * point2.x;
        point.y = float(pointID) / float(pointNum) * point1.y + (1.0 - float(pointID) / float(pointNum)) * point2.y;
        point.z = 0;
        point.intensity = 100.0;

        for (int j = 0; j < pointPerPathThre; j++) {
          boundaryCloud->push_back(point);
        }
      }
    }
  }
}

void SetAddedObstacles(
    const pcl::PointCloud<pcl::PointXYZI>& obstacleCloud)
{
  *addedObstacles = obstacleCloud;

  int addedObstaclesSize = addedObstacles->points.size();
  for (int i = 0; i < addedObstaclesSize; i++) {
    addedObstacles->points[i].intensity = 200.0;
  }
}

void SetObstacleChecking(bool enabled)
{
  checkObstacle = enabled;
}

void SetSafetyStop(std::int8_t stopMask)
{
  safetyStop = stopMask;
}

void SetGoalValid(bool valid)
{
  goalValid = valid;
}

common_struct::Twist stopCommand()
{
  commandedSpeed = 0;
  commandedSideSpeed = 0;
  commandedYawRate = 0;
  return common_struct::Twist();
}

common_struct::Twist controlSelectedPath(
    const common_struct::Path& selectedPath)
{
  if (!selectedPathValid || !goalReceived || !goalValid || selectedPath.poses.size() <= 1) {
    return stopCommand();
  }

  const double goalDistance = hypot(goalX - vehicleX, goalY - vehicleY);
  if (goalDistance <= goalStopDistance) {
    return stopCommand();
  }

  const float vehicleXRel =
      cos(selectedPathVehicleYaw) * (vehicleX - selectedPathVehicleX) +
      sin(selectedPathVehicleYaw) * (vehicleY - selectedPathVehicleY);
  const float vehicleYRel =
      -sin(selectedPathVehicleYaw) * (vehicleX - selectedPathVehicleX) +
      cos(selectedPathVehicleYaw) * (vehicleY - selectedPathVehicleY);

  int trackingPoint = selectedPath.poses.size() - 1;
  for (size_t i = 0; i < selectedPath.poses.size(); ++i) {
    const double dx = selectedPath.poses[i].pose.position.x - vehicleXRel;
    const double dy = selectedPath.poses[i].pose.position.y - vehicleYRel;
    if (hypot(dx, dy) >= controlLookAheadDis) {
      trackingPoint = i;
      break;
    }
  }

  const double dx = selectedPath.poses[trackingPoint].pose.position.x - vehicleXRel;
  const double dy = selectedPath.poses[trackingPoint].pose.position.y - vehicleYRel;
  const double pathHeading = atan2(dy, dx);
  const double relativeYaw = normalizeAngleDeg((vehicleYaw - selectedPathVehicleYaw) * 180.0 / PI)
                           * PI / 180.0;
  const double trackingHeading = selectedPathReverseMode ? (relativeYaw + PI) : relativeYaw;
  const double headingError = atan2(sin(pathHeading - trackingHeading),
                                    cos(pathHeading - trackingHeading));

  double targetYawRate = yawRateGain * headingError;
  const double maxYawRateRad = maxYawRate * PI / 180.0;
  if (targetYawRate > maxYawRateRad) targetYawRate = maxYawRateRad;
  if (targetYawRate < -maxYawRateRad) targetYawRate = -maxYawRateRad;

  // Forward motion is permitted only when the selected free path is almost
  // straight ahead. A left/right free path causes rotation first, never
  // forward motion in the old heading.
  double targetSpeed = 0;
  if (fabs(headingError) <= forwardAlignAngle * PI / 180.0) {
    const double goalScale =
        std::max(0.0, std::min(1.0,
            (goalDistance - goalStopDistance) / (goalSlowDistance - goalStopDistance)));
    const double signedSpeed = maxSpeed * speedRatio * goalScale;
    targetSpeed = selectedPathReverseMode ? -signedSpeed : signedSpeed;
  }

  const double dt = 1.0 / controlFrequency;
  if (targetSpeed <= 0) {
    // No forward free path or not aligned: stop translation immediately.
    commandedSpeed = 0;
  } else if (commandedSpeed < targetSpeed) {
    commandedSpeed = std::min<float>(targetSpeed, commandedSpeed + maxAccel * dt);
  } else {
    commandedSpeed = std::max<float>(targetSpeed, commandedSpeed - maxAccel * dt);
  }

  const double maxYawStep = maxYawAccel * PI / 180.0 * dt;
  if (commandedYawRate < targetYawRate) {
    commandedYawRate = std::min<float>(targetYawRate, commandedYawRate + maxYawStep);
  } else {
    commandedYawRate = std::max<float>(targetYawRate, commandedYawRate - maxYawStep);
  }

  common_struct::Twist command;
  command.linear.x = commandedSpeed;
  command.angular.z = commandedYawRate;

  if ((safetyStop & 1) && command.linear.x > 0) command.linear.x = 0;
  if ((safetyStop & 2) && command.linear.x < 0) command.linear.x = 0;
  if ((safetyStop & 4) && command.angular.z > 0) command.angular.z = 0;
  if ((safetyStop & 8) && command.angular.z < 0) command.angular.z = 0;
  commandedSpeed = command.linear.x;
  commandedYawRate = command.angular.z;
  return command;
}

int readPlyHeader(FILE *filePtr)
{
  char str[50];
  int val, pointNum;
  string strCur, strLast;
  while (strCur != "end_header") {
    val = fscanf(filePtr, "%s", str);
    if (val != 1) {
      throw std::runtime_error("failed to read local planner PLY header");
    }

    strLast = strCur;
    strCur = string(str);

    if (strCur == "vertex" && strLast == "element") {
      val = fscanf(filePtr, "%d", &pointNum);
      if (val != 1) {
        throw std::runtime_error("invalid local planner PLY vertex count");
      }
    }
  }

  return pointNum;
}

void readStartPaths()
{
  string fileName = pathFolder + "/startPaths.ply";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    throw std::runtime_error("cannot read local planner path file: " +
                             fileName);
  }

  int pointNum = readPlyHeader(filePtr);

  pcl::PointXYZ point;
  int val1, val2, val3, val4, groupID;
  for (int i = 0; i < pointNum; i++) {
    val1 = fscanf(filePtr, "%f", &point.x);
    val2 = fscanf(filePtr, "%f", &point.y);
    val3 = fscanf(filePtr, "%f", &point.z);
    val4 = fscanf(filePtr, "%d", &groupID);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1) {
      fclose(filePtr);
      throw std::runtime_error("invalid local planner path file: " +
                               fileName);
    }

    if (groupID >= 0 && groupID < groupNum) {
      startPaths[groupID]->push_back(point);
    }
  }

  fclose(filePtr);
}

#if PLOTPATHSET == 1
void readPaths()
{
  string fileName = pathFolder + "/paths.ply";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    throw std::runtime_error("cannot read local planner path file: " +
                             fileName);
  }

  int pointNum = readPlyHeader(filePtr);

  pcl::PointXYZI point;
  int pointSkipNum = 30;
  int pointSkipCount = 0;
  int val1, val2, val3, val4, val5, pathID;
  for (int i = 0; i < pointNum; i++) {
    val1 = fscanf(filePtr, "%f", &point.x);
    val2 = fscanf(filePtr, "%f", &point.y);
    val3 = fscanf(filePtr, "%f", &point.z);
    val4 = fscanf(filePtr, "%d", &pathID);
    val5 = fscanf(filePtr, "%f", &point.intensity);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
      fclose(filePtr);
      throw std::runtime_error("invalid local planner path file: " +
                               fileName);
    }

    if (pathID >= 0 && pathID < pathNum) {
      pointSkipCount++;
      if (pointSkipCount > pointSkipNum) {
        paths[pathID]->push_back(point);
        pointSkipCount = 0;
      }
    }
  }

  fclose(filePtr);
}
#endif

void readPathList()
{
  string fileName = pathFolder + "/pathList.ply";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    throw std::runtime_error("cannot read local planner path file: " +
                             fileName);
  }

  if (pathNum != readPlyHeader(filePtr)) {
    fclose(filePtr);
    throw std::runtime_error("incorrect local planner path count in: " +
                             fileName);
  }

  int val1, val2, val3, val4, val5, pathID, groupID;
  float endX, endY, endZ;
  for (int i = 0; i < pathNum; i++) {
    val1 = fscanf(filePtr, "%f", &endX);
    val2 = fscanf(filePtr, "%f", &endY);
    val3 = fscanf(filePtr, "%f", &endZ);
    val4 = fscanf(filePtr, "%d", &pathID);
    val5 = fscanf(filePtr, "%d", &groupID);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
      fclose(filePtr);
      throw std::runtime_error("invalid local planner path file: " +
                               fileName);
    }

    if (pathID >= 0 && pathID < pathNum && groupID >= 0 && groupID < groupNum) {
      pathList[pathID] = groupID;
      endDirPathList[pathID] = 2.0 * atan2(endY, endX) * 180 / PI;
    }
  }

  fclose(filePtr);
}

void readCorrespondences()
{
  string fileName = pathFolder + "/correspondences.txt";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    throw std::runtime_error("cannot read local planner correspondence file: " +
                             fileName);
  }

  int val1, gridVoxelID, pathID;
  for (int i = 0; i < gridVoxelNum; i++) {
    val1 = fscanf(filePtr, "%d", &gridVoxelID);
    if (val1 != 1) {
      fclose(filePtr);
      throw std::runtime_error(
          "invalid local planner correspondence file: " + fileName);
    }

    while (1) {
      val1 = fscanf(filePtr, "%d", &pathID);
      if (val1 != 1) {
        fclose(filePtr);
        throw std::runtime_error(
            "invalid local planner correspondence file: " + fileName);
      }

      if (pathID != -1) {
        if (gridVoxelID >= 0 && gridVoxelID < gridVoxelNum && pathID >= 0 && pathID < pathNum) {
          correspondences[gridVoxelID].push_back(pathID);
        }
      } else {
        break;
      }
    }
  }

  fclose(filePtr);
}

struct VehicleTrig
{
  float sinYaw;
  float cosYaw;
};

struct RotationObstacleLimits
{
  float minObsAngCW;
  float minObsAngCCW;
};

void initializePathData()
{
  if (autonomyMode) {
    speedRatio = autonomySpeed / maxSpeed;
    if (speedRatio < 0) speedRatio = 0;
    else if (speedRatio > 1.0) speedRatio = 1.0;
  }

  for (int i = 0; i < laserCloudStackNum; i++) {
    laserCloudStack[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }
  for (int i = 0; i < groupNum; i++) {
    startPaths[i].reset(new pcl::PointCloud<pcl::PointXYZ>());
  }
#if PLOTPATHSET == 1
  for (int i = 0; i < pathNum; i++) {
    paths[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }
#endif
  for (int i = 0; i < gridVoxelNum; i++) {
    correspondences[i].resize(0);
  }

  laserDwzFilter.setLeafSize(laserVoxelSize, laserVoxelSize, laserVoxelSize);
  terrainDwzFilter.setLeafSize(terrainVoxelSize, terrainVoxelSize, terrainVoxelSize);

  readStartPaths();
#if PLOTPATHSET == 1
  readPaths();
#endif
  readPathList();
  readCorrespondences();

}

VehicleTrig makeVehicleTrig()
{
  VehicleTrig trig;
  trig.sinYaw = sin(vehicleYaw);
  trig.cosYaw = cos(vehicleYaw);
  return trig;
}

void updatePlannerCloud()
{
  if (newLaserCloud) {
    newLaserCloud = false;

    laserCloudStack[laserCloudCount]->clear();
    *laserCloudStack[laserCloudCount] = *laserCloudDwz;
    laserCloudCount = (laserCloudCount + 1) % laserCloudStackNum;

    plannerCloud->clear();
    for (int i = 0; i < laserCloudStackNum; i++) {
      *plannerCloud += *laserCloudStack[i];
    }
  }

  if (newTerrainCloud) {
    newTerrainCloud = false;
    plannerCloud->clear();
    *plannerCloud = *terrainCloudDwz;
  }
}

void appendWorldCloudInVehicleFrame(const pcl::PointCloud<pcl::PointXYZI>& sourceCloud,
                                    const VehicleTrig& trig,
                                    bool filterHeight,
                                    bool subtractVehicleZ)
{
  pcl::PointXYZI point;
  const int cloudSize = sourceCloud.points.size();
  for (int i = 0; i < cloudSize; i++) {
    const float pointX1 = sourceCloud.points[i].x - vehicleX;
    const float pointY1 = sourceCloud.points[i].y - vehicleY;
    const float pointZ1 = subtractVehicleZ ? sourceCloud.points[i].z - vehicleZ : sourceCloud.points[i].z;

    point.x = pointX1 * trig.cosYaw + pointY1 * trig.sinYaw;
    point.y = -pointX1 * trig.sinYaw + pointY1 * trig.cosYaw;
    point.z = pointZ1;
    point.intensity = sourceCloud.points[i].intensity;

    const float dis = sqrt(point.x * point.x + point.y * point.y);
    if (dis < adjacentRange && (!filterHeight || (point.z > minRelZ && point.z < maxRelZ) || useTerrainAnalysis)) {
      plannerCloudCrop->push_back(point);
    }
  }
}

void buildPlannerCloudCrop(const VehicleTrig& trig)
{
  plannerCloudCrop->clear();
  appendWorldCloudInVehicleFrame(*plannerCloud, trig, true, true);
  appendWorldCloudInVehicleFrame(*boundaryCloud, trig, false, false);
  appendWorldCloudInVehicleFrame(*addedObstacles, trig, false, false);
}

float initialPathRange()
{
  float range = adjacentRange;
  if (pathRangeBySpeed) range = adjacentRange * speedRatio;
  if (range < minPathRange) range = minPathRange;
  return range;
}

void updateAutonomyTarget(const VehicleTrig& trig, float& relativeGoalDis)
{
  if (!autonomyMode) {
    return;
  }

  const float relativeGoalX = ((goalX - vehicleX) * trig.cosYaw + (goalY - vehicleY) * trig.sinYaw);
  const float relativeGoalY = (-(goalX - vehicleX) * trig.sinYaw + (goalY - vehicleY) * trig.cosYaw);

  relativeGoalDis = sqrt(relativeGoalX * relativeGoalX + relativeGoalY * relativeGoalY);
  bool useGlobalReference = false;

  if (hasGlobalReferencePath && relativeGoalDis > globalPathGoalSwitchDis) {
    double nearestDis = std::numeric_limits<double>::infinity();
    int nearestIdx = -1;
    const int pathSize = globalReferencePath.poses.size();
    for (int i = 0; i < pathSize; ++i) {
      const double dx = globalReferencePath.poses[i].pose.position.x - vehicleX;
      const double dy = globalReferencePath.poses[i].pose.position.y - vehicleY;
      const double dis = hypot(dx, dy);
      if (dis < nearestDis) {
        nearestDis = dis;
        nearestIdx = i;
      }
    }

    if (nearestIdx >= 0) {
      double accumDis = 0.0;
      int refIdx = nearestIdx;
      for (int i = nearestIdx + 1; i < pathSize; ++i) {
        const double dx = globalReferencePath.poses[i].pose.position.x - globalReferencePath.poses[refIdx].pose.position.x;
        const double dy = globalReferencePath.poses[i].pose.position.y - globalReferencePath.poses[refIdx].pose.position.y;
        accumDis += hypot(dx, dy);
        refIdx = i;
        if (accumDis >= globalPathLookAhead) {
          break;
        }
      }

      const float refGoalX = ((globalReferencePath.poses[refIdx].pose.position.x - vehicleX) * trig.cosYaw +
                              (globalReferencePath.poses[refIdx].pose.position.y - vehicleY) * trig.sinYaw);
      const float refGoalY = (-(globalReferencePath.poses[refIdx].pose.position.x - vehicleX) * trig.sinYaw +
                              (globalReferencePath.poses[refIdx].pose.position.y - vehicleY) * trig.cosYaw);
      const float refGoalDis = sqrt(refGoalX * refGoalX + refGoalY * refGoalY);

      if (refGoalDis > 0.1) {
        relativeGoalDis = refGoalDis;
        desiredDir = atan2(refGoalY, refGoalX) * 180 / PI;
        useGlobalReference = true;
      }
    }
  }

  if (!useGlobalReference) {
    desiredDir = atan2(relativeGoalY, relativeGoalX) * 180 / PI;
  }

  if (!twoWayDrive) {
    if (desiredDir > 90.0) desiredDir = 90.0;
    else if (desiredDir < -90.0) desiredDir = -90.0;
  }
}

bool isRotDirAllowed(int rotDir)
{
  const float rotDeg = 10.0 * rotDir - 180.0;
  float angDiff = fabs(desiredDir - rotDeg);
  if (angDiff > 180.0) {
    angDiff = 360.0 - angDiff;
  }
  return !((angDiff > dirThre && !dirToVehicle) ||
      (fabs(rotDeg) > dirThre && fabs(desiredDir) <= 90.0 && dirToVehicle) ||
      ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre) && fabs(desiredDir) > 90.0 && dirToVehicle));
}

void resetEvaluationBuffers()
{
  for (int i = 0; i < 36 * pathNum; i++) {
    clearPathList[i] = 0;
    inflatedPathList[i] = 0;
    pathPenaltyList[i] = 0;
  }
  for (int i = 0; i < 36 * groupNum; i++) {
    clearPathPerGroupScore[i] = 0;
  }
}

void projectObstaclesToPathGrid(float pathRange, float relativeGoalDis, RotationObstacleLimits& limits)
{
  limits.minObsAngCW = -180.0;
  limits.minObsAngCCW = 180.0;

  const float diameter = sqrt(vehicleLength / 2.0 * vehicleLength / 2.0 + vehicleWidth / 2.0 * vehicleWidth / 2.0);
  const float angOffset = atan2(vehicleWidth, vehicleLength) * 180.0 / PI;
  const int plannerCloudCropSize = plannerCloudCrop->points.size();
  const int inflateStep = int(obstacleInflationRadius / gridVoxelSize);

  for (int i = 0; i < plannerCloudCropSize; i++) {
    const float x = plannerCloudCrop->points[i].x / pathScale;
    const float y = plannerCloudCrop->points[i].y / pathScale;
    const float h = plannerCloudCrop->points[i].intensity;
    const float dis = sqrt(x * x + y * y);

    if (dis < pathRange / pathScale &&
        (dis <= (relativeGoalDis + goalClearRange) / pathScale || !pathCropByGoal) &&
        checkObstacle) {
      for (int rotDir = 0; rotDir < 36; rotDir++) {
        if (!isRotDirAllowed(rotDir)) {
          continue;
        }

        const float rotAng = (10.0 * rotDir - 180.0) * PI / 180;
        const float x2 = cos(rotAng) * x + sin(rotAng) * y;
        const float y2 = -sin(rotAng) * x + cos(rotAng) * y;
        const float scaleY = x2 / gridVoxelOffsetX + searchRadius / gridVoxelOffsetY
                           * (gridVoxelOffsetX - x2) / gridVoxelOffsetX;
        const int indX = int((gridVoxelOffsetX + gridVoxelSize / 2 - x2) / gridVoxelSize);
        const int indY = int((gridVoxelOffsetY + gridVoxelSize / 2 - y2 / scaleY) / gridVoxelSize);

        for (int dx = -inflateStep; dx <= inflateStep; dx++) {
          for (int dy = -inflateStep; dy <= inflateStep; dy++) {
            if ((dx != 0 || dy != 0) && dx * dx + dy * dy > inflateStep * inflateStep) {
              continue;
            }

            const int voxelX = indX + dx;
            const int voxelY = indY + dy;
            if (voxelX < 0 || voxelX >= gridVoxelNumX || voxelY < 0 || voxelY >= gridVoxelNumY) {
              continue;
            }

            const int ind = gridVoxelNumY * voxelX + voxelY;
            const int blockedPathByVoxelNum = correspondences[ind].size();
            for (int j = 0; j < blockedPathByVoxelNum; j++) {
              const int pathIdx = pathNum * rotDir + correspondences[ind][j];
              if (h > obstacleHeightThre || !useTerrainAnalysis) {
                if (dx == 0 && dy == 0) {
                  clearPathList[pathIdx]++;
                } else {
                  inflatedPathList[pathIdx]++;
                }
              } else if (pathPenaltyList[pathIdx] < h && h > groundHeightThre) {
                pathPenaltyList[pathIdx] = h;
              }
            }
          }
        }
      }
    }

    if (dis < diameter / pathScale &&
        (fabs(x) > vehicleLength / pathScale / 2.0 || fabs(y) > vehicleWidth / pathScale / 2.0) &&
        (h > obstacleHeightThre || !useTerrainAnalysis) && checkRotObstacle) {
      const float angObs = atan2(y, x) * 180.0 / PI;
      if (angObs > 0) {
        if (limits.minObsAngCCW > angObs - angOffset) limits.minObsAngCCW = angObs - angOffset;
        if (limits.minObsAngCW < angObs + angOffset - 180.0) limits.minObsAngCW = angObs + angOffset - 180.0;
      } else {
        if (limits.minObsAngCW < angObs + angOffset) limits.minObsAngCW = angObs + angOffset;
        if (limits.minObsAngCCW > 180.0 + angObs - angOffset) limits.minObsAngCCW = 180.0 + angObs - angOffset;
      }
    }
  }

  if (limits.minObsAngCW > 0) limits.minObsAngCW = 0;
  if (limits.minObsAngCCW < 0) limits.minObsAngCCW = 0;
}

void scoreCandidatePaths(float relativeGoalDis)
{
  for (int i = 0; i < 36 * pathNum; i++) {
    const int rotDir = int(i / pathNum);
    const float candidateRotDeg = normalizeAngleDeg(10.0 * rotDir - 180.0);
    if (!isRotDirAllowed(rotDir)) {
      continue;
    }

    if (clearPathList[i] < pointPerPathThre) {
      float penaltyScore = 1.0 - pathPenaltyList[i] / costHeightThre;
      if (penaltyScore < costScore) penaltyScore = costScore;
      const float inflationPenaltyScore = 1.0 / (1.0 + inflatedObstaclePenalty * inflatedPathList[i]);
      const float dirDiff = angleDiffDeg(desiredDir, endDirPathList[i % pathNum] + candidateRotDeg);

      float rotDirW;
      if (rotDir < 18) rotDirW = fabs(fabs(rotDir - 9) + 1);
      else rotDirW = fabs(fabs(rotDir - 27) + 1);

      const int pathGroup = pathList[i % pathNum];
      const float groupDirW = 4  - fabs(pathGroup - 3);
      const float centerBiasScore = 1.0 + centerPathBias * (1.0 - fabs(pathGroup - 3) / 3.0);
      float score = (1 - sqrt(sqrt(dirWeight * dirDiff))) * rotDirW * rotDirW * rotDirW * rotDirW
                  * penaltyScore * inflationPenaltyScore * centerBiasScore;
      if (relativeGoalDis < goalCloseDis) {
        score = (1 - sqrt(sqrt(dirWeight * dirDiff))) * groupDirW * groupDirW
              * penaltyScore * inflationPenaltyScore * centerBiasScore;
      }

      if (score > 0 && hasLastSelection) {
        const float lastRotDeg = normalizeAngleDeg(10.0 * lastSelectedRotDir - 180.0);
        const float switchAngle = angleDiffDeg(candidateRotDeg, lastRotDeg);
        const float rotContinuityScore = 1.0 / (1.0 + pathContinuityWeight * switchAngle / 10.0);
        const float groupContinuityScore = 1.0 / (1.0 + groupContinuityWeight * fabs(pathGroup - lastSelectedPathGroup));
        score *= rotContinuityScore * groupContinuityScore;

        if (switchAngle > largeSwitchAngleDeg &&
            hasDirectionSign(candidateRotDeg) &&
            hasDirectionSign(lastRotDeg) &&
            candidateRotDeg * lastRotDeg < 0.0) {
          score *= sideSwitchPenalty;
        }
      }

      if (score > 0) {
        clearPathPerGroupScore[groupNum * rotDir + pathGroup] += score;
      }
    }
  }
}

bool isRotationFeasible(int rotDir, const RotationObstacleLimits& limits)
{
  const float rotAng = (10.0 * rotDir - 180.0) * PI / 180;
  float rotDeg = 10.0 * rotDir;
  if (rotDeg > 180.0) rotDeg -= 360.0;
  return ((rotAng * 180.0 / PI > limits.minObsAngCW && rotAng * 180.0 / PI < limits.minObsAngCCW) ||
      (rotDeg > limits.minObsAngCW && rotDeg < limits.minObsAngCCW && twoWayDrive) || !checkRotObstacle);
}

PathSelectionDecision selectBestGroup(const RotationObstacleLimits& limits)
{
  PathSelectionDecision decision;
  float maxForwardScore = 0;
  float lastForwardSelectionScore = -1.0;
  int selectedForwardGroupID = -1;
  int forwardCandidateCount = 0;
  int reverseCandidateCount = 0;
  int uniqueReverseGroupID = -1;

  for (int i = 0; i < 36 * groupNum; i++) {
    const int rotDir = int(i / groupNum);
    const bool rotationFeasible = isRotationFeasible(rotDir, limits);
    const float score = clearPathPerGroupScore[i];
    if (!rotationFeasible || score <= 0) {
      continue;
    }

    if (isReverseRotation(rotDir)) {
      reverseCandidateCount++;
      uniqueReverseGroupID = i;
      continue;
    }

    forwardCandidateCount++;
    if (hasLastSelection &&
        rotDir == lastSelectedRotDir && (i % groupNum) == lastSelectedPathGroup) {
      lastForwardSelectionScore = score;
    }
    if (maxForwardScore < score) {
      maxForwardScore = score;
      selectedForwardGroupID = i;
    }
  }

  if (forwardCandidateCount > 0) {
    if (hasLastSelection && !isReverseRotation(lastSelectedRotDir) &&
        lastForwardSelectionScore > 0 && maxForwardScore > 0 &&
        lastForwardSelectionScore >= holdPathScoreRatio * maxForwardScore) {
      decision.selectedGroupIndex = groupNum * lastSelectedRotDir + lastSelectedPathGroup;
    } else {
      decision.selectedGroupIndex = selectedForwardGroupID;
    }
    return decision;
  }

  if (reverseCandidateCount == 1) {
    decision.selectedGroupIndex = uniqueReverseGroupID;
    decision.reverseMode = true;
  }

  return decision;
}

void fillSelectedPath(int rotDir, int selectedGroupID, float pathRange,
                      float relativeGoalDis, double nowSeconds,
                      common_struct::Path& path)
{
  const float rotAng = (10.0 * rotDir - 180.0) * PI / 180;
  const int selectedPathLength = startPaths[selectedGroupID]->points.size();
  path.poses.resize(selectedPathLength);

  for (int i = 0; i < selectedPathLength; i++) {
    const float x = startPaths[selectedGroupID]->points[i].x;
    const float y = startPaths[selectedGroupID]->points[i].y;
    const float z = startPaths[selectedGroupID]->points[i].z;
    const float dis = sqrt(x * x + y * y);

    if (dis <= pathRange / pathScale && dis <= relativeGoalDis / pathScale) {
      path.poses[i].pose.position.x = pathScale * (cos(rotAng) * x - sin(rotAng) * y);
      path.poses[i].pose.position.y = pathScale * (sin(rotAng) * x + cos(rotAng) * y);
      path.poses[i].pose.position.z = pathScale * z;
    } else {
      path.poses.resize(i);
      break;
    }
  }

  path.header.timestamp = static_cast<std::uint64_t>(
      std::max(0.0, odomTime) * 1.0e9);
  selectedPathValid = path.poses.size() > 1;
  selectedPathReverseMode = isReverseRotation(rotDir);
  selectedPathTime = nowSeconds;
  selectedPathVehicleX = vehicleX;
  selectedPathVehicleY = vehicleY;
  selectedPathVehicleYaw = vehicleYaw;
}

#if PLOTPATHSET == 1
void buildFreePaths(const RotationObstacleLimits& limits,
                    float pathRange,
                    float relativeGoalDis)
{
  freePaths->clear();
  pcl::PointXYZI point;
  for (int i = 0; i < 36 * pathNum; i++) {
    const int rotDir = int(i / pathNum);
    const float rotAng = (10.0 * rotDir - 180.0) * PI / 180;
    if (!isRotDirAllowed(rotDir) || !isRotationFeasible(rotDir, limits)) {
      continue;
    }

    if (clearPathList[i] < pointPerPathThre) {
      const int freePathLength = paths[i % pathNum]->points.size();
      for (int j = 0; j < freePathLength; j++) {
        point = paths[i % pathNum]->points[j];
        const float x = point.x;
        const float y = point.y;
        const float z = point.z;
        const float dis = sqrt(x * x + y * y);
        if (dis <= pathRange / pathScale &&
            (dis <= (relativeGoalDis + goalClearRange) / pathScale || !pathCropByGoal)) {
          point.x = pathScale * (cos(rotAng) * x - sin(rotAng) * y);
          point.y = pathScale * (sin(rotAng) * x + cos(rotAng) * y);
          point.z = pathScale * z;
          point.intensity = 1.0;
          freePaths->push_back(point);
        }
      }
    }
  }
}

void clearFreePaths()
{
  freePaths->clear();
}
#endif

void buildNoPath(LocalPlannerOutput& output, double nowSeconds,
                 common_struct::Path& path)
{
  hasLastSelection = false;
  lastSelectedRotDir = -1;
  lastSelectedPathGroup = -1;
  path.poses.resize(1);
  path.poses[0].pose.position.x = 0;
  path.poses[0].pose.position.y = 0;
  path.poses[0].pose.position.z = 0;
  path.header.timestamp = static_cast<std::uint64_t>(
      std::max(0.0, odomTime) * 1.0e9);
  selectedPathValid = false;
  selectedPathReverseMode = false;
  selectedPathTime = nowSeconds;
  output.path_updated = true;
  output.path = path;
#if PLOTPATHSET == 1
  clearFreePaths();
  output.free_paths_updated = true;
  output.free_paths = *freePaths;
#endif
}

bool evaluateAndBuildPath(LocalPlannerOutput& output, double nowSeconds,
                          common_struct::Path& path, float pathRange,
                          float relativeGoalDis)
{
  const float defPathScale = pathScale;
  if (pathScaleBySpeed) pathScale = defPathScale * speedRatio;
  if (pathScale < minPathScale) pathScale = minPathScale;

  bool pathFound = false;
  while (pathScale >= minPathScale && pathRange >= minPathRange) {
    resetEvaluationBuffers();

    RotationObstacleLimits limits;
    projectObstaclesToPathGrid(pathRange, relativeGoalDis, limits);
    scoreCandidatePaths(relativeGoalDis);

    const PathSelectionDecision selection = selectBestGroup(limits);
    if (selection.selectedGroupIndex >= 0) {
      int selectedGroupID = selection.selectedGroupIndex;
      const int rotDir = int(selectedGroupID / groupNum);
      selectedGroupID = selectedGroupID % groupNum;
      lastSelectedRotDir = rotDir;
      lastSelectedPathGroup = selectedGroupID;
      hasLastSelection = true;

      fillSelectedPath(rotDir, selectedGroupID, pathRange, relativeGoalDis,
                       nowSeconds, path);
      selectedPathReverseMode = selection.reverseMode;
      output.path_updated = true;
      output.path = path;
#if PLOTPATHSET == 1
      buildFreePaths(limits, pathRange, relativeGoalDis);
      output.free_paths_updated = true;
      output.free_paths = *freePaths;
#endif
      pathFound = true;
      break;
    }

    if (pathScale >= minPathScale + pathScaleStep) {
      pathScale -= pathScaleStep;
      pathRange = adjacentRange * pathScale / defPathScale;
    } else {
      pathRange -= pathRangeStep;
    }
  }

  pathScale = defPathScale;
  return pathFound;
}

void processNewPlanningData(LocalPlannerOutput& output, double nowSeconds,
                            common_struct::Path& path)
{
  updatePlannerCloud();

  const VehicleTrig trig = makeVehicleTrig();
  buildPlannerCloudCrop(trig);

  float pathRange = initialPathRange();
  float relativeGoalDis = adjacentRange;
  updateAutonomyTarget(trig, relativeGoalDis);

  if (!evaluateAndBuildPath(output, nowSeconds, path, pathRange,
                            relativeGoalDis)) {
    buildNoPath(output, nowSeconds, path);
  }
}

double currentGoalDistance()
{
  return hypot(goalX - vehicleX, goalY - vehicleY);
}

bool shouldUseNearGoalControl()
{
  if (!goalReceived || !goalValid) {
    nearGoalControlMode = false;
    return false;
  }

  const double goalDistance = currentGoalDistance();
  if (!nearGoalControlMode && goalDistance <= nearGoalEnterDistance) {
    nearGoalControlMode = true;
  } else if (nearGoalControlMode && goalDistance >= nearGoalExitDistance) {
    nearGoalControlMode = false;
  }
  return nearGoalControlMode;
}

double applyMinCommand(double command, double error, double tolerance, double minAbsCommand)
{
  if (fabs(error) <= tolerance) {
    return 0.0;
  }
  if (command > 0 && command < minAbsCommand) {
    return minAbsCommand;
  }
  if (command < 0 && command > -minAbsCommand) {
    return -minAbsCommand;
  }
  return command;
}

void limitNearGoalCommandByPoint(const pcl::PointXYZI& point,
                                 bool filterHeight,
                                 common_struct::Twist& command)
{
  if (useTerrainAnalysis && point.intensity <= obstacleHeightThre) {
    return;
  }

  const double dx = point.x - vehicleX;
  const double dy = point.y - vehicleY;
  const double x = cos(vehicleYaw) * dx + sin(vehicleYaw) * dy;
  const double y = -sin(vehicleYaw) * dx + cos(vehicleYaw) * dy;
  const double z = point.z - vehicleZ;

  if (filterHeight && !useTerrainAnalysis && (z < minRelZ || z > maxRelZ)) {
    return;
  }

  const double halfLength = vehicleLength / 2.0 + nearGoalObstacleCheckMargin;
  const double halfWidth = vehicleWidth / 2.0 + nearGoalObstacleCheckMargin;
  const double range = nearGoalObstacleCheckRange;

  if (command.linear.x > 0 && x > 0 && x < range && fabs(y) < halfWidth) {
    command.linear.x = 0;
  }
  if (command.linear.x < 0 && x < 0 && x > -range && fabs(y) < halfWidth) {
    command.linear.x = 0;
  }
  if (command.linear.y > 0 && y > 0 && y < range && fabs(x) < halfLength) {
    command.linear.y = 0;
  }
  if (command.linear.y < 0 && y < 0 && y > -range && fabs(x) < halfLength) {
    command.linear.y = 0;
  }

  if (command.angular.z > 0 && y > 0 && y < range && fabs(x) < halfLength) {
    command.angular.z = 0;
  }
  if (command.angular.z < 0 && y < 0 && y > -range && fabs(x) < halfLength) {
    command.angular.z = 0;
  }
}

void limitNearGoalCommandByCloud(const pcl::PointCloud<pcl::PointXYZI>& cloud,
                                 bool filterHeight,
                                 common_struct::Twist& command)
{
  const int cloudSize = cloud.points.size();
  for (int i = 0; i < cloudSize; ++i) {
    limitNearGoalCommandByPoint(cloud.points[i], filterHeight, command);
    if (command.linear.x == 0 && command.linear.y == 0 && command.angular.z == 0) {
      break;
    }
  }
}

void applyNearGoalObstacleLimits(common_struct::Twist& command)
{
  if (!checkObstacle || nearGoalObstacleCheckRange <= 0) {
    return;
  }

  if (useTerrainAnalysis) {
    limitNearGoalCommandByCloud(*terrainCloudDwz, false, command);
  } else {
    limitNearGoalCommandByCloud(*laserCloudDwz, true, command);
  }
  limitNearGoalCommandByCloud(*boundaryCloud, false, command);
  limitNearGoalCommandByCloud(*addedObstacles, false, command);
}

common_struct::Twist controlNearGoalDirect()
{
  if (!goalReceived || !goalValid) {
    return stopCommand();
  }

  const double dx = goalX - vehicleX;
  const double dy = goalY - vehicleY;
  const double goalXRel = cos(vehicleYaw) * dx + sin(vehicleYaw) * dy;
  const double goalYRel = -sin(vehicleYaw) * dx + cos(vehicleYaw) * dy;
  const double goalDistance = hypot(goalXRel, goalYRel);
  const double yawError = atan2(sin(goalYaw - vehicleYaw), cos(goalYaw - vehicleYaw));
  const double maxPlanarSpeed = maxSpeed * speedRatio;
  const double yawTolerance = nearGoalYawToleranceDeg * PI / 180.0;
  const double minYawRate = nearGoalMinYawRateDeg * PI / 180.0;

  common_struct::Twist command;
  command.linear.x = nearGoalPositionGain * goalXRel;
  command.linear.y = nearGoalPositionGain * goalYRel;
  command.angular.z = nearGoalYawGain * yawError;

  const double planarSpeed = hypot(command.linear.x, command.linear.y);
  if (planarSpeed > maxPlanarSpeed && planarSpeed > 0) {
    const double scale = maxPlanarSpeed / planarSpeed;
    command.linear.x *= scale;
    command.linear.y *= scale;
  }

  if (goalDistance <= nearGoalXYTolerance) {
    command.linear.x = 0;
    command.linear.y = 0;
  } else {
    const double limitedPlanarSpeed = hypot(command.linear.x, command.linear.y);
    if (limitedPlanarSpeed < nearGoalMinSpeed) {
      command.linear.x = nearGoalMinSpeed * goalXRel / goalDistance;
      command.linear.y = nearGoalMinSpeed * goalYRel / goalDistance;
    }
  }
  command.angular.z = applyMinCommand(command.angular.z, yawError, yawTolerance, minYawRate);

  const double maxYawRateRad = maxYawRate * PI / 180.0;
  if (command.angular.z > maxYawRateRad) command.angular.z = maxYawRateRad;
  if (command.angular.z < -maxYawRateRad) command.angular.z = -maxYawRateRad;

  applyNearGoalObstacleLimits(command);

  if ((safetyStop & 1) && command.linear.x > 0) command.linear.x = 0;
  if ((safetyStop & 2) && command.linear.x < 0) command.linear.x = 0;
  if ((safetyStop & 4) && command.angular.z > 0) command.angular.z = 0;
  if ((safetyStop & 8) && command.angular.z < 0) command.angular.z = 0;

  commandedSpeed = command.linear.x;
  commandedSideSpeed = command.linear.y;
  commandedYawRate = command.angular.z;
  return command;
}

common_struct::Twist buildControlCommand(
    double nowSeconds, const common_struct::Path& path)
{
  common_struct::Twist command;
  if (shouldUseNearGoalControl()) {
    command = controlNearGoalDirect();
  } else if (selectedPathValid &&
             nowSeconds - selectedPathTime <= planTimeout) {
    command = controlSelectedPath(path);
  } else {
    command = stopCommand();
  }
  return command;
}

LocalPlannerOutput Step(double nowSeconds)
{
  LocalPlannerOutput output;
  if ((newLaserCloud || newTerrainCloud) && !shouldUseNearGoalControl()) {
    processNewPlanningData(output, nowSeconds, selectedPath);
  }
  output.command = buildControlCommand(nowSeconds, selectedPath);
  return output;
}

};  // struct LocalPlanner::Impl

LocalPlanner::LocalPlanner(const LocalPlannerConfig& config)
    : impl_(new Impl(config)) {}

LocalPlanner::~LocalPlanner() = default;
LocalPlanner::LocalPlanner(LocalPlanner&&) noexcept = default;
LocalPlanner& LocalPlanner::operator=(LocalPlanner&&) noexcept = default;

void LocalPlanner::SetOdometry(double timestamp_seconds,
                               const common_struct::Pose& pose) {
  impl_->SetOdometry(timestamp_seconds, pose);
}

void LocalPlanner::SetRegisteredScan(
    const pcl::PointCloud<pcl::PointXYZI>& registered_scan) {
  impl_->SetRegisteredScan(registered_scan);
}

void LocalPlanner::SetTerrain(
    const pcl::PointCloud<pcl::PointXYZI>& terrain) {
  impl_->SetTerrain(terrain);
}

void LocalPlanner::SetGoal(const common_struct::Pose& goal) {
  impl_->SetGoal(goal);
}

void LocalPlanner::SetGlobalPath(const common_struct::Path& path) {
  impl_->SetGlobalPath(path);
}

void LocalPlanner::SetSpeed(double speed) {
  impl_->SetSpeed(speed);
}

void LocalPlanner::SetBoundary(
    const common_struct::PolygonStamped& boundary) {
  impl_->SetBoundary(boundary);
}

void LocalPlanner::SetAddedObstacles(
    const pcl::PointCloud<pcl::PointXYZI>& obstacles) {
  impl_->SetAddedObstacles(obstacles);
}

void LocalPlanner::SetObstacleChecking(bool enabled) {
  impl_->SetObstacleChecking(enabled);
}

void LocalPlanner::SetSafetyStop(std::int8_t stop_mask) {
  impl_->SetSafetyStop(stop_mask);
}

void LocalPlanner::SetGoalValid(bool valid) {
  impl_->SetGoalValid(valid);
}

LocalPlannerOutput LocalPlanner::Step(double now_seconds) {
  return impl_->Step(now_seconds);
}

double LocalPlanner::control_frequency() const {
  return impl_->controlFrequency;
}

}  // namespace planning
}  // namespace jojo

