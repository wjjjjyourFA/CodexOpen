#include <math.h>
#include <limits>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

const double PI = 3.1415926;

#define PLOTPATHSET 1

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
const int laserCloudStackNum = 1;
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

const int pathNum = 343;
const int groupNum = 7;
float gridVoxelSize = 0.02;
float searchRadius = 0.45;
float gridVoxelOffsetX = 3.2;
float gridVoxelOffsetY = 4.5;
const int gridVoxelNumX = 161;
const int gridVoxelNumY = 451;
const int gridVoxelNum = gridVoxelNumX * gridVoxelNumY;

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudStack[laserCloudStackNum];
pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr boundaryCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr addedObstacles(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZ>::Ptr startPaths[groupNum];
nav_msgs::Path globalReferencePath;
bool hasGlobalReferencePath = false;
#if PLOTPATHSET == 1
pcl::PointCloud<pcl::PointXYZI>::Ptr paths[pathNum];
pcl::PointCloud<pcl::PointXYZI>::Ptr freePaths(new pcl::PointCloud<pcl::PointXYZI>());
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





void odometryHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
  odomTime = odom->header.stamp.toSec();

  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = odom->pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY;
  vehicleY = odom->pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY;
  vehicleZ = odom->pose.pose.position.z;
}

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloud2)
{
  if (!useTerrainAnalysis) {
    laserCloud->clear();
    pcl::fromROSMsg(*laserCloud2, *laserCloud);

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

void terrainCloudHandler(const sensor_msgs::PointCloud2ConstPtr& terrainCloud2)
{
  if (useTerrainAnalysis) {
    terrainCloud->clear();
    pcl::fromROSMsg(*terrainCloud2, *terrainCloud);

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

void goalHandler(const geometry_msgs::PoseStamped::ConstPtr& goal)
{
  goalX = goal->pose.position.x;
  goalY = goal->pose.position.y;
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = goal->pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);
  goalYaw = yaw;
  goalReceived = true;
}

void globalPathHandler(const nav_msgs::Path::ConstPtr& pathIn)
{
  globalReferencePath = *pathIn;
  hasGlobalReferencePath = !globalReferencePath.poses.empty();
}

void speedHandler(const std_msgs::Float32::ConstPtr& speed)
{
  speedRatio = speed->data / maxSpeed;
  if (speedRatio < 0) speedRatio = 0;
  else if (speedRatio > 1.0) speedRatio = 1.0;
}

void boundaryHandler(const geometry_msgs::PolygonStamped::ConstPtr& boundary)
{
  boundaryCloud->clear();
  pcl::PointXYZI point, point1, point2;
  int boundarySize = boundary->polygon.points.size();

  if (boundarySize >= 1) {
    point2.x = boundary->polygon.points[0].x;
    point2.y = boundary->polygon.points[0].y;
    point2.z = boundary->polygon.points[0].z;
  }

  for (int i = 0; i < boundarySize; i++) {
    point1 = point2;

    point2.x = boundary->polygon.points[i].x;
    point2.y = boundary->polygon.points[i].y;
    point2.z = boundary->polygon.points[i].z;

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

void addedObstaclesHandler(const sensor_msgs::PointCloud2ConstPtr& addedObstacles2)
{
  addedObstacles->clear();
  pcl::fromROSMsg(*addedObstacles2, *addedObstacles);

  int addedObstaclesSize = addedObstacles->points.size();
  for (int i = 0; i < addedObstaclesSize; i++) {
    addedObstacles->points[i].intensity = 200.0;
  }
}

void checkObstacleHandler(const std_msgs::Bool::ConstPtr& checkObs)
{
  checkObstacle = checkObs->data;
}

void stopHandler(const std_msgs::Int8::ConstPtr& stop)
{
  safetyStop = stop->data;
}

void goalValidHandler(const std_msgs::Bool::ConstPtr& valid)
{
  goalValid = valid->data;
}

geometry_msgs::Twist stopCommand()
{
  commandedSpeed = 0;
  commandedSideSpeed = 0;
  commandedYawRate = 0;
  return geometry_msgs::Twist();
}

geometry_msgs::Twist controlSelectedPath(const nav_msgs::Path& selectedPath)
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

  geometry_msgs::Twist command;
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
      printf ("\nError reading input files, exit.\n\n");
      exit(1);
    }

    strLast = strCur;
    strCur = string(str);

    if (strCur == "vertex" && strLast == "element") {
      val = fscanf(filePtr, "%d", &pointNum);
      if (val != 1) {
        printf ("\nError reading input files, exit.\n\n");
        exit(1);
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
    printf ("\nCannot read input files, exit.\n\n");
    exit(1);
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
      printf ("\nError reading input files, exit.\n\n");
        exit(1);
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
    printf ("\nCannot read input files, exit.\n\n");
    exit(1);
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
      printf ("\nError reading input files, exit.\n\n");
        exit(1);
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
    printf ("\nCannot read input files, exit.\n\n");
    exit(1);
  }

  if (pathNum != readPlyHeader(filePtr)) {
    printf ("\nIncorrect path number, exit.\n\n");
    exit(1);
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
      printf ("\nError reading input files, exit.\n\n");
        exit(1);
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
    printf ("\nCannot read input files, exit.\n\n");
    exit(1);
  }

  int val1, gridVoxelID, pathID;
  for (int i = 0; i < gridVoxelNum; i++) {
    val1 = fscanf(filePtr, "%d", &gridVoxelID);
    if (val1 != 1) {
      printf ("\nError reading input files, exit.\n\n");
        exit(1);
    }

    while (1) {
      val1 = fscanf(filePtr, "%d", &pathID);
      if (val1 != 1) {
        printf ("\nError reading input files, exit.\n\n");
          exit(1);
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

struct RosInterfaces
{
  ros::Subscriber subOdometry;
  ros::Subscriber subLaserCloud;
  ros::Subscriber subTerrainCloud;
  ros::Subscriber subGoal;
  ros::Subscriber subGlobalPath;
  ros::Subscriber subSpeed;
  ros::Subscriber subBoundary;
  ros::Subscriber subAddedObstacles;
  ros::Subscriber subStop;
  ros::Subscriber subGoalValid;
  ros::Publisher pubPath;
  ros::Publisher pubSpeed;
#if PLOTPATHSET == 1
  ros::Publisher pubFreePaths;
#endif
};

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

void loadParameters(ros::NodeHandle& nhPrivate)
{
  nhPrivate.getParam("pathFolder", pathFolder);
  nhPrivate.getParam("vehicleLength", vehicleLength);
  nhPrivate.getParam("vehicleWidth", vehicleWidth);
  nhPrivate.getParam("sensorOffsetX", sensorOffsetX);
  nhPrivate.getParam("sensorOffsetY", sensorOffsetY);
  nhPrivate.getParam("twoWayDrive", twoWayDrive);
  nhPrivate.getParam("laserVoxelSize", laserVoxelSize);
  nhPrivate.getParam("terrainVoxelSize", terrainVoxelSize);
  nhPrivate.getParam("useTerrainAnalysis", useTerrainAnalysis);
  nhPrivate.getParam("checkObstacle", checkObstacle);
  nhPrivate.getParam("checkRotObstacle", checkRotObstacle);
  nhPrivate.getParam("adjacentRange", adjacentRange);
  nhPrivate.getParam("obstacleHeightThre", obstacleHeightThre);
  nhPrivate.getParam("groundHeightThre", groundHeightThre);
  nhPrivate.getParam("costHeightThre", costHeightThre);
  nhPrivate.getParam("costScore", costScore);
  nhPrivate.getParam("obstacleInflationRadius", obstacleInflationRadius);
  nhPrivate.getParam("inflatedObstaclePenalty", inflatedObstaclePenalty);
  nhPrivate.getParam("centerPathBias", centerPathBias);
  nhPrivate.getParam("pathContinuityWeight", pathContinuityWeight);
  nhPrivate.getParam("groupContinuityWeight", groupContinuityWeight);
  nhPrivate.getParam("sideSwitchPenalty", sideSwitchPenalty);
  nhPrivate.getParam("largeSwitchAngleDeg", largeSwitchAngleDeg);
  nhPrivate.getParam("holdPathScoreRatio", holdPathScoreRatio);
  nhPrivate.getParam("useCost", useCost);
  nhPrivate.getParam("pointPerPathThre", pointPerPathThre);
  nhPrivate.getParam("minRelZ", minRelZ);
  nhPrivate.getParam("maxRelZ", maxRelZ);
  nhPrivate.getParam("maxSpeed", maxSpeed);
  nhPrivate.getParam("dirWeight", dirWeight);
  nhPrivate.getParam("dirThre", dirThre);
  nhPrivate.getParam("dirToVehicle", dirToVehicle);
  nhPrivate.getParam("pathScale", pathScale);
  nhPrivate.getParam("minPathScale", minPathScale);
  nhPrivate.getParam("pathScaleStep", pathScaleStep);
  nhPrivate.getParam("pathScaleBySpeed", pathScaleBySpeed);
  nhPrivate.getParam("minPathRange", minPathRange);
  nhPrivate.getParam("pathRangeStep", pathRangeStep);
  nhPrivate.getParam("pathRangeBySpeed", pathRangeBySpeed);
  nhPrivate.getParam("pathCropByGoal", pathCropByGoal);
  nhPrivate.getParam("autonomyMode", autonomyMode);
  nhPrivate.getParam("autonomySpeed", autonomySpeed);
  nhPrivate.getParam("goalClearRange", goalClearRange);
  nhPrivate.getParam("goalX", goalX);
  nhPrivate.getParam("goalY", goalY);
  nhPrivate.getParam("globalPathLookAhead", globalPathLookAhead);
  nhPrivate.getParam("globalPathGoalSwitchDis", globalPathGoalSwitchDis);
  nhPrivate.getParam("controlLookAheadDis", controlLookAheadDis);
  nhPrivate.getParam("forwardAlignAngle", forwardAlignAngle);
  nhPrivate.getParam("yawRateGain", yawRateGain);
  nhPrivate.getParam("maxYawRate", maxYawRate);
  nhPrivate.getParam("maxAccel", maxAccel);
  nhPrivate.getParam("maxYawAccel", maxYawAccel);
  nhPrivate.getParam("goalStopDistance", goalStopDistance);
  nhPrivate.getParam("goalSlowDistance", goalSlowDistance);
  nhPrivate.getParam("planTimeout", planTimeout);
  nhPrivate.getParam("controlFrequency", controlFrequency);
  nhPrivate.getParam("nearGoalEnterDistance", nearGoalEnterDistance);
  nhPrivate.getParam("nearGoalExitDistance", nearGoalExitDistance);
  nhPrivate.getParam("nearGoalXYTolerance", nearGoalXYTolerance);
  nhPrivate.getParam("nearGoalYawToleranceDeg", nearGoalYawToleranceDeg);
  nhPrivate.getParam("nearGoalMinSpeed", nearGoalMinSpeed);
  nhPrivate.getParam("nearGoalMinYawRateDeg", nearGoalMinYawRateDeg);
  nhPrivate.getParam("nearGoalPositionGain", nearGoalPositionGain);
  nhPrivate.getParam("nearGoalYawGain", nearGoalYawGain);
  nhPrivate.getParam("nearGoalObstacleCheckRange", nearGoalObstacleCheckRange);
  nhPrivate.getParam("nearGoalObstacleCheckMargin", nearGoalObstacleCheckMargin);
  goalStopDistance = std::max(0.2, goalStopDistance);
  goalSlowDistance = std::max(goalStopDistance + 0.05, goalSlowDistance);
  controlFrequency = std::max(10.0, controlFrequency);
  nearGoalExitDistance = std::max(nearGoalEnterDistance, nearGoalExitDistance);
  nearGoalYawToleranceDeg = std::max(0.1, nearGoalYawToleranceDeg);
  nearGoalMinSpeed = std::max(0.0, nearGoalMinSpeed);
  nearGoalMinYawRateDeg = std::max(0.0, nearGoalMinYawRateDeg);
  nearGoalObstacleCheckRange = std::max(0.0, nearGoalObstacleCheckRange);
  nearGoalObstacleCheckMargin = std::max(0.0, nearGoalObstacleCheckMargin);
}

RosInterfaces setupRosInterfaces(ros::NodeHandle& nh,
                                 ros::NodeHandle& nhPrivate)
{
  RosInterfaces io;
  std::string odometryTopic = "/state_estimation";
  std::string registeredScanTopic = "/registered_scan";
  std::string terrainTopic = "/terrain_map";
  std::string goalTopic = "/way_point";
  std::string globalPathTopic = "/global_reference_path";
  std::string speedTopic = "/speed";
  std::string boundaryTopic = "/navigation_boundary";
  std::string addedObstaclesTopic = "/added_obstacles";
  std::string stopTopic = "/stop";
  std::string goalValidTopic = "/isgoal_vaild";
  std::string pathTopic = "/path";
  std::string commandTopic = "/cmd_vel_corrected";
  std::string freePathsTopic = "/free_paths";
  nhPrivate.getParam("odometry_topic", odometryTopic);
  nhPrivate.getParam("registered_scan_topic", registeredScanTopic);
  nhPrivate.getParam("terrain_topic", terrainTopic);
  nhPrivate.getParam("goal_topic", goalTopic);
  nhPrivate.getParam("global_path_topic", globalPathTopic);
  nhPrivate.getParam("speed_topic", speedTopic);
  nhPrivate.getParam("boundary_topic", boundaryTopic);
  nhPrivate.getParam("added_obstacles_topic", addedObstaclesTopic);
  nhPrivate.getParam("stop_topic", stopTopic);
  nhPrivate.getParam("goal_valid_topic", goalValidTopic);
  nhPrivate.getParam("path_topic", pathTopic);
  nhPrivate.getParam("command_topic", commandTopic);
  nhPrivate.getParam("free_paths_topic", freePathsTopic);
  io.subOdometry = nh.subscribe<nav_msgs::Odometry>(odometryTopic, 5, odometryHandler);
  io.subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(registeredScanTopic, 5, laserCloudHandler);
  io.subTerrainCloud = nh.subscribe<sensor_msgs::PointCloud2>(terrainTopic, 5, terrainCloudHandler);
  io.subGoal = nh.subscribe<geometry_msgs::PoseStamped>(goalTopic, 5, goalHandler);
  io.subGlobalPath = nh.subscribe<nav_msgs::Path>(globalPathTopic, 5, globalPathHandler);
  io.subSpeed = nh.subscribe<std_msgs::Float32>(speedTopic, 5, speedHandler);
  io.subBoundary = nh.subscribe<geometry_msgs::PolygonStamped>(boundaryTopic, 5, boundaryHandler);
  io.subAddedObstacles = nh.subscribe<sensor_msgs::PointCloud2>(addedObstaclesTopic, 5, addedObstaclesHandler);
  io.subStop = nh.subscribe<std_msgs::Int8>(stopTopic, 5, stopHandler);
  io.subGoalValid = nh.subscribe<std_msgs::Bool>(goalValidTopic, 5, goalValidHandler);
  io.pubPath = nh.advertise<nav_msgs::Path>(pathTopic, 5);
  io.pubSpeed = nh.advertise<geometry_msgs::Twist>(commandTopic, 5);
#if PLOTPATHSET == 1
  io.pubFreePaths = nh.advertise<sensor_msgs::PointCloud2>(freePathsTopic, 2);
#endif
  return io;
}

void initializePathData()
{
  printf ("\nReading path files.\n");

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

  printf ("\nInitialization complete.\n\n");
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

void fillSelectedPath(int rotDir, int selectedGroupID, float pathRange, float relativeGoalDis, nav_msgs::Path& path)
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
      std::cout<<" i, dis : " <<i << " "<< dis <<std::endl;
      break;
    }
  }

  path.header.stamp = ros::Time().fromSec(odomTime);
  path.header.frame_id = "vehicle";
  selectedPathValid = path.poses.size() > 1;
  selectedPathReverseMode = isReverseRotation(rotDir);
  selectedPathTime = ros::Time::now().toSec();
  selectedPathVehicleX = vehicleX;
  selectedPathVehicleY = vehicleY;
  selectedPathVehicleYaw = vehicleYaw;
}

#if PLOTPATHSET == 1
void publishFreePaths(const ros::Publisher& pubFreePaths,
                      const RotationObstacleLimits& limits,
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

  sensor_msgs::PointCloud2 freePaths2;
  pcl::toROSMsg(*freePaths, freePaths2);
  freePaths2.header.stamp = ros::Time().fromSec(odomTime);
  freePaths2.header.frame_id = "vehicle";
  pubFreePaths.publish(freePaths2);
}

void publishEmptyFreePaths(const ros::Publisher& pubFreePaths)
{
  freePaths->clear();
  sensor_msgs::PointCloud2 freePaths2;
  pcl::toROSMsg(*freePaths, freePaths2);
  freePaths2.header.stamp = ros::Time().fromSec(odomTime);
  freePaths2.header.frame_id = "vehicle";
  pubFreePaths.publish(freePaths2);
}
#endif

void publishNoPath(const RosInterfaces& io, nav_msgs::Path& path)
{
  hasLastSelection = false;
  lastSelectedRotDir = -1;
  lastSelectedPathGroup = -1;
  path.poses.resize(1);
  path.poses[0].pose.position.x = 0;
  path.poses[0].pose.position.y = 0;
  path.poses[0].pose.position.z = 0;
  path.header.stamp = ros::Time().fromSec(odomTime);
  path.header.frame_id = "vehicle";
  selectedPathValid = false;
  selectedPathReverseMode = false;
  selectedPathTime = ros::Time::now().toSec();
  io.pubPath.publish(path);
#if PLOTPATHSET == 1
  publishEmptyFreePaths(io.pubFreePaths);
#endif
}

bool evaluateAndPublishPath(const RosInterfaces& io, nav_msgs::Path& path, float pathRange, float relativeGoalDis)
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

      fillSelectedPath(rotDir, selectedGroupID, pathRange, relativeGoalDis, path);
      selectedPathReverseMode = selection.reverseMode;
      io.pubPath.publish(path);
#if PLOTPATHSET == 1
      publishFreePaths(io.pubFreePaths, limits, pathRange, relativeGoalDis);
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

void processNewPlanningData(const RosInterfaces& io, nav_msgs::Path& path)
{
  updatePlannerCloud();

  const VehicleTrig trig = makeVehicleTrig();
  buildPlannerCloudCrop(trig);

  float pathRange = initialPathRange();
  float relativeGoalDis = adjacentRange;
  updateAutonomyTarget(trig, relativeGoalDis);

  if (!evaluateAndPublishPath(io, path, pathRange, relativeGoalDis)) {
    publishNoPath(io, path);
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

void limitNearGoalCommandByPoint(const pcl::PointXYZI& point, bool filterHeight, geometry_msgs::Twist& command)
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
                                 geometry_msgs::Twist& command)
{
  const int cloudSize = cloud.points.size();
  for (int i = 0; i < cloudSize; ++i) {
    limitNearGoalCommandByPoint(cloud.points[i], filterHeight, command);
    if (command.linear.x == 0 && command.linear.y == 0 && command.angular.z == 0) {
      break;
    }
  }
}

void applyNearGoalObstacleLimits(geometry_msgs::Twist& command)
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

geometry_msgs::Twist controlNearGoalDirect()
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

  geometry_msgs::Twist command;
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

void publishControlCommand(const ros::Publisher& pubSpeed, const nav_msgs::Path& path)
{
  geometry_msgs::Twist command;
  const double now = ros::Time::now().toSec();
  if (shouldUseNearGoalControl()) {
    command = controlNearGoalDirect();
  } else if (selectedPathValid && now - selectedPathTime <= planTimeout) {
    command = controlSelectedPath(path);
  } else {
    command = stopCommand();
  }
  pubSpeed.publish(command);
}

int RunRobotPlanLocalPlanner(ros::NodeHandle& nh,
                             ros::NodeHandle& nhPrivate)
{
  loadParameters(nhPrivate);
  RosInterfaces io = setupRosInterfaces(nh, nhPrivate);
  nav_msgs::Path path;
  initializePathData();

  ros::Rate rate(controlFrequency);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();

    if ((newLaserCloud || newTerrainCloud) && !shouldUseNearGoalControl()) {
      processNewPlanningData(io, path);
    }
    publishControlCommand(io.pubSpeed, path);

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}

#ifndef ROBOT_PLAN_EXPV2_NO_MAIN
int main(int argc, char** argv)
{
  ros::init(argc, argv, "local_path");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate("~");
  return RunRobotPlanLocalPlanner(nh, nhPrivate);
}
#endif

