#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>   
#include <geometry_msgs/PointStamped.h>

#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <mutex>

using namespace std;

const double PI = 3.1415926;

string waypoint_file_dir;
string boundary_file_dir;
double waypointZBound = 5.0;
double waitTime = 0;
double waitTimeStart = 0;
bool isWaiting = false;
double frameRate = 5.0;
double speed = 1.0;
bool sendSpeed = true;
bool sendBoundary = true;

double waypointXYRadius = 0.5;
double waypointYawThreshold = 0.1745;   // 10 degrees
std::mutex waypoint_mutex;

std::vector<geometry_msgs::Pose> waypointPoses;
pcl::PointCloud<pcl::PointXYZ>::Ptr boundary(new pcl::PointCloud<pcl::PointXYZ>());

float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
float vehicleYaw = 0;
double curTime = 0, waypointTime = 0;

// reading waypoints from file function

void readWaypointFile()
{
  FILE* waypoint_file = fopen(waypoint_file_dir.c_str(), "r");
  if (waypoint_file == NULL) {
    printf("\nCannot read waypoint file, exit.\n\n");
    exit(1);
  }

  waypointPoses.clear();
  geometry_msgs::Pose pose;
  float tx, ty, tz, roll, pitch, yaw;

  while (fscanf(waypoint_file, "%f %f %f %f %f %f", 
                &tx, &ty, &tz, &roll, &pitch, &yaw) == 6) {
    pose.position.x = tx;
    pose.position.y = ty;
    pose.position.z = tz;

    tf2::Quaternion q;
    q.setRPY(roll*M_PI/180.0, pitch*M_PI/180.0, yaw*M_PI/180.0);  // 转换为四元数
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();


    waypointPoses.push_back(pose);
  }

  fclose(waypoint_file);
}

void navGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(waypoint_mutex);

  geometry_msgs::Pose pose = msg->pose;
  waypointPoses.push_back(pose);

  // 如果是第一个 waypoint，重置状态
  if (waypointPoses.size() == 1) {
    ROS_INFO("Received first RViz goal, start navigation.");
  } else {
    ROS_INFO("Received new RViz goal, total waypoints: %lu",
             waypointPoses.size());
  }
}

void readBoundaryFile()
{
  FILE* boundary_file = fopen(boundary_file_dir.c_str(), "r");
  if (boundary_file == NULL) {
    printf ("\nCannot read boundary file, exit.\n\n");
    exit(1);
  }

  boundary->clear();
  pcl::PointXYZ point;
  while (fscanf(boundary_file, "%f %f %f", &point.x, &point.y, &point.z) == 3) {
    boundary->push_back(point);
  }

  fclose(boundary_file);
}



double GetYaw(const geometry_msgs::Pose& pose)
{
  tf::Quaternion q(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
  );
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

// vehicle pose callback function
void poseHandler(const nav_msgs::Odometry::ConstPtr& pose)
{
  curTime = pose->header.stamp.toSec();


  vehicleX = pose->pose.pose.position.x;
  vehicleY = pose->pose.pose.position.y;
  vehicleZ = pose->pose.pose.position.z;

  vehicleYaw = GetYaw(pose->pose.pose);
}

int RunWaypointPublisher(ros::NodeHandle& nh,
                         ros::NodeHandle& nhPrivate)
{
  nhPrivate.getParam("waypoint_file_dir", waypoint_file_dir);
  nhPrivate.getParam("boundary_file_dir", boundary_file_dir);
  nhPrivate.getParam("waypointZBound", waypointZBound);
  nhPrivate.getParam("waitTime", waitTime);
  nhPrivate.getParam("frameRate", frameRate);
  nhPrivate.getParam("speed", speed);
  nhPrivate.getParam("sendSpeed", sendSpeed);
  nhPrivate.getParam("sendBoundary", sendBoundary);

  nhPrivate.getParam("waypointXYRadius", waypointXYRadius);
  nhPrivate.getParam("waypointYawThreshold", waypointYawThreshold);

  string odometryTopic = "/state_estimation";
  string waypointTopic = "/way_point";
  string waypointShowTopic = "/way_point_show";
  string navigationGoalTopic = "/move_base_simple/goal";
  string speedTopic = "/speed";
  string boundaryTopic = "/navigation_boundary";
  string goalValidTopic = "/isgoal_vaild";
  string waypointFrame = "map";
  string boundaryFrame = "vehicle";
  nhPrivate.getParam("odometry_topic", odometryTopic);
  nhPrivate.getParam("waypoint_topic", waypointTopic);
  nhPrivate.getParam("waypoint_show_topic", waypointShowTopic);
  nhPrivate.getParam("navigation_goal_topic", navigationGoalTopic);
  nhPrivate.getParam("speed_topic", speedTopic);
  nhPrivate.getParam("boundary_topic", boundaryTopic);
  nhPrivate.getParam("goal_valid_topic", goalValidTopic);
  nhPrivate.getParam("waypoint_frame", waypointFrame);
  nhPrivate.getParam("boundary_frame", boundaryFrame);

  ros::Subscriber subPose = nh.subscribe<nav_msgs::Odometry> (odometryTopic, 5, poseHandler);

  ros::Publisher pubWaypoint = nh.advertise<geometry_msgs::PoseStamped> (waypointTopic, 5);
  geometry_msgs::PoseStamped waypointMsgs;
  waypointMsgs.header.frame_id = waypointFrame;

  ros::Publisher pubWaypoint_show = nh.advertise<geometry_msgs::PointStamped> (waypointShowTopic, 5);
  geometry_msgs::PointStamped waypointMsgs_show;
  waypointMsgs_show.header.frame_id = waypointFrame;

  ros::Subscriber subNavGoal =
    nh.subscribe<geometry_msgs::PoseStamped>(
        navigationGoalTopic, 5, navGoalCallback);


  ros::Publisher pubSpeed = nh.advertise<std_msgs::Float32> (speedTopic, 5);
  std_msgs::Float32 speedMsgs;

  ros::Publisher pubBoundary = nh.advertise<geometry_msgs::PolygonStamped> (boundaryTopic, 5);
  geometry_msgs::PolygonStamped boundaryMsgs;
  boundaryMsgs.header.frame_id = boundaryFrame;

  //////////////////////

  ros::Publisher pubIsgoalvaild = nh.advertise<std_msgs::Bool> (goalValidTopic, 5);
  std_msgs::Bool nav_done;
  nav_done.data = false;

  /////////////////////

  // read waypoints from file
  readWaypointFile();

  // read boundary from file
  if (sendBoundary) {
    readBoundaryFile();

    int boundarySize = boundary->points.size();
    boundaryMsgs.polygon.points.resize(boundarySize);
    for (int i = 0; i < boundarySize; i++) {
      boundaryMsgs.polygon.points[i].x = boundary->points[i].x;
      boundaryMsgs.polygon.points[i].y = boundary->points[i].y;
      boundaryMsgs.polygon.points[i].z = boundary->points[i].z;
    }
  }

  int wayPointID = 0;
  int waypointSize;

  {
    std::lock_guard<std::mutex> lock(waypoint_mutex);

    waypointSize = waypointPoses.size();
  }

  if (waypointSize == 0)
  {
    ROS_WARN("No waypoint available, waiting for RViz 2D Nav Goal...");
  }

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();

    {
      std::lock_guard<std::mutex> lock(waypoint_mutex);

      waypointSize = waypointPoses.size();
    }

    if (waypointSize == 0) {
      pubIsgoalvaild.publish(nav_done);
      continue;
    }

    // if (wayPointID >= waypointSize){
    //   continue;
    //   // wayPointID = waypointSize - 1;
    // }

    float disX = vehicleX - waypointPoses[wayPointID].position.x;
    float disY = vehicleY - waypointPoses[wayPointID].position.y;
    float disZ = vehicleZ - waypointPoses[wayPointID].position.z;


    double goalYaw = GetYaw(waypointPoses[wayPointID]);

    double yawDiff = goalYaw - vehicleYaw;
    yawDiff = atan2(sin(yawDiff), cos(yawDiff));


    // start waiting if the current waypoint is reached
    if (sqrt(disX * disX + disY * disY) < waypointXYRadius && 
        fabs(disZ) < waypointZBound && 
        fabs(yawDiff) < waypointYawThreshold && 
        !isWaiting) {
      waitTimeStart = curTime;
      isWaiting = true;
    }

    // move to the next waypoint after waiting is over
    if (isWaiting && waitTimeStart + waitTime < curTime && wayPointID < waypointSize - 1) {
      wayPointID++;
      isWaiting = false;
    }

    // publish waypoint, speed, and boundary messages at certain frame rate
    if (curTime - waypointTime > 1.0 / frameRate) {
      if (!isWaiting) {
        waypointMsgs.header.stamp = ros::Time().fromSec(curTime);
        waypointMsgs.pose = waypointPoses[wayPointID];
        pubWaypoint.publish(waypointMsgs);

        waypointMsgs_show.header.stamp = ros::Time().fromSec(curTime);
        waypointMsgs_show.point.x = waypointPoses[wayPointID].position.x;
        waypointMsgs_show.point.y = waypointPoses[wayPointID].position.y;
        waypointMsgs_show.point.z = waypointPoses[wayPointID].position.z;
        pubWaypoint_show.publish(waypointMsgs_show);

        
      }

      if (sendSpeed) {
        speedMsgs.data = speed;
        pubSpeed.publish(speedMsgs);
      }

      if (sendBoundary) {
        boundaryMsgs.header.stamp = ros::Time().fromSec(curTime);
        pubBoundary.publish(boundaryMsgs);
      }

      waypointTime = curTime;
    }

    if(isWaiting) {
      nav_done.data = false;
    }else{
      nav_done.data = true;
    }

    pubIsgoalvaild.publish(nav_done);

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}

#ifndef WAYPOINT_PUBLISHER_NO_MAIN
int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypointExample");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate("~");
  return RunWaypointPublisher(nh, nhPrivate);
}
#endif
