#ifndef ENVIRONMENT_ROS1_H
#define ENVIRONMENT_ROS2_H

#pragma once

#include "modules/common/environment_conf.h"

#if defined(ENABLE_ROS1)
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Pose2D.h>

// sub
#include <nav_msgs/Odometry.h>
// #include "self_state/GlobalPose.h"
// #include "self_state/LocalPose.h"
// #include "self_state/LidarLocalPose.h"
// #include "ars548_msg/DetectionList.h"
// #include "ars548_msg/detections.h"
// #include "sensor/ESR_Radar_Info.h"
// #include "sensor/ESR_Radar_Object.h"

// pub
// #include "world_state/EntityMap.h"
// #include <world_state/PEDESTRIAN_OBJ.h>
// #include <world_state/DYNAMIC_OBJ.h>

typedef geometry_msgs::Pose2D global_pose_msgtype;
typedef geometry_msgs::Pose2D local_pose_msgtype;
typedef geometry_msgs::Pose2D lidar_pose_msgtype;
typedef geometry_msgs::Pose2D lidar_msgtype;
typedef geometry_msgs::Pose2D radar_msgtype;
typedef geometry_msgs::Pose2D cimage_msgtype;
#endif  // ENABLE_ROS1

#endif  // ENVIRONMENT_CONF_H
