#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <thread>

// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>

// msgs
#include "sensor_msgs/PointCloud2.h"
#include <visualization_msgs/Marker.h>

#include <ars430_process/RadarDetection.h>
#include <ars430_process/RadarPacket.h>
#include <ars430_process/SensorStatus.h>

#include "config/config_manager.h"
#include "config/runtime_config.h"

using namespace jojo::drivers;

#define BASE_FRAME "base_link"
#define FOV_ANGLE 75  //Degrees, from centre line outwards, equal on both sides
#define MAX_DIST 16  //metres, arbitrary
std::string radarFrame = "radar_fixed";

pthread_mutex_t mutex;  //Non recursive
tf::Transform fixed;
std::vector<ars430_process::RadarPacket> packets;
ros::Subscriber rawData;
ros::Publisher pcData;
ros::Publisher marker_pub;
visualization_msgs::Marker leftFOVLine, rightFOVline;

uint32_t curTimeStamp = 0;

void packetCloudGenerator(std::vector<ars430_process::RadarPacket> group) {
  static tf::TransformBroadcaster tf_br;

  if (group.size() == 0) {
    printf("Skipping empty group\r\n");
    return;
  }
  sensor_msgs::PointCloud2 pc;
  pc.header          = group[0].header;
  pc.header.frame_id = radarFrame;
  pc.height          = 1;
  pc.width           = 0;

  // Set x field type
  sensor_msgs::PointField pf;
  pf.name     = 'x';
  pf.offset   = 0;
  pf.datatype = sensor_msgs::PointField::FLOAT32;
  pf.count    = 1;
  pc.fields.push_back(pf);

  // Same except name & offset, reuse
  pf.name   = 'y';
  pf.offset = 4;
  pc.fields.push_back(pf);

  pf.name   = 'z';
  pf.offset = 8;
  pc.fields.push_back(pf);

  // Green = small (low rcs), Red = large (high rcs)
  pf.name   = "intensity";
  pf.offset = 12;
  pc.fields.push_back(pf);

  // All computer tested on are little endian
  pc.is_bigendian = false;
  // 4 bytes for x, 4 bytes for y, 4 bytes for z, 4 bytes for intensity
  pc.point_step = 16;
  pc.is_dense   = true;

  uint8_t* tmp;
  for (uint8_t i = 0; i < group.size(); i++) {
    for (uint8_t j = 0; j < group[i].detections.size(); j++) {
      pc.width++;
      // My computer is little endian (lsb @ lowest index)
      tmp = (uint8_t*)&(group[i].detections[j].pos_x);
      for (uint8_t k = 0; k < 4; k++) {
        pc.data.push_back(tmp[k]);
      }

      tmp = (uint8_t*)&(group[i].detections[j].pos_y);
      for (uint8_t k = 0; k < 4; k++) {
        pc.data.push_back(tmp[k]);
      }

      tmp = (uint8_t*)&(group[i].detections[j].pos_z);
      for (uint8_t k = 0; k < 4; k++) {
        pc.data.push_back(tmp[k]);
      }

      // Map from -100/+100 to 0/+100, lower span & more color changes
      float intensity = (group[i].detections[j].rcs / 2.0) + 50.0;
      // Turn float32 into 4 bytes
      tmp = (uint8_t*)&(intensity);
      for (uint8_t k = 0; k < 4; k++) {
        pc.data.push_back(tmp[k]);
      }
    }
  }

  pc.row_step              = pc.point_step * pc.width;
  leftFOVLine.header.stamp = rightFOVline.header.stamp = pc.header.stamp;

  tf_br.sendTransform(
      tf::StampedTransform(fixed, ros::Time::now(), "/base_link", BASE_FRAME));

  pcData.publish(pc);

  marker_pub.publish(leftFOVLine);
  marker_pub.publish(rightFOVline);

  return;
}

void radarCallback(const ars430_process::RadarPacket::ConstPtr& msg) {
  if (curTimeStamp == 0) {
    curTimeStamp = msg->time_stamp;
    packets.push_back(*msg);
  } else if (curTimeStamp != msg->time_stamp) {
    curTimeStamp = msg->time_stamp;
    // Make a local copy
    std::vector<ars430_process::RadarPacket> newGroup = packets;
    packets.clear();
    packets.push_back(*msg);
    // packetCloudGenerator(newGroup);
  } else {  // Time Stamps are the same
    packets.push_back(*msg);
  }
  return;
}

void configureFOVlines(void) {
  leftFOVLine.header.frame_id = rightFOVline.header.frame_id = BASE_FRAME;
  // Configure timestamp at each publish
  leftFOVLine.ns = rightFOVline.ns = "points_and_lines";
  leftFOVLine.action = rightFOVline.action = visualization_msgs::Marker::ADD;
  leftFOVLine.pose.orientation.w = rightFOVline.pose.orientation.w = 1.0;

  leftFOVLine.id  = 1;
  rightFOVline.id = 2;

  leftFOVLine.type = rightFOVline.type = visualization_msgs::Marker::LINE_STRIP;

  leftFOVLine.scale.x = rightFOVline.scale.x = 0.05;
  leftFOVLine.color.b = rightFOVline.color.b = 1.0;
  leftFOVLine.color.r = rightFOVline.color.r = 1.0;
  leftFOVLine.color.g = rightFOVline.color.g = 1.0;
  leftFOVLine.color.a = rightFOVline.color.a = 0.5;

  // Create origin point
  geometry_msgs::Point p;
  p.x = p.y = p.z = 0;
  rightFOVline.points.push_back(p);
  leftFOVLine.points.push_back(p);

  // Create farthest point
  p.x = MAX_DIST;
  p.y = p.x * tan(FOV_ANGLE * M_PI / 180.0);
  rightFOVline.points.push_back(p);
  p.y *= -1;
  leftFOVLine.points.push_back(p);
}

int main(int argc, char** argv) {
  std::string packetTopic = "";
  // Radar Default
  int id = 0, c;

  std::string name   = "";
  auto param_manager = std::make_shared<ConfigManager>();
  auto param_simple  = std::make_shared<RuntimeConfig>();

  // 测试项代码，仅读取第一个雷达
  bool manual_conf = false;
  if (argc <= 1) {
    ROS_INFO("Insufficient args\r\n");

    std::string base_path = ros::package::getPath("ars430_process");
    std::string config_file_path_ = base_path + "/../../../../../install/common/vehicle_sensor_config.yaml";
    ROS_INFO("No command-line args. Using config file: %s",
             config_file_path_.c_str());

    param_manager->LoadConfig(config_file_path_);
    name = param_manager->GetVehicleName() + "_radar_visualizer";
  } else {
    // Get the command line option, if any
    // Pass remainder of arguments to the sniffer
    while ((c = getopt(argc, argv, "+hi:t:")) != -1) {
      switch (c) {
        case 'h':
          printf("\nUsage: %s [-h] [-i ID] [-t Topic Name]\r\n", argv[0]);
          printf("\ti: radar id (int)\r\n");
          printf("\tt: Input radar Topic\r\n");
          exit(0);
          break;
        case 'i':
          id = atoi(optarg);
          printf("ID: %d\r\n", id);
          break;
        case 't':
          packetTopic = std::string(optarg);
          printf("Converting Data From Topic: %s\r\n", packetTopic);
          break;
      }
    }

    manual_conf = true;
  }

  ros::init(argc, argv, name);
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  std::vector<SensorConfig>& radar_configs =
      param_manager->vehicle_model_config.radar_configs;

  if (radar_configs.size() < 1) {
    return false;
  }

  if (!manual_conf) {
    param_simple->vehicle_name = param_manager->GetVehicleName();
    param_simple->LoadConfig(radar_configs.at(0).config_file);
    id = param_simple->id;
  }

  std::string ns    = param_simple->GetVehicleName();
  std::string topic = "/" + ns + param_simple->channel_name;
  packetTopic       = topic + "/filtered_radar_packet_" + std::to_string(id);

  // Non recursive
  mutex = PTHREAD_MUTEX_INITIALIZER;
  // Set transform to be 0 rot, 0 trans
  fixed.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  fixed.setRotation(tf::Quaternion(0, 0, 0, 1));

  pcData = nh.advertise<sensor_msgs::PointCloud2>(
      std::string("radar_pointcloud_") + std::to_string(id), 100);
  rawData = nh.subscribe(packetTopic, 100, radarCallback);
  marker_pub =
      nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  packets.clear();

  // configureFOVlines();

  ros::spin();

  return 0;
}
