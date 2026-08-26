#include <string>
#include <vector>

#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "codexopen_ros1/yaml_param_loader.h"
#include "fast_calib/fast_calib.h"
#include "fast_calib_ros1/bag_loader.h"
#include "fast_calib_ros1/config.h"

namespace fast_calib = jojo::tools::fast_calib;
namespace fast_calib_ros1 = jojo::tools::fast_calib::ros1;

namespace {

template <typename Point>
void PublishCloud(const pcl::PointCloud<Point>& cloud,
                  const std_msgs::Header& header,
                  const ros::Publisher& publisher) {
  sensor_msgs::PointCloud2 message;
  pcl::toROSMsg(cloud, message);
  message.header = header;
  publisher.publish(message);
}

void PrintUsage(const char* executable) {
  ROS_ERROR_STREAM(
      "Usage: " << executable
                << " <FastCalib.yaml> <Interface.yaml> "
                   "[--bag PATH] [--image PATH] [--output DIR] "
                   "[--topic TOPIC]");
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "fast_calib");
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");
  std::vector<std::string> arguments;
  ros::removeROSArgs(argc, argv, arguments);
  if (arguments.size() < 3) {
    PrintUsage(arguments.empty() ? "fast_calib_ros1" : arguments[0].c_str());
    return 2;
  }

  const std::string runtime_config = arguments[1];
  const std::string interface_config = arguments[2];
  if (!codexopen_ros1::LoadYamlParameters(runtime_config, "", private_node) ||
      !codexopen_ros1::LoadYamlParameters(interface_config, "fast_calib",
                                          private_node)) {
    return 1;
  }

  fast_calib::Params params;
  fast_calib_ros1::InterfaceParams interface;
  std::string error;
  if (!fast_calib_ros1::ReadParameters(private_node, &params, &interface,
                                       &error)) {
    ROS_ERROR_STREAM("Cannot read FAST-Calib parameters: " << error);
    return 1;
  }
  for (std::size_t index = 3; index < arguments.size();) {
    if (index + 1 >= arguments.size()) {
      PrintUsage(arguments[0].c_str());
      return 2;
    }
    const std::string& option = arguments[index];
    const std::string& value = arguments[index + 1];
    if (option == "--bag") {
      params.bag_path = value;
    } else if (option == "--image") {
      params.image_path = value;
    } else if (option == "--output") {
      params.output_path = value;
    } else if (option == "--topic") {
      params.lidar_topic = value;
    } else {
      PrintUsage(arguments[0].c_str());
      return 2;
    }
    index += 2;
  }

  ROS_INFO_STREAM("FAST-Calib runtime configuration: " << runtime_config);
  ROS_INFO_STREAM("FAST-Calib interface configuration: " << interface_config);
  ROS_INFO_STREAM("FAST-Calib input bag: " << params.bag_path);
  ROS_INFO_STREAM("FAST-Calib input image: " << params.image_path);
  ROS_INFO_STREAM("FAST-Calib LiDAR topic: " << params.lidar_topic);
  ROS_INFO_STREAM("FAST-Calib output directory: " << params.output_path);

  fast_calib_ros1::InputData input;
  if (!fast_calib_ros1::LoadInputData(params, &input, &error)) {
    ROS_ERROR_STREAM("Cannot load FAST-Calib inputs: " << error);
    return 1;
  }

  fast_calib::SingleCalibrationResult result;
  if (!fast_calib::RunSingleCalibration(params, input.image, input.cloud,
                                        input.lidar_type, &result, &error)) {
    ROS_ERROR_STREAM("FAST-Calib single-scene calibration failed: " << error);
    return 1;
  }
  if (!interface.debug_enabled) {
    ROS_INFO("FAST-Calib finished; debug publishing is disabled.");
    return 0;
  }

  const int cloud_queue = interface.cloud_queue_size;
  const int center_queue = interface.center_queue_size;
  const ros::Publisher qr_publisher =
      node.advertise<sensor_msgs::PointCloud2>(interface.qr_cloud_topic,
                                               cloud_queue);
  const ros::Publisher center_publisher =
      node.advertise<sensor_msgs::PointCloud2>(interface.center_cloud_topic,
                                               center_queue);
  const ros::Publisher filtered_publisher =
      node.advertise<sensor_msgs::PointCloud2>(interface.filtered_cloud_topic,
                                               cloud_queue);
  const ros::Publisher plane_publisher =
      node.advertise<sensor_msgs::PointCloud2>(interface.plane_cloud_topic,
                                               cloud_queue);
  const ros::Publisher aligned_publisher =
      node.advertise<sensor_msgs::PointCloud2>(interface.aligned_cloud_topic,
                                               cloud_queue);
  const ros::Publisher edge_publisher =
      node.advertise<sensor_msgs::PointCloud2>(interface.edge_cloud_topic,
                                               cloud_queue);
  const ros::Publisher center_z0_publisher =
      node.advertise<sensor_msgs::PointCloud2>(interface.center_z0_cloud_topic,
                                               center_queue);
  const ros::Publisher aligned_centers_publisher =
      node.advertise<sensor_msgs::PointCloud2>(
          interface.aligned_lidar_centers_topic, cloud_queue);
  const ros::Publisher colored_publisher =
      node.advertise<sensor_msgs::PointCloud2>(interface.colored_cloud_topic,
                                               cloud_queue);

  ROS_INFO_STREAM("FAST-Calib finished; publishing debug clouds at "
                  << interface.debug_publish_rate_hz << " Hz in frame '"
                  << interface.debug_frame_id << "'.");
  ros::Rate rate(interface.debug_publish_rate_hz);
  while (ros::ok()) {
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = interface.debug_frame_id;
    PublishCloud(*result.camera_centers, header, qr_publisher);
    PublishCloud(*result.lidar_centers, header, center_publisher);
    PublishCloud(*result.lidar_debug.filtered, header, filtered_publisher);
    PublishCloud(*result.lidar_debug.plane, header, plane_publisher);
    PublishCloud(*result.lidar_debug.aligned, header, aligned_publisher);
    PublishCloud(*result.lidar_debug.edge, header, edge_publisher);
    PublishCloud(*result.lidar_debug.centers_z0, header, center_z0_publisher);
    PublishCloud(*result.aligned_lidar_centers, header,
                 aligned_centers_publisher);
    PublishCloud(*result.colored_cloud, header, colored_publisher);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
