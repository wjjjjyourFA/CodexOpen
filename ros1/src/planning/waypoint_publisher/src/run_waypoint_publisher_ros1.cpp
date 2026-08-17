#include <ros/ros.h>

#include "codexopen_ros1/yaml_param_loader.h"
#include "waypoint_publisher_ros1/ros1_convert.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "waypoint_publisher");
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");

  const std::string runtime = argc > 1
      ? argv[1]
      : "./../../../config/WaypointPublisher/WaypointPublisher.yaml";
  const std::string interface = argc > 2
      ? argv[2]
      : "./../../../config/WaypointPublisher/Interface.yaml";
  const std::string waypoint = argc > 3
      ? argv[3]
      : "./../../../config/WaypointPublisher/data/waypoint.txt";
  const std::string boundary = argc > 4
      ? argv[4]
      : "./../../../config/WaypointPublisher/data/boundary.txt";

  if (!codexopen_ros1::LoadYamlParameters(runtime, "", private_node) ||
      !codexopen_ros1::LoadYamlParameters(
          interface, "waypoint_publisher", private_node)) {
    return 1;
  }
  private_node.setParam("waypoint_file_dir", waypoint);
  private_node.setParam("boundary_file_dir", boundary);
  jojo::planning::ros1::WaypointPublisherRos1Convert adapter(
      node, private_node);
  if (!adapter.Init()) {
    return 1;
  }
  adapter.Run();
  return 0;
}
