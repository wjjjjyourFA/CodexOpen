#include <ros/ros.h>

#include "codexopen_ros1/yaml_param_loader.h"
#include "world_planner_ros1/ros1_convert.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "world_planner");
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");
  const std::string runtime = argc > 1
      ? argv[1]
      : "./../../../config/WorldPlanner/WorldPlanner.yaml";
  const std::string interface = argc > 2
      ? argv[2]
      : "./../../../config/WorldPlanner/Interface.yaml";
  const std::string waypoint = argc > 3
      ? argv[3]
      : "./../../../config/WorldPlanner/data/waypoint.txt";
  if (!codexopen_ros1::LoadYamlParameters(runtime, "", private_node) ||
      !codexopen_ros1::LoadYamlParameters(
          interface, "world_planner", private_node)) {
    return 1;
  }
  private_node.setParam("waypoint_file_dir", waypoint);
  jojo::planning::ros1::Ros1Convert adapter(node, private_node);
  if (!adapter.Init()) {
    return 1;
  }
  adapter.Run();
  return 0;
}
