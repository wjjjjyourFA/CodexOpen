#include <ros/ros.h>

#include "codexopen_ros1/yaml_param_loader.h"

int RunRobotPlanLocalPlanner(ros::NodeHandle& node,
                             ros::NodeHandle& private_node);

int main(int argc, char** argv) {
  ros::init(argc, argv, "local_path");
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");
  const std::string runtime = argc > 1
      ? argv[1]
      : "./../../../config/LocalPlanner/LocalPlanner.yaml";
  const std::string interface = argc > 2
      ? argv[2]
      : "./../../../config/LocalPlanner/Interface.yaml";
  const std::string paths = argc > 3
      ? argv[3]
      : "./../../../config/LocalPlanner/paths";
  if (!codexopen_ros1::LoadYamlParameters(runtime, "", private_node) ||
      !codexopen_ros1::LoadYamlParameters(
          interface, "local_planner", private_node)) {
    return 1;
  }
  private_node.setParam("pathFolder", paths);
  return RunRobotPlanLocalPlanner(node, private_node);
}
