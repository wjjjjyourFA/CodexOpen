#include <memory>

#include <pcl/console/print.h>
#include <ros/ros.h>

#include "codexopen_ros1/yaml_param_loader.h"
#include "rog_map/rog_map.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "rm_node");
  ros::NodeHandle private_node("~");
  const std::string runtime = argc > 1
      ? argv[1]
      : "./../../../config/RogMap/RogMap.yaml";
  const std::string interface = argc > 2
      ? argv[2]
      : "./../../../config/RogMap/Interface.yaml";
  if (!codexopen_ros1::LoadYamlParameters(runtime, "", private_node) ||
      !codexopen_ros1::LoadYamlParameters(
          interface, "rog_map_interface", private_node)) {
    return 1;
  }

  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
  auto map = std::make_shared<rog_map::ROGMap>(private_node);
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::Duration(1.0).sleep();
  ros::waitForShutdown();
  return 0;
}
