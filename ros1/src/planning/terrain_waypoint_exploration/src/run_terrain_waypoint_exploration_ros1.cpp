#include <ros/ros.h>

#include "codexopen_ros1/yaml_param_loader.h"
#include "terrain_waypoint_exploration/terrain_waypoint_explorer.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "terrain_waypoint_explorer");
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");
  const std::string runtime = argc > 1
      ? argv[1]
      : "./../../../config/TerrainWaypointExploration/TerrainWaypointExplorer.yaml";
  const std::string interface = argc > 2
      ? argv[2]
      : "./../../../config/TerrainWaypointExploration/Interface.yaml";
  if (!codexopen_ros1::LoadYamlParameters(runtime, "", private_node) ||
      !codexopen_ros1::LoadYamlParameters(
          interface, "terrain_waypoint_explorer", private_node)) {
    return 1;
  }
  terrain_waypoint_exploration::TerrainWaypointExplorer explorer(
      node, private_node);
  ros::spin();
  return 0;
}
