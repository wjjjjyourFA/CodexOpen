#include <ros/ros.h>

#include "codexopen_ros1/yaml_param_loader.h"

int RunRobotPlanTerrainAnalysis(ros::NodeHandle& node,
                                ros::NodeHandle& private_node);

int main(int argc, char** argv) {
  ros::init(argc, argv, "terrainAnalysisv2");
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");
  const std::string runtime = argc > 1
      ? argv[1]
      : "./../../../config/TerrainAnalysis/TerrainAnalysis.yaml";
  const std::string interface = argc > 2
      ? argv[2]
      : "./../../../config/TerrainAnalysis/Interface.yaml";
  if (!codexopen_ros1::LoadYamlParameters(runtime, "", private_node) ||
      !codexopen_ros1::LoadYamlParameters(
          interface, "terrain_analysis", private_node)) {
    return 1;
  }
  return RunRobotPlanTerrainAnalysis(node, private_node);
}
