#include <ros/ros.h>

#include "codexopen_ros1/yaml_param_loader.h"
#include "terrain_analysis_ros1/ros1_convert.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "terrain_analysis");
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
  jojo::perception::ros1::TerrainAnalysisRos1Convert adapter(node,
                                                              private_node);
  if (!adapter.Init()) {
    return 1;
  }
  adapter.Run();
  return 0;
}
