#include <memory>

#include <ros/ros.h>

#include "manual_loop_session_exporter_ros1/ros1_convert.h"
#include "codexopen_ros1/yaml_param_loader.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "manual_loop_session_exporter");
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");

  const std::string runtime_config = argc > 1
      ? argv[1]
      : "./../../config/ManualLoopSessionExporter/ManualLoopSessionExporter.yaml";
  const std::string interface_config = argc > 2
      ? argv[2]
      : "./../../config/ManualLoopSessionExporter/Interface.yaml";
  if (!codexopen_ros1::LoadYamlParameters(
          runtime_config, "", private_node) ||
      !codexopen_ros1::LoadYamlParameters(
          interface_config, "manual_loop_session_exporter", private_node)) {
    return 1;
  }
  if (argc > 3) {
    private_node.setParam("io/output_directory", std::string(argv[3]));
  }

  jojo::tools::manual_loop::ros1::Ros1Convert adapter(node, private_node);
  if (!adapter.Init()) {
    return 1;
  }
  ros::spin();
  return 0;
}
