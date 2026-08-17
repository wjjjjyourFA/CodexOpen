#include <memory>

#include <ros/ros.h>

#include "codexopen_ros1/yaml_param_loader.h"
#include "robot_dog_fast_lio_ros1/ros1_convert.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_dog_fast_lio");
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");

  const std::string runtime_config = argc > 1
      ? argv[1]
      : "./../../../config/RobotDogFastLio/RobotDogFastLio.yaml";
  const std::string interface_config = argc > 2
      ? argv[2]
      : "./../../../config/RobotDogFastLio/Interface.yaml";
  if (!codexopen_ros1::LoadYamlParameters(
          runtime_config, "", private_node) ||
      !codexopen_ros1::LoadYamlParameters(
          interface_config, "fast_lio", private_node)) {
    return 1;
  }

  auto adapter =
      std::make_shared<jojo::localization::ros1::Ros1Convert>(node, private_node);
  if (!adapter->Init()) {
    ROS_FATAL("robot_dog_fast_lio initialization failed");
    return 1;
  }
  adapter->Run();
  return 0;
}
