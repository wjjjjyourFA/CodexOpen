#include <string>
#include <vector>

#include <ros/ros.h>

#include "codexopen_ros1/yaml_param_loader.h"
#include "fast_calib/fast_calib.h"
#include "fast_calib_ros1/config.h"

namespace fast_calib = jojo::tools::fast_calib;
namespace fast_calib_ros1 = jojo::tools::fast_calib::ros1;

int main(int argc, char** argv) {
  ros::init(argc, argv, "multi_fast_calib");
  ros::NodeHandle private_node("~");
  std::vector<std::string> arguments;
  ros::removeROSArgs(argc, argv, arguments);
  if (arguments.size() < 3) {
    ROS_ERROR_STREAM(
        "Usage: "
        << (arguments.empty() ? "multi_fast_calib_ros1" : arguments[0])
        << " <FastCalib.yaml> <Interface.yaml> [--output DIR]");
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
    if (index + 1 >= arguments.size() || arguments[index] != "--output") {
      ROS_ERROR_STREAM("Usage: " << arguments[0]
                                  << " <FastCalib.yaml> <Interface.yaml> "
                                     "[--output DIR]");
      return 2;
    }
    params.output_path = arguments[index + 1];
    index += 2;
  }
  ROS_INFO_STREAM("FAST-Calib runtime configuration: " << runtime_config);
  ROS_INFO_STREAM("FAST-Calib interface configuration: " << interface_config);
  ROS_INFO_STREAM("FAST-Calib multi-scene input/output directory: "
                  << params.output_path);

  fast_calib::MultiCalibrationResult result;
  if (!fast_calib::RunMultiSceneCalibration(params.output_path, &result,
                                             &error)) {
    ROS_ERROR_STREAM("FAST-Calib multi-scene calibration failed: " << error);
    return 1;
  }
  return 0;
}
