#include <filesystem>
#include <iostream>
#include <string>

#include <ros/ros.h>

#include "codexopen_ros1/yaml_param_loader.h"
#include "prior_map_localization_ros1/ros1_convert.h"

namespace {

std::filesystem::path AbsolutePath(const char* path) {
  return std::filesystem::absolute(std::filesystem::path(path));
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 5) {
    std::cerr << "Usage: " << argv[0]
              << " <runtime.yaml> <interface.yaml> <map.pcd> <log_dir>\n";
    return 2;
  }

  const std::filesystem::path runtime_config = AbsolutePath(argv[1]);
  const std::filesystem::path interface_config = AbsolutePath(argv[2]);
  const std::filesystem::path map_path = AbsolutePath(argv[3]);
  const std::filesystem::path log_dir = AbsolutePath(argv[4]);

  if (!std::filesystem::is_regular_file(runtime_config) ||
      !std::filesystem::is_regular_file(interface_config) ||
      !std::filesystem::is_regular_file(map_path)) {
    std::cerr << "Prior-map localization input file is missing\n";
    return 2;
  }

  std::filesystem::create_directories(log_dir / "Log");
  std::filesystem::create_directories(log_dir / "log");
  std::filesystem::current_path(log_dir);

  ros::init(argc, argv, "prior_map_localization");
  ros::NodeHandle private_node("~");
  if (!codexopen_ros1::LoadYamlParameters(
          interface_config.string(), "prior_map_localization", private_node)) {
    return 2;
  }

  ros::NodeHandle node;
  jojo::localization::ros1::PriorMapLocalizationRos1Convert adapter(
      node, private_node, runtime_config.string(), map_path.string(),
      (log_dir / "Log").string());
  if (!adapter.Init()) {
    return 2;
  }
  adapter.Run();
  return 0;
}
