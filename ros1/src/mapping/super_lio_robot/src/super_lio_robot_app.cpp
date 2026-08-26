#include <csignal>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include <ros/ros.h>

#include "basic/logs.h"
#include "codexopen_ros1/yaml_param_loader.h"
#include "lio/params.h"
#include "lio/super_lio.h"
#include "lio/super_lio_reloc.h"
#include "super_lio_robot_ros1/ros1_convert.h"

namespace {

void SignalHandler(int) {
  LI2Sup::g_flag_run = false;
}

void PrintUsage(const char* program) {
  std::cerr << "Usage: " << program
            << " <runtime.yaml> <interface.yaml>"
               " <mapping|relocation> <map-root>\n";
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 5) {
    PrintUsage(argv[0]);
    return 2;
  }
  const std::string runtime_config = argv[1];
  const std::string interface_config = argv[2];
  const std::string operation = argv[3];
  const std::string map_root = argv[4];
  if (operation != "mapping" && operation != "relocation") {
    PrintUsage(argv[0]);
    return 2;
  }

  ::ros::init(argc, argv, "super_lio_robot",
              ::ros::init_options::NoSigintHandler);
  std::signal(SIGINT, SignalHandler);
  std::signal(SIGTERM, SignalHandler);
  ::ros::NodeHandle node;
  ::ros::NodeHandle private_node("~");

  if (!codexopen_ros1::LoadYamlParameters(
          runtime_config, "lio", private_node) ||
      !codexopen_ros1::LoadYamlParameters(
          interface_config, "super_lio_robot", private_node)) {
    return 1;
  }

  auto adapter = std::make_shared<LI2Sup::ros1::Ros1Convert>(
      node, private_node);
  if (!adapter->Init(map_root)) {
    ROS_FATAL("super_lio_robot initialization failed");
    return 1;
  }

  std::shared_ptr<LI2Sup::SuperLIO> lio;
  std::shared_ptr<LI2Sup::SuperLIOReLoc> relocation;
  if (operation == "relocation") {
    const std::filesystem::path map_path =
        std::filesystem::path(LI2Sup::g_save_map_dir) / LI2Sup::g_map_name;
    if (!std::filesystem::is_regular_file(map_path)) {
      ROS_FATAL_STREAM("Relocation map not found: " << map_path.string());
      return 1;
    }
    relocation = std::make_shared<LI2Sup::SuperLIOReLoc>();
    lio = relocation;
  } else {
    lio = std::make_shared<LI2Sup::SuperLIO>();
  }
  lio->setDataInterface(adapter);
  lio->init();

  if (relocation) {
    LOG(INFO) << YELLOW
              << " ---> Relocation waits for RViz /initialpose by default. "
                 "Press Enter in this terminal to use relocation/init_pose once."
              << RESET;
    std::thread([relocation]() {
      std::string line;
      while (LI2Sup::g_flag_run && std::getline(std::cin, line)) {
        if (line.empty()) relocation->requestParamFileInit();
      }
    }).detach();
  }

  ::ros::Rate rate(adapter->processing_rate());
  while (::ros::ok() && LI2Sup::g_flag_run) {
    adapter->SpinOnce();
    lio->process();
    rate.sleep();
  }
  lio->saveMap();
  lio->printTimeRecord();
  ::ros::shutdown();
  return 0;
}
