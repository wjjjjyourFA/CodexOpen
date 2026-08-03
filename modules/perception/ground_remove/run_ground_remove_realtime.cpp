#include "modules/common/environment_conf.h"
#include "modules/perception/ground_remove/ros1_convert.h"

using namespace jojo::perception;
namespace cfg = jojo::perception::config;

int main(int argc, char** argv) {
  // clang-format off
  std::string name = "GroundRemoveRealTime";
  std::string config_path = "./../../../config/GroundRemove/GroundRemove.ini";
  // clang-format on

  auto runtime_config = std::make_shared<jojo::perception::RuntimeConfig>();
  runtime_config->set_name(name);
  runtime_config->LoadConfig(config_path);

  std::string if_config_path = "./../../../config/GroundRemove/Interface.ini";

  auto if_runtime_config = std::make_shared<jojo::perception::Interface>();
  if_runtime_config->set_name(name);
  if_runtime_config->LoadConfig(if_config_path);

  // 初始化 ROS 节点
  ros::init(argc, argv, "ground_remove_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // GroundRemoveNode
  auto _pRos1Convert = std::make_shared<Ros1Convert>();
  _pRos1Convert->Init(nh, pnh, runtime_config, if_runtime_config);

  _pRos1Convert->Run();

  // ros::spin();

  return 0;
}
