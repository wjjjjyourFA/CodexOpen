#include "runtime_config.h"

namespace jojo {
namespace drivers {

void RuntimeConfig::LoadConfig(const std::string& config_path) {
  try {
    // 创建一个 property_tree 对象
    boost::property_tree::ptree pt;

    // 读取 ini 文件
    boost::property_tree::ini_parser::read_ini(config_path, pt);
    // clang-format off
    // 从文件中读取配置项
    type = pt.get<std::string>("general.type", "udp");
    group_ip = pt.get<std::string>("general.group_ip", "224.0.2.2");
    group_port = pt.get<int>("general.group_port", 42102);
    radar_ip = pt.get<std::string>("general.radar_ip", "10.13.1.113");
    radar_port = pt.get<int>("general.radar_port", 42101);
    local_ip = pt.get<std::string>("general.local_ip", "10.13.1.166");
    local_port = pt.get<int>("general.local_port", 42401);

    frame_id = pt.get<std::string>("topics.frame_id", "");
    channel_name = pt.get<std::string>("topics.channel_name", "");
    // 打印其他参数...
    // clang-format on
  } catch (const std::exception& e) {
    std::cerr << "Error reading ini file: " << e.what() << std::endl;
  }
}

}  // namespace drivers
}  // namespace jojo
