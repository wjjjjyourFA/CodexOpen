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

    id = pt.get<int>("general.id", 1);
    std::string interface_str = pt.get<std::string>("general.interface", "eth0");
    std::string filter_str = pt.get<std::string>("general.filter", "");
    packets = pt.get<int>("general.packets", 0);
    port = pt.get<int>("general.port", 31122);
    capture_live = pt.get<int>("general.capture_live", 1);
    std::string path_str = pt.get<std::string>("general.capture_path", "");

    // 拷贝到 C 字符数组
    strncpy(interface, interface_str.c_str(), sizeof(interface)-1);
    strncpy(filter, filter_str.c_str(), sizeof(filter)-1);
    strncpy(capture_path, path_str.c_str(), sizeof(capture_path)-1);

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
