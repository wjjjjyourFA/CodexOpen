#include "modules/drivers/gnss/hc_wrapper/config/runtime_config.h"

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
      type = pt.get<std::string>("general.type", "uart");
      dev_addr = pt.get<std::string>("general.dev_addr", "");
      baudrate = pt.get<uint32_t>("general.baudrate", 57600);
      host = pt.get<std::string>("general.host", "127.0.0.1");
      port = pt.get<int>("general.port", 9904);

      frame_rate = pt.get<int>("topics.frame_rate", 10);
      frame_id = pt.get<std::string>("topics.frame_id", "");
      channel_name = pt.get<std::string>("topics.channel_name", "");

      pub_imu = pt.get<bool>("general.pub_imu", false);
      save = pt.get<int>("general.save", false);
      // 打印其他参数...
    // clang-format on
  } catch (const std::exception& e) {
    std::cerr << "Error reading ini file: " << e.what() << std::endl;
  }
}

}  // namespace tools
}  // namespace jojo