#ifndef DDS_NODE_BASE_H
#define DDS_NODE_BASE_H
#include "cyber/common/environment_conf.h"

#ifdef ENABLE_DDS
#include "dds/dds.hpp"

namespace jojo {
namespace cyber {
namespace io_bridge {

struct ParticipantCache {
  std::mutex mutex;
  std::map<std::tuple<uint32_t, bool, std::string>,
           std::weak_ptr<dds::domain::DomainParticipant>> map;

  ParticipantCache() = default;

  std::shared_ptr<dds::domain::DomainParticipant> FindOrCreate(
      uint32_t domain_id, bool zcc_flag, const std::string& network_interface,
      std::function<std::shared_ptr<dds::domain::DomainParticipant>()> creator) {
    auto key = std::make_tuple(domain_id, zcc_flag, network_interface);

    std::lock_guard<std::mutex> lock(mutex);

    auto it = map.find(key);
    if (it != map.end()) {
      if (auto sp = it->second.lock()) {
        return sp;
      }
      map.erase(it);
    }

    auto sp = creator();
    map.emplace(key, sp);

    return sp;
  }
};

class DdsGlobalCache {
 protected:
  static ParticipantCache& GetCache() {
    static ParticipantCache cache;  // 全局唯一
    return cache;
  }

  std::shared_ptr<dds::domain::DomainParticipant> GetParticipant(
      uint32_t domain_id, bool zcc_flag, const std::string& network_interface) {
    auto creator = [&]() -> std::shared_ptr<dds::domain::DomainParticipant> {
      std::cout << "create DomainParticipant" << std::endl;
      if (zcc_flag) {
        // clang-format off
        static const std::string shm_config =
            "<KernelSoftDDS><Domain><SharedMemory><Enable>true</Enable>"
            "<LogLevel>verbose</LogLevel></SharedMemory></Domain></KernelSoftDDS>";
        // clang-format on

        return std::make_shared<dds::domain::DomainParticipant>(
            domain_id,
            dds::domain::DomainParticipant::default_participant_qos(), nullptr,
            dds::core::status::StatusMask::none(), shm_config);
      }

      if (network_interface.empty()) {
        return std::make_shared<dds::domain::DomainParticipant>(domain_id);
      }

      static const std::string dds_config_prefix =
          "<KernelSoftDDS><Domain Id=\"any\"><General><Interfaces>"
          "<NetworkInterface name=\"";
      static const std::string dds_config_suffix =
          "\"/></Interfaces></General></Domain></KernelSoftDDS>";

      return std::make_shared<dds::domain::DomainParticipant>(
          domain_id, dds::domain::DomainParticipant::default_participant_qos(),
          nullptr, dds::core::status::StatusMask::none(),
          dds_config_prefix + network_interface + dds_config_suffix);
    };

    return GetCache().FindOrCreate(domain_id, zcc_flag, network_interface, creator);
  }
};

}  // namespace io_bridge
}  // namespace cyber
}  // namespace jojo

#endif

#endif  // DDS_NODE_BASE_H
