#ifndef PUBLISH_DDS_MSG_H
#define PUBLISH_DDS_MSG_H
#include "cyber/io_bridge/pub_msg_base.h"

#ifdef ENABLE_DDS
#include "dds/dds.hpp"
#include "cyber/io_bridge/dds/dds_node_base.h"

namespace jojo {
namespace cyber {
namespace io_bridge {

template <typename msg_type>
class PublishDDSMsg : public PublishMsgBase<msg_type>, public DDSGlobalCache {
 public:
  PublishDDSMsg(uint32_t domain_id, const std::string& topic_name,
                bool zcc_flag, const std::string& network_interface = "");
  virtual ~PublishDDSMsg();

  virtual void Publish(const msg_type& msg) override;

 protected:
  uint32_t domain_id_;
  bool zcc_flag_;
  std::shared_ptr<dds::domain::DomainParticipant> participant_ptr_;
  std::shared_ptr<dds::pub::Publisher> publisher_ptr_;
  std::shared_ptr<dds::topic::Topic<msg_type>> msg_topic_ptr_;
  std::shared_ptr<dds::pub::DataWriter<msg_type>> data_writer_ptr_;

 private:
  mutable std::mutex data_mutex_;
};

template <typename msg_type>
PublishDDSMsg<msg_type>::PublishDDSMsg(uint32_t domain_id,
                                       const std::string& topic_name,
                                       bool zcc_flag,
                                       const std::string& network_interface)
    : domain_id_(domain_id), zcc_flag_(zcc_flag) {
  /*  // 系统中 participant_ptr_ 太多，导致话题匹配复杂度上升
  if (zcc_flag) {
    // clang-format off
    static const std::string shm_config =
        "<KernelSoftDDS><Domain><SharedMemory><Enable>true</Enable>"
        "<LogLevel>verbose</LogLevel></SharedMemory></Domain></KernelSoftDDS>";
    // clang-format on
    participant_ptr_ = std::make_shared<dds::domain::DomainParticipant>(
        domain_id, dds::domain::DomainParticipant::default_participant_qos(),
        nullptr, dds::core::status::StatusMask::none(), shm_config);
    std::cout << "zcc:" << topic_name << std::endl;
  } else {
    if (network_interface.empty()) {
      participant_ptr_ =
          std::make_shared<dds::domain::DomainParticipant>(domain_id);
    } else {
      // clang-format off
      static const std::string dds_config{
          "<KernelSoftDDS><Domain Id=\"any\"><General><Interfaces><NetworkInterface name=\"" +
          network_interface + "\"/></Interfaces></General></Domain></KernelSoftDDS>"};
      // clang-format on
      participant_ptr_ = std::make_shared<dds::domain::DomainParticipant>(
          domain_id, dds::domain::DomainParticipant::default_participant_qos(),
          nullptr, dds::core::status::StatusMask::none(), dds_config);
    }
  }
  */

  participant_ptr_ = GetParticipant(domain_id, zcc_flag, network_interface);

  // 创建订阅者
  publisher_ptr_ = std::make_shared<dds::pub::Publisher>(*participant_ptr_);

  // 创建话题
  msg_topic_ptr_ = std::make_shared<dds::topic::Topic<msg_type>>(
      *participant_ptr_, topic_name);

  dds::pub::qos::DataWriterQos dwqos;
  dwqos << dds::core::policy::Reliability::Reliable();
  dwqos << dds::core::policy::History::KeepLast(50);

  // 创建写对象
  std::lock_guard<std::mutex> lock(data_mutex_);
  data_writer_ptr_ = std::make_shared<dds::pub::DataWriter<msg_type>>(
      *publisher_ptr_, *msg_topic_ptr_, dwqos);
};

template <typename msg_type>
PublishDDSMsg<msg_type>::~PublishDDSMsg() {
  if (data_writer_ptr_) data_writer_ptr_->close();
  if (msg_topic_ptr_) msg_topic_ptr_->close();
  if (publisher_ptr_) publisher_ptr_->close();
  // participant_ptr_ 是共享资源，直接 close 可能会导致其他实例持有的 participant_ptr_ 变为无效状态
  // if (participant_ptr_) participant_ptr_->close();
  std::cout << "~PublishDDSMsg" << std::endl;
}

template <typename msg_type>
void PublishDDSMsg<msg_type>::Publish(const msg_type& msg) {
  data_writer_ptr_->write(msg);
};

}  // namespace io_bridge
}  // namespace cyber
}  // namespace jojo

#else
namespace jojo {
namespace cyber {
namespace io_bridge {

template <typename msg_type>
class PublishDDSMsg : public PublishMsgBase<msg_type> {};

}  // namespace io_bridge
}  // namespace cyber
}  // namespace jojo

#endif  // ENABLE_DDS

#endif  // PUBLISH_DDS_MSG_H
