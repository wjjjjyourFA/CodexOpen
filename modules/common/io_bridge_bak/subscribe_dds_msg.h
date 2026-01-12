#ifndef SUBSCRIBE_DDS_MSG_H
#define SUBSCRIBE_DDS_MSG_H
#include "cyber/io_bridge/sub_msg_base.h"

#ifdef ENABLE_DDS
#include "dds/dds.hpp"
#include "cyber/io_bridge/dds/dds_node_base.h"

namespace jojo {
namespace cyber {
namespace io_bridge {

template <typename msg_type>
class SubscribeDDSMsg
    : public SubscribeMsgBase<msg_type>,
      public virtual dds::sub::NoOpDataReaderListener<msg_type>,
      DDSGlobalCache {
 public:
  SubscribeDDSMsg(const std::string& topic_name, const std::string& msg_info,
                  uint32_t domain_id, bool zcc_flag,
                  const std::string& network_interface = "");
  virtual ~SubscribeDDSMsg();

 protected:
  // ==> Callback()
  virtual void on_data_available(
      dds::sub::DataReader<msg_type>& reader) override;

  virtual void PrintCallbackInfo() {}

 protected:
  uint32_t domain_id_;
  bool zcc_flag_;
  std::shared_ptr<dds::domain::DomainParticipant> participant_ptr_;
  std::shared_ptr<dds::sub::Subscriber> subscriber_ptr_;
  std::shared_ptr<dds::topic::Topic<msg_type>> msg_topic_ptr_;
  std::shared_ptr<dds::sub::DataReader<msg_type>> data_reader_ptr_;

 private:
  mutable std::mutex data_mutex_;
};

// 以下是类中函数的实现，==> .cpp
template <typename msg_type>
SubscribeDDSMsg<msg_type>::SubscribeDDSMsg(const std::string& topic_name,
                                           const std::string& msg_info,
                                           uint32_t domain_id, bool zcc_flag,
                                           const std::string& network_interface)
    : SubscribeMsgBase<msg_type>(topic_name, msg_info),
      domain_id_(domain_id),
      zcc_flag_(zcc_flag) {
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

  participant_ptr_ = GetParticipant(domain_id_, zcc_flag_, network_interface);

  // 创建订阅者
  subscriber_ptr_ = std::make_shared<dds::sub::Subscriber>(*participant_ptr_);

  // 创建话题
  msg_topic_ptr_ = std::make_shared<dds::topic::Topic<msg_type>>(
      *participant_ptr_, topic_name);

  // DataReader服务质量对象，DataReader构造函数接口需要
  dds::sub::qos::DataReaderQos drqos;
  drqos << dds::core::policy::Reliability::Reliable();
  drqos << dds::core::policy::History::KeepLast(50);

  // 创建读对象
  std::lock_guard<std::mutex> lock(data_mutex_);
  data_reader_ptr_ = std::make_shared<dds::sub::DataReader<msg_type>>(
      *subscriber_ptr_, *msg_topic_ptr_, drqos, this,
      dds::core::status::StatusMask::data_available());
}

template <typename msg_type>
SubscribeDDSMsg<msg_type>::~SubscribeDDSMsg() {
  if (data_reader_ptr_) data_reader_ptr_->close();
  if (msg_topic_ptr_) msg_topic_ptr_->close();
  if (subscriber_ptr_) subscriber_ptr_->close();
  // participant_ptr_ 是共享资源，直接close可能会导致其他实例持有的 participant_ptr_ 变为无效状态
  // if (participant_ptr_) participant_ptr_->close();
  std::cout << "~SubscribeDDSMsg" << std::endl;
}

template <typename msg_type>
void SubscribeDDSMsg<msg_type>::on_data_available(
    dds::sub::DataReader<msg_type>& reader) {
  dds::sub::LoanedSamples<msg_type> samples;

  // take(): 读取并清空历史缓存
  // read(): 读取但不清空历史缓存
  samples = reader.take();

  // 判断是否有数据
  if (samples.length() > 0) {
    auto sample_iter = samples.begin();
    for (; sample_iter < samples.end(); ++sample_iter) {
      const dds::sub::SampleInfo& info = sample_iter->info();

      // 检查当前数据是否有效
      if (info.valid()) {
        std::lock_guard<std::mutex> lock_gard(this->msg_mutex_);

        this->last_msg_time_ = MsgTime::GetSystemTime();

        this->msg_ = std::move(sample_iter->data());

        this->optional_callback_func_(this->msg_);

        this->new_signal_ = true;

        if (!this->receive_flag_) {
          this->receive_flag_ = true;
          std::cout << "DDS SUB ----> get " << this->msg_info_ << std::endl;
        }

        PrintCallbackInfo();
      }
    }
  }
}

}  // namespace io_bridge
}  // namespace cyber
}  // namespace jojo

#else
namespace jojo {
namespace cyber {
namespace io_bridge {

template <typename msg_type>
class SubscribeDDSMsg : public SubscribeMsgBase<msg_type> {};

}  // namespace io_bridge
}  // namespace cyber
}  // namespace jojo

#endif  // ENABLE_DDS

#endif  // SUBSCRIBEDDSMSG_H
