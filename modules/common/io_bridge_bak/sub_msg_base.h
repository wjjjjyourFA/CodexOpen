#ifndef SUBSCRIBE_MSG_BASE_H
#define SUBSCRIBE_MSG_BASE_H
#include <chrono>
#include <functional>
#include <iostream>
#include <mutex>
#include "cyber/common/environment_conf.h"
#include "cyber/common/types.h"

namespace jojo {
namespace cyber {
namespace io_bridge {

class MsgTime {
 public:
  static double GetSystemTime() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
  }
};

template <typename msg_type>
class SubscribeMsgBase {
 public:
  SubscribeMsgBase(const std::string& topic_name, const std::string& msg_info)
      : receive_flag_(false),
        new_signal_(false),
        topic_name_(topic_name),
        msg_info_(msg_info) {
    // lambda 表达式初始化
    // optional_callback_func_，该函数默认为空，即不执行任何操作。
    optional_callback_func_ = [](const msg_type& msg) { return; };
  }

  msg_type GetMsg();

  bool GetNewMsg(msg_type& msg);

  double GetLastMsgTime() { return last_msg_time_; }

  bool Got() const;

  void Clear();

  void Check(int miss_time_diff);  // ms

  // 通过设置回调函数，暴露给外部数据处理
  void SetOptionalCallbackFunc(
      const std::function<void(const msg_type& msg)>& optional_callback_func);

 protected:
  bool receive_flag_ = false;
  bool new_signal_ = false;
  std::mutex msg_mutex_;
  double last_msg_time_;
  std::string topic_name_ = "";
  std::string msg_info_ = "";
  msg_type msg_;
  std::function<void(const msg_type& msg)> optional_callback_func_;
};

/********************* function of the base class
 * ***************************************/
// 以下是类中函数的实现，==> .cpp

// return the subscribed message, no matter it's new or old
template <typename msg_type>
msg_type SubscribeMsgBase<msg_type>::GetMsg() {
  std::lock_guard<std::mutex> lock_gard(msg_mutex_);
  return msg_;
}

// return a unread message, if there is no unreaded message, return false
template <typename msg_type>
bool SubscribeMsgBase<msg_type>::GetNewMsg(msg_type& msg) {
  // 这里其实是一个不干扰原数据处理的取数据操作
  // 如果取数据的频率比回调函数被触发的频率高，不通过这里就会取到旧数据
  if (new_signal_) {
    std::lock_guard<std::mutex> lock_gard(msg_mutex_);
    msg = msg_;
    new_signal_ = false;

    return true;
  }
  return false;
}

// whether the message is receiving
template <typename msg_type>
bool SubscribeMsgBase<msg_type>::Got() const {
  return receive_flag_;
}

template <typename msg_type>
void SubscribeMsgBase<msg_type>::Clear() {
  new_signal_ = false;
  receive_flag_ = false;
}

// Check the message interrupt time, if it is longer than [miss_time_diff], then
// consider it break off
template <typename msg_type>
void SubscribeMsgBase<msg_type>::Check(int miss_time_diff) {
  if (receive_flag_) {
    double current_time = MsgTime::GetSystemTime();
    double time_diff = current_time - last_msg_time_;

    if (time_diff > miss_time_diff) {
      this->Clear();

      std::cout << "miss " << msg_info_ << " msg [" << time_diff << "ms]"
                << std::endl;
    }
  }
}

// 设置回调函数
template <typename msg_type>
void SubscribeMsgBase<msg_type>::SetOptionalCallbackFunc(
    const std::function<void(const msg_type& msg)>& func) {
  optional_callback_func_ = func;
}

}  // namespace io_bridge
}  // namespace cyber
}  // namespace jojo

#endif  // SUBSCRIBEMSGBASE_H
