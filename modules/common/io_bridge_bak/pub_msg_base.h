#ifndef PUBLISH_MSG_BASE_H
#define PUBLISH_MSG_BASE_H
#include <mutex>
#include <functional>
#include "cyber/common/environment_conf.h"
#include "cyber/common/types.h"

namespace jojo {
namespace cyber {
namespace io_bridge {

// 这个模板类的 msg_type 可以传入不同消息类型，依赖于 Publish() 函数去实现
template <typename msg_type>
class PublishMsgBase {
 public:
  PublishMsgBase() {}
  ~PublishMsgBase() {}

  virtual void Publish(const msg_type& msg) = 0;
};

}  // namespace io_bridge
}  // namespace cyber
}  // namespace jojo

#endif  // PUBLISH_MSG_BASE_H
