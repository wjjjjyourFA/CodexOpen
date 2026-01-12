#ifndef CYBER_BASE_THREAD_SAFE_QUEUE_H_
#define CYBER_BASE_THREAD_SAFE_QUEUE_H_

#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>
#include <utility>
#include <deque>

namespace jojo {
namespace cyber {
namespace base {

// 线程安全队列
template <typename T>
class ThreadSafeQueue {
 public:
  ThreadSafeQueue() {}
  ThreadSafeQueue& operator=(const ThreadSafeQueue& other) = delete;
  ThreadSafeQueue(const ThreadSafeQueue& other) = delete;

  ~ThreadSafeQueue() { BreakAllWait(); }

  // 插入数据 push()
  void Enqueue(const T& element) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      queue_.emplace_back(element);
    }
    cv_.notify_one();  // 通知一个等待的线程
  }

  // 非阻塞 pop()（如果队列为空，返回 false）
  // 引用一定要绑定到一个已有的对象上
  bool Dequeue(T& element) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.empty()) {
      return false;
    }
    element = std::move(queue_.front());  // ① 移动赋值
    queue_.pop_front();                   // ② 删除队头元素                     
    return true;
  }

  // 阻塞 wait_and_pop（如果队列为空，会等待直到有数据）
  bool WaitDequeue(T& element) {
    std::unique_lock<std::mutex> lock(mutex_);
    // 等待直到不为空或者 break_all_wait_ 为 true
    cv_.wait(lock, [this]() { return break_all_wait_ || !queue_.empty(); });
    if (break_all_wait_) {
      return false;
    }
    element = std::move(queue_.front());
    queue_.pop_front();
    return true;
  }

  typename std::deque<T>::size_type Size() {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
  }

  bool Empty() {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.empty();
  }

  void BreakAllWait() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      break_all_wait_ = true;
    }
    cv_.notify_all();
  }

 private:
  bool break_all_wait_{false};
  mutable std::mutex mutex_;
  std::deque<T> queue_;
  std::condition_variable cv_;
};

}  // namespace base
}  // namespace cyber
}  // namespace jojo

#endif