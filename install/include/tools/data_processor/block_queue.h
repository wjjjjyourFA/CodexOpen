#pragma once

#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

// ============================================================
//  BlockingQueue —— 单生产者单消费者安全队列
// ============================================================

/**
 * @brief 线程安全的阻塞队列
 *
 * Reader（主线程）push，Worker（子线程）pop。
 * 当 Reader 调用 Close() 后，Worker 的 Pop() 会在队列清空时返回 false，
 * Worker 线程自然退出。
 *
 * @tparam T 队列元素类型
 */
template <typename T>
class BlockingQueue {
 public:
  explicit BlockingQueue(size_t max_size = 64)
      : max_size_(max_size), closed_(false) {}

  /**
   * @brief 生产者推入元素（阻塞直到队列有空间）
   * @return false 如果队列已关闭
   */
  bool Push(T item) {
    std::unique_lock<std::mutex> lock(mutex_);
    not_full_.wait(lock,
                   [this] { return queue_.size() < max_size_ || closed_; });
    if (closed_) return false;
    queue_.push(std::move(item));
    not_empty_.notify_one();
    return true;
  }

  /**
   * @brief 消费者弹出元素（阻塞直到有数据或关闭）
   * @param[out] item 弹出的元素
   * @return false 表示队列已关闭且为空，Worker 应退出
   */
  bool Pop(T& item) {
    std::unique_lock<std::mutex> lock(mutex_);
    not_empty_.wait(lock, [this] { return !queue_.empty() || closed_; });
    if (queue_.empty()) return false;  // closed + empty → Worker 退出
    item = std::move(queue_.front());
    queue_.pop();
    not_full_.notify_one();
    return true;
  }

  /**
   * @brief 关闭队列，通知所有等待的消费者退出
   *        必须在所有 Push 完成后调用
   */
  void Close() {
    std::unique_lock<std::mutex> lock(mutex_);
    closed_ = true;
    not_empty_.notify_all();
    not_full_.notify_all();
  }

  bool IsClosed() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return closed_;
  }

 private:
  std::queue<T> queue_;
  mutable std::mutex mutex_;
  std::condition_variable not_empty_;
  std::condition_variable not_full_;
  size_t max_size_;
  bool closed_;
};
