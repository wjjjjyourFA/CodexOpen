#ifndef FRAME_QUEUE_H
#define FRAME_QUEUE_H

#pragma once

#include <QMutex>
#include <QMutexLocker>
#include <QWaitCondition>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <queue>
#include <utility>

// 有界、可等待、可关闭的帧队列。统计在 reset() 时按一次录像会话清零。
template <typename T>
class FrameQueue {
 public:
  struct Stats {
    std::size_t queued{0};
    std::uint64_t pushed{0};
    std::uint64_t dropped{0};
    std::uint64_t popped{0};
  };

  explicit FrameQueue(std::size_t max_size = 10)
      : max_size_(std::max<std::size_t>(1, max_size)) {}

  // push 帧，如果满了丢掉最老的。返回 false 表示本次发生了丢帧。
  bool push(T value) {
    QMutexLocker locker(&mutex_);
    if (stopped_) return false;

    const bool dropped = queue_.size() >= max_size_;
    if (dropped) {
      queue_.pop();
      ++dropped_count_;
    }
    queue_.push(std::move(value));
    ++pushed_count_;
    not_empty_.wakeOne();
    return !dropped;
  }

  bool pop(T& value) {
    QMutexLocker locker(&mutex_);
    if (queue_.empty()) return false;
    value = std::move(queue_.front());
    queue_.pop();
    ++popped_count_;
    return true;
  }

  // 等待一帧，stop() 后立即唤醒；若仍有排队帧则先消费完。
  bool waitPop(T& value) {
    QMutexLocker locker(&mutex_);
    while (queue_.empty() && !stopped_) {
      not_empty_.wait(&mutex_);
    }
    if (queue_.empty()) return false;

    value = std::move(queue_.front());
    queue_.pop();
    ++popped_count_;
    return true;
  }

  void stop() {
    QMutexLocker locker(&mutex_);
    stopped_ = true;
    not_empty_.wakeAll();
  }

  void reset() {
    QMutexLocker locker(&mutex_);
    while (!queue_.empty()) queue_.pop();
    stopped_ = false;
    pushed_count_ = 0;
    dropped_count_ = 0;
    popped_count_ = 0;
  }

  std::size_t size() const {
    QMutexLocker locker(&mutex_);
    return queue_.size();
  }

  Stats stats() const {
    QMutexLocker locker(&mutex_);
    return Stats{queue_.size(), pushed_count_, dropped_count_, popped_count_};
  }

  void clear() {
    QMutexLocker locker(&mutex_);
    while (!queue_.empty()) queue_.pop();
  }

 private:
  mutable QMutex mutex_;
  QWaitCondition not_empty_;
  std::queue<T> queue_;
  std::size_t max_size_;
  bool stopped_{false};
  std::uint64_t pushed_count_{0};
  std::uint64_t dropped_count_{0};
  std::uint64_t popped_count_{0};
};

#endif
