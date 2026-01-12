#ifndef FRAME_QUEUE_H
#define FRAME_QUEUE_H

#pragma once

#include <QMutex>
#include <QMutexLocker>

#include <queue>
#include <memory>
#include <atomic>

// ----------------- 帧队列 -----------------
template <typename T>
class FrameQueue {
 public:
  // push 帧，如果满了丢掉最老的
  void push(const T& value) {
    QMutexLocker locker(&mutex_);
    if (queue_.size() >= max_size_) {
      queue_.pop();  // 丢掉最老的一帧
    }
    queue_.push(value);
  }

  // pop 返回队列中最老可用帧
  bool pop(T& value) {
    QMutexLocker locker(&mutex_);
    if (queue_.empty()) {
      return false;
    }

    value = queue_.front();
    queue_.pop();
    return true;
  }

  // 获取当前队列大小
  size_t size() {
    QMutexLocker locker(&mutex_);
    return queue_.size();
  }

  // 清空队列
  void clear() {
    QMutexLocker locker(&mutex_);
    while (!queue_.empty()) queue_.pop();
  }

 private:
  QMutex mutex_;
  std::queue<T> queue_;
  size_t max_size_ = 10;
};

#endif