#include "cyber/base/thread_pool_legacy.h"

namespace jojo {
namespace cyber {
namespace base {

ThreadPool::ThreadPool(std::size_t threads,
                                   std::size_t max_task_num)
    : start_(false), stop_(false) {
  threads_num_ = threads;

  // 仿 apollo 的线程池实现，这里暂时不实现 最大限制任务数
  // if (!task_queue_.Init(max_task_num, new BlockWaitStrategy())) {
  //   throw std::runtime_error("Task queue init failed.");
  // }

  this->Start();
}

ThreadPool::~ThreadPool() { Stop(); }

void ThreadPool::Start() {
  if (start_) {
    return;
  }
  start_ = true;
  stop_  = false;

  workers_.reserve(threads_num_);
  for (size_t i = 0; i < threads_num_; ++i) {
    workers_.emplace_back(&ThreadPool::WorkerThread, this);
  }
}

void ThreadPool::Stop() {
  bool expected = false;
  if (!stop_.compare_exchange_strong(expected, true)) {
    return;  // 已经 stop
  }

  {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    stop_ = true;
    // 清空所有未执行的任务
    task_queue_.clear();
  }
  // 通知所有线程 执行一次循环触发 return 退出
  condition_.notify_all();

  for (std::thread &worker : workers_) {
    if (worker.joinable()) {
      worker.join();
    }
  }
  workers_.clear();
  start_ = false;
}

// 简易版，只执行函数，不返回结果
void ThreadPool::Enqueue(std::function<void()> task) {
  {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    // don't allow enqueueing after stopping the pool
    if (stop_) {
      throw std::runtime_error(
          "ThreadPool has been stopped; cannot enqueue new tasks.");
      return;
    }
    task_queue_.emplace_back(std::move(task));
  }
  condition_.notify_one();
}

void ThreadPool::WorkerThread() {
  // while (!stop_) {
  while (true) {
    std::function<void()> task;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      condition_.wait(lock, [this] { return stop_ || !task_queue_.empty(); });
      if (stop_ && task_queue_.empty()) {
        return;  // <-- 只在完全没任务时退出
      }
      task = std::move(task_queue_.front());
      task_queue_.pop_front();
    }
    try {
      task();
    } catch (const std::exception &e) {
      // 记录异常日志，不让线程崩溃
      std::cerr << "[ThreadPool] Task exception: " << e.what()
                << std::endl;
    } catch (...) {
      std::cerr << "[ThreadPool] Unknown task exception" << std::endl;
    }
  }
}

}  // namespace base
}  // namespace cyber
}  // namespace jojo