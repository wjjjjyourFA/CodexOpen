#ifndef CYBER_BASE_THREAD_POOL_SIMPLE_H_
#define CYBER_BASE_THREAD_POOL_SIMPLE_H_

#include <iostream>
#include <atomic>
#include <functional>
#include <future>
#include <queue>
#include <thread>
#include <vector>
#include <mutex>
#include <condition_variable>

namespace jojo {
namespace cyber {
namespace base {

class ThreadPool {
 public:
  // 获取全局唯一实例
  static std::shared_ptr<ThreadPool> Instance(size_t threads = -1) {
    // 线程安全且只创建一次
    static std::shared_ptr<ThreadPool> instance = nullptr;
    static std::once_flag flag;

    std::call_once(flag, [&]() {
      size_t use_threads = (threads > 0 ? threads : 4);
      instance           = std::make_shared<ThreadPool>(use_threads);
    });

    return instance;
  }

  explicit ThreadPool(std::size_t threads,
                            std::size_t max_task_num = 100);
  virtual ~ThreadPool();

  // 禁止拷贝与赋值
  ThreadPool(const ThreadPool &)            = delete;
  ThreadPool &operator=(const ThreadPool &) = delete;

  // 防止默认构造被外部调用
  ThreadPool() = delete;

  // enqueue 不返回值的任务
  void Enqueue(std::function<void()> task);

  // enqueue 带返回值任务
  // Lambda 返回 void，C++ 优先选择模板匹配
  template <class F, class... Args>
  auto Enqueue(F &&f, Args &&...args)
      -> std::future<std::invoke_result_t<F, Args...>>;

  void Start();
  void Stop();

 private:
  std::vector<std::thread> workers_;
  std::deque<std::function<void()>> task_queue_;
  std::atomic<bool> start_{false}, stop_{false};

  size_t threads_num_;
  std::mutex queue_mutex_;
  std::condition_variable condition_;

  void WorkerThread();
};

// before using the return value, you should check value.valid()
template <class F, class... Args>
auto ThreadPool::Enqueue(F &&f, Args &&...args)
    -> std::future<std::invoke_result_t<F, Args...>> {
  using return_type = std::invoke_result_t<F, Args...>;

  auto task = std::make_shared<std::packaged_task<return_type()>>(
      std::bind(std::forward<F>(f), std::forward<Args>(args)...));

  std::future<return_type> res = task->get_future();

  {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    // don't allow enqueueing after stopping the pool
    if (stop_) {
      throw std::runtime_error(
          "ThreadPool has been stopped; cannot enqueue new tasks.");
      return std::future<return_type>();
    }
    task_queue_.emplace_back([task]() { (*task)(); });
  }
  condition_.notify_one();

  return res;
};

}  // namespace base
}  // namespace cyber
}  // namespace jojo

#endif
