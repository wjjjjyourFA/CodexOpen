#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdio>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

class AsyncWriter {
 public:
  explicit AsyncWriter(FILE* fp) : fp_(fp), running_(true) {
    if (fp_ == nullptr) {
      throw std::invalid_argument("AsyncWriter requires a valid FILE pointer");
    }
    writer_thread_ = std::thread(&AsyncWriter::WriteThread, this);
  }

  ~AsyncWriter() { Stop(); }

  bool Push(const char* data, size_t len) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!running_) return false;
    queue_.emplace(data, len);
    cv_.notify_one();
    return true;
  }

  void Stop() {
    running_ = false;
    cv_.notify_all();

    if (writer_thread_.joinable()) writer_thread_.join();
  }

 private:
  void WriteThread() {
    std::vector<std::string> batch;
    batch.reserve(1024);

    while (true) {
      {
        std::unique_lock<std::mutex> lk(mutex_);

        cv_.wait(lk, [&] { return !queue_.empty() || !running_; });

        if (queue_.empty() && !running_) break;

        while (!queue_.empty() && batch.size() < 1024) {
          batch.emplace_back(std::move(queue_.front()));
          queue_.pop();
        }
      }

      for (auto& s : batch) {
        std::fwrite(s.data(), 1, s.size(), fp_);
      }

      batch.clear();
    }

    std::fflush(fp_);
  }

 private:
  FILE* fp_;

  std::queue<std::string> queue_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::thread writer_thread_;
  std::atomic<bool> running_;
};
