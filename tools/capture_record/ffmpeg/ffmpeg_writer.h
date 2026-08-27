#ifndef FFMPEG_WRITER_H
#define FFMPEG_WRITER_H

#include <sys/statvfs.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

#include <opencv2/opencv.hpp>

class FFmpegWriter {
 public:
  FFmpegWriter(int width, int height, int fps, int segment_seconds,
               const std::string& file_path);
  ~FFmpegWriter();

  bool openNewFile();
  int write(const cv::Mat& frame);
  bool rotateIfNeeded();
  bool close();  // 正常停止：成功时发布临时文件
  bool reset();  // 与 close() 相同，保留旧调用点语义
  bool abort();  // 异常停止：关闭进程但不发布临时文件

  bool isInited() const { return initialized_.load(std::memory_order_acquire); }
  bool isRunning() const { return pipe_ != nullptr; }
  const std::string& lastError() const { return last_error_; }

 protected:
  std::string file_path;
  std::string generateVideoFilePath(const std::string& file_path,
                                    const std::string& suffix = ".mp4");

 private:
  bool closeCurrentFile(bool publish);
  bool hasEnoughSpace(const std::string& path,
                      uint64_t threshold_bytes = 10ULL * 1024 * 1024 * 1024);
  void setError(const std::string& error);

  std::atomic_bool initialized_{false};
  FILE* pipe_{nullptr};

  int width_;
  int height_;
  int fps_;
  int segment_seconds_;
  std::chrono::steady_clock::time_point segment_start_;

  std::string ffmpeg_executable_;
  std::string current_tmp_path_;
  std::string current_final_path_;
  std::string last_error_;

  int frame_cnt{0};
  std::chrono::steady_clock::time_point last_space_check_;
  bool first_space_check_{true};
  bool space_enough_{true};
  // 2TB 2ULL * 1024 * 1024 * 1024 * 1024
  // 10GB 10ULL * 1024 * 1024 * 1024
};

#endif
