#ifndef FFMPEG_WRITER_H
#define FFMPEG_WRITER_H

#include <cstdio>
#include <string>
#include <stdexcept>
#include <sstream>
#include <cstdlib>
#include <sys/statvfs.h>
#include <iostream>
#include <atomic>

#include <opencv2/opencv.hpp>

class FFmpegWriter {
 public:
  FFmpegWriter(int width, int height, int fps, int segment_seconds,
               const std::string& file_path);
  ~FFmpegWriter() { reset(); }

  bool openNewFile();
  int write(const cv::Mat& frame);
  bool rotateIfNeeded();
  void close();  // 外部停止录制时调用
  void reset();  // 强制关闭

  bool isInited() const { return initialized_; };
  bool isRunning() const { return pipe_ != nullptr; }

 protected:
  std::string file_path;
  std::string generateVideoFilePath(const std::string& file_path,
                                    const std::string& suffix = ".mp4");

 private:
  std::atomic_bool initialized_{false};

  void closeCurrentFile();

  FILE* pipe_ = nullptr;

  int width_;
  int height_;
  int fps_;

  int segment_seconds_;
  std::chrono::steady_clock::time_point segment_start_;

  std::string current_tmp_path_;
  std::string current_final_path_;

  int frame_cnt = 0;

  std::chrono::steady_clock::time_point last_space_check_;
  bool first_space_check_ = true, space_enough_ = true;
  bool hasEnoughSpace(const std::string& path,
                      uint64_t threshold_bytes = 10ULL * 1024 * 1024 * 1024);
  // 2TB 2ULL * 1024 * 1024 * 1024 * 1024
  // 10GB 10ULL * 1024 * 1024 * 1024
};

#endif