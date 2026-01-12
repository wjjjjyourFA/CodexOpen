#include "tools/capture_record/ffmpeg/ffmpeg_writer.h"

FFmpegWriter::FFmpegWriter(int width, int height, int fps, int segment_seconds,
                           const std::string& file_path)
    : width_(width),
      height_(height),
      fps_(fps),
      segment_seconds_(segment_seconds) {
  this->file_path = file_path;

  if (openNewFile()) {
    initialized_ = true;
  }
}

bool FFmpegWriter::openNewFile() {
  closeCurrentFile();

  std::string filename = this->generateVideoFilePath(this->file_path);

  current_tmp_path_   = filename + ".tmp";
  current_final_path_ = filename;

  std::ostringstream cmd;
  cmd << "ffmpeg -y "
      << "-loglevel error "
      << "-f rawvideo -pix_fmt bgr24 "
      << "-s " << width_ << "x" << height_ << " "
      << "-r " << fps_ << " "
      << "-i - "
      << "-c:v libx264 "
      << "-pix_fmt yuv420p "
      << "-preset veryfast "
      << "-crf 26 "
      << "-g " << fps_ * 2 << " "
      << "-keyint_min " << fps_ * 2 << " "
      << "-x264-params scenecut=0 "
      << "-f mp4 "
      << "-movflags +faststart+frag_keyframe+empty_moov "
      << "\"" << current_tmp_path_ << "\"";

  // << "-movflags +faststart "
  // << "-movflags +faststart+frag_keyframe+empty_moov "

  pipe_ = popen(cmd.str().c_str(), "w");
  if (!pipe_) {
    throw std::runtime_error("Failed to start ffmpeg");
    return false;
  }
  // 关闭 stdio 缓冲
  setvbuf(pipe_, nullptr, _IONBF, 0);

  segment_start_ = std::chrono::steady_clock::now();

  return true;
}

void FFmpegWriter::closeCurrentFile() {
  if (!pipe_) return;

  fflush(pipe_);
  pclose(pipe_);

  // 正常结束：tmp → mp4
  std::ostringstream mv_cmd;
  mv_cmd << "mv \"" << current_tmp_path_ << "\" \"" << current_final_path_
         << "\"";
  system(mv_cmd.str().c_str());

  // 清空所有状态
  pipe_ = nullptr;
  current_tmp_path_.clear();
  current_final_path_.clear();
}

int FFmpegWriter::write(const cv::Mat& frame) {
  if (!pipe_) {
    // pipe is null
    // 前几帧可能没有初始化结束，也是会返回 null 的
    // 不应该结束任务，而是跳过后续处理
    return -1;
  }

  // 首次写入或者每秒检查一次
  auto now = std::chrono::steady_clock::now();
  if (first_space_check_ ||
      std::chrono::duration_cast<std::chrono::seconds>(now - last_space_check_)
              .count() >= 1) {
    last_space_check_  = now;  // 更新时间戳
    first_space_check_ = false;  // 标记首次检查完成

    // 检查磁盘空间
    if (!this->hasEnoughSpace(this->file_path)) {
      std::cerr << "ffmpeg_writer: not enough disk space" << std::endl;
      this->space_enough_ = false;
      // 磁盘空间不足
      return 3;
    }
  }
  if (!this->space_enough_) {
    return 3;
  }

  // 必须是 BGR + 连续内存
  CV_Assert(frame.type() == CV_8UC3);
  // 保证连续内存
  CV_Assert(frame.isContinuous());
  // cv::Mat writeFrame = frame.isContinuous() ? frame : frame.clone();

  // 避免分辨率动态变化
  CV_Assert(frame.cols == width_);
  CV_Assert(frame.rows == height_);
  // std::cout << "step =" << frame.step << " expect =" << width_ * 3 << std::endl;

  cv::Mat packedFrame;
  if (frame.step != width_ * 3) {
    // 去掉行对齐 padding
    packedFrame = frame.clone();
  } else {
    packedFrame = frame;
  }

  const size_t frame_bytes = width_ * height_ * 3;
  size_t written           = 0;

  while (written < frame_bytes) {
    size_t ret =
        fwrite(packedFrame.data + written, 1, frame_bytes - written, pipe_);
    if (ret == 0) {
      std::cerr << "ffmpeg_writer: fwrite failed" << std::endl;
      // ffmpeg 已挂
      closeCurrentFile();
      // ffmpeg 写入失败
      return 2;
    }
    written += ret;
  }

  if (++frame_cnt % fps_ == 0) {  // 每 1 秒
    frame_cnt = 0;
    fflush(pipe_);
  }

  // 成功写入 frame
  return 0;
}

bool FFmpegWriter::rotateIfNeeded() {
  auto now = std::chrono::steady_clock::now();
  auto elapsed =
      std::chrono::duration_cast<std::chrono::seconds>(now - segment_start_)
          .count();

  if (elapsed >= segment_seconds_) {
    if (!openNewFile()) {
      throw std::runtime_error("Failed to auto open new file");
    }

    return true;
  }

  return false;
}

void FFmpegWriter::close() { closeCurrentFile(); }

void FFmpegWriter::reset() {
  // 关闭当前文件（如果有）
  closeCurrentFile();

  segment_start_ = std::chrono::steady_clock::now();
}

std::string FFmpegWriter::generateVideoFilePath(const std::string& file_path,
                                                const std::string& suffix) {
  // hh-mm-ss
  std::time_t t = std::time(nullptr);
  std::tm tm{};
  localtime_r(&t, &tm);

  char time_buf[32];
  std::strftime(time_buf, sizeof(time_buf), "%H-%M-%S", &tm);

  // 组装文件名
  std::ostringstream oss;
  oss << file_path;
  if (!file_path.empty() && file_path.back() != '/') {
    oss << "/";
  }
  oss << time_buf << "" << suffix;

  return oss.str();
}

bool FFmpegWriter::hasEnoughSpace(const std::string& path,
                                  uint64_t threshold_bytes) {
  // 检查的是目录而不是文件
  // 应该传入视频文件所在的目录而不是完整文件路径
  // 如果 file_path = "/home/user/videos/record.mp4"，应该取 "/home/user/videos"
  struct statvfs stat;
  if (statvfs(path.c_str(), &stat) != 0) {
    std::cerr << "Failed to get disk space for " << path << std::endl;
    return false;  // 失败就安全起见返回 false
  }

  /* debug
  if (frame_cnt > 9) {
    return false;
  }
  */

  uint64_t available = stat.f_bavail * stat.f_frsize;  // 可用空间
  return available >= threshold_bytes;
}