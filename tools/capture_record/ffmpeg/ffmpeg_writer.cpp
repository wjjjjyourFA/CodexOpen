#include "tools/capture_record/ffmpeg/ffmpeg_writer.h"

#include <sys/wait.h>

#include <QByteArray>
#include <QFileInfo>
#include <QProcess>
#include <QStandardPaths>
#include <QStringList>
#include <mutex>

namespace {

std::string shellQuote(const std::string& value) {
  std::string quoted("'");
  for (char ch : value) {
    if (ch == '\'') {
      quoted += "'\"'\"'";
    } else {
      quoted += ch;
    }
  }
  quoted += '\'';
  return quoted;
}

void ignoreSigPipeOnce() {
  static std::once_flag flag;
  std::call_once(flag, [] { std::signal(SIGPIPE, SIG_IGN); });
}

struct FFmpegProbeResult {
  bool ok{false};
  QString executable;
  QString error;
};

const FFmpegProbeResult& probeFFmpegOnce() {
  static const FFmpegProbeResult result = [] {
    FFmpegProbeResult probe;
    probe.executable = QStandardPaths::findExecutable("ffmpeg");
    if (probe.executable.isEmpty()) {
      probe.error = "ffmpeg executable was not found in PATH";
      return probe;
    }

    QProcess process;
    process.setProgram(probe.executable);
    process.setArguments(
        {"-hide_banner", "-loglevel", "error", "-encoders"});
    process.setProcessChannelMode(QProcess::MergedChannels);
    process.start(QIODevice::ReadOnly);
    if (!process.waitForStarted(3000)) {
      probe.error = QString("failed to start ffmpeg probe: %1")
                        .arg(process.errorString());
      return probe;
    }
    if (!process.waitForFinished(10000)) {
      process.kill();
      process.waitForFinished(3000);
      probe.error = "ffmpeg encoder probe timed out";
      return probe;
    }

    const QByteArray output = process.readAll();
    if (process.exitStatus() != QProcess::NormalExit ||
        process.exitCode() != 0) {
      probe.error = QString("ffmpeg encoder probe failed: %1")
                        .arg(QString::fromLocal8Bit(output).trimmed());
      return probe;
    }
    if (!output.contains("libx264")) {
      probe.error = "ffmpeg does not provide the libx264 encoder";
      return probe;
    }

    probe.ok = true;
    return probe;
  }();
  return result;
}

}  // namespace

FFmpegWriter::FFmpegWriter(int width, int height, int fps, int segment_seconds,
                           const std::string& file_path)
    : file_path(file_path),
      width_(width),
      height_(height),
      fps_(fps),
      segment_seconds_(segment_seconds) {
  ignoreSigPipeOnce();

  if (width_ <= 0 || height_ <= 0 || fps_ <= 0 || segment_seconds_ <= 0 ||
      file_path.empty()) {
    setError("invalid FFmpeg writer arguments");
    return;
  }

  const FFmpegProbeResult& probe = probeFFmpegOnce();
  if (!probe.ok) {
    setError(probe.error.toStdString());
    return;
  }
  ffmpeg_executable_ = probe.executable.toStdString();

  if (openNewFile()) initialized_.store(true, std::memory_order_release);
}

FFmpegWriter::~FFmpegWriter() { closeCurrentFile(true); }

void FFmpegWriter::setError(const std::string& error) {
  last_error_ = error;
  std::cerr << "FFmpegWriter: " << error << std::endl;
}

bool FFmpegWriter::openNewFile() {
  if (!closeCurrentFile(true)) return false;

  std::string filename = generateVideoFilePath(file_path);
  if (filename.empty()) {
    setError("failed to generate a unique output path");
    return false;
  }

  current_tmp_path_ = filename + ".tmp";
  current_final_path_ = filename;

  std::ostringstream cmd;
  cmd << shellQuote(ffmpeg_executable_) << " -n "
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
      << shellQuote(current_tmp_path_);

  pipe_ = popen(cmd.str().c_str(), "w");
  if (!pipe_) {
    setError(std::string("popen failed: ") + std::strerror(errno));
    current_tmp_path_.clear();
    current_final_path_.clear();
    return false;
  }
  setvbuf(pipe_, nullptr, _IONBF, 0);

  segment_start_ = std::chrono::steady_clock::now();
  initialized_.store(true, std::memory_order_release);
  first_space_check_ = true;
  space_enough_ = true;
  frame_cnt = 0;
  last_error_.clear();
  return true;
}

bool FFmpegWriter::closeCurrentFile(bool publish) {
  if (!pipe_) return true;

  const bool flush_ok = fflush(pipe_) == 0;
  const int status = pclose(pipe_);
  pipe_ = nullptr;
  initialized_.store(false, std::memory_order_release);

  const bool process_ok =
      flush_ok && status != -1 && WIFEXITED(status) && WEXITSTATUS(status) == 0;
  if (!process_ok) {
    setError("ffmpeg flush/exit failed; keeping temporary file: " +
             current_tmp_path_);
    publish = false;
  }

  bool published = process_ok;
  if (publish && process_ok) {
    if (std::rename(current_tmp_path_.c_str(), current_final_path_.c_str()) != 0) {
      setError(std::string("failed to publish recording: ") +
               std::strerror(errno));
      published = false;
    }
  } else if (!publish) {
    published = false;
  }

  current_tmp_path_.clear();
  current_final_path_.clear();
  return publish ? published : process_ok;
}

int FFmpegWriter::write(const cv::Mat& frame) {
  if (!pipe_) {
    // pipe is null
    // 前几帧可能没有初始化结束，也是会返回 null 的
    // 不应该结束任务，而是跳过后续处理
    return -1;
  }

  // 首次写入或者每秒检查一次
  const auto now = std::chrono::steady_clock::now();
  if (first_space_check_ ||
      std::chrono::duration_cast<std::chrono::seconds>(now - last_space_check_)
              .count() >= 1) {
    last_space_check_  = now;  // 更新时间戳
    first_space_check_ = false;  // 标记首次检查完成

    // 检查磁盘空间
    if (!hasEnoughSpace(file_path)) {
      space_enough_ = false;
      setError("not enough disk space");
	  // 磁盘空间不足
      return 3;
    }
  }
  if (!space_enough_) return 3;

  if (frame.empty() || frame.type() != CV_8UC3 || frame.cols != width_ ||
      frame.rows != height_) {
    setError("invalid input frame");
    return 5;
  }

  cv::Mat packed_frame;
  if (!frame.isContinuous() || frame.step != static_cast<size_t>(width_ * 3)) {
    // 去掉行对齐 padding
    packed_frame = frame.clone();
  } else {
    packed_frame = frame;
  }

  const size_t frame_bytes = static_cast<size_t>(width_) * height_ * 3;
  size_t written = 0;
  while (written < frame_bytes) {
    const size_t ret =
        fwrite(packed_frame.data + written, 1, frame_bytes - written, pipe_);
    if (ret == 0) {
      setError("fwrite to ffmpeg failed");
	  // ffmpeg 已挂
      closeCurrentFile(false);
      // ffmpeg 写入失败
      return 2;
    }
    written += ret;
  }

  if (++frame_cnt % fps_ == 0) {
    frame_cnt = 0;
    if (fflush(pipe_) != 0) {
      setError("fflush to ffmpeg failed");
      closeCurrentFile(false);
      return 2;
    }
  }
  // 成功写入 frame
  return 0;
}

bool FFmpegWriter::rotateIfNeeded() {
  const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                           std::chrono::steady_clock::now() - segment_start_)
                           .count();
  if (elapsed < segment_seconds_) return false;
  if (!openNewFile()) throw std::runtime_error(last_error_);
  return true;
}

bool FFmpegWriter::close() { return closeCurrentFile(true); }
bool FFmpegWriter::reset() { return close(); }
bool FFmpegWriter::abort() { return closeCurrentFile(false); }

std::string FFmpegWriter::generateVideoFilePath(const std::string& directory,
                                                const std::string& suffix) {
  static std::atomic<uint64_t> sequence{0};
  for (int attempt = 0; attempt < 1000; ++attempt) {
    std::time_t t = std::time(nullptr);
    std::tm tm{};
    gmtime_r(&t, &tm);

    char time_buf[32];
    std::strftime(time_buf, sizeof(time_buf), "%Y%m%dT%H%M%S", &tm);
    const auto millis =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count() %
        1000;

    // 组装文件名
    std::ostringstream oss;
    oss << directory;
    if (!directory.empty() && directory.back() != '/') oss << '/';
    oss << time_buf << '.' << std::setfill('0') << std::setw(3) << millis << 'Z'
        << '_' << sequence.fetch_add(1, std::memory_order_relaxed) << suffix;
    const std::string candidate = oss.str();
    if (!QFileInfo::exists(QString::fromStdString(candidate)) &&
        !QFileInfo::exists(QString::fromStdString(candidate + ".tmp"))) {
      return candidate;
    }
  }
  return {};
}

bool FFmpegWriter::hasEnoughSpace(const std::string& path,
                                  uint64_t threshold_bytes) {
  // 检查的是目录而不是文件
  // 应该传入视频文件所在的目录而不是完整文件路径
  // 如果 file_path = "/home/user/videos/record.mp4"，应该取 "/home/user/videos"
  struct statvfs stat {};
  if (statvfs(path.c_str(), &stat) != 0) {
    setError("failed to get disk space for " + path);
	// 失败就安全起见返回 false
    return false;
  }
  const uint64_t available = stat.f_bavail * stat.f_frsize;
  return available >= threshold_bytes;
}
