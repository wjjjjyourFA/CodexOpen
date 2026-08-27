#ifndef COMMON_H
#define COMMON_H

#include <QImage>
#include <QMetaType>
#include <QString>
#include <memory>

Q_DECLARE_METATYPE(std::shared_ptr<QImage>)

struct CustomSettings {
  QString filePath       = "";
  float fps              = 10.0;
  int width              = 1920;
  int height             = 1080;
  int segmentSeconds     = 30 * 60;
  int recordStartSeconds = 8 * 3600;
  int recordEndSeconds   = 20 * 3600;
  bool isCrossDay        = false;
  int crossEndSec        = 0;
};

enum class RecordState {
  Starting,  // writer 正在启动
  Recording,  // 手动录制
  AutoRecording,  // 自动录制
  Paused,  // 暂停
  Stopping,  // 正在停止并发布文件
  Stopped,  // 停止
  Error  // writer/编码错误
};
Q_DECLARE_METATYPE(RecordState)

inline bool isWithinDailyRecordWindow(int nowSeconds, int startSeconds,
                                      int endSeconds) {
  constexpr int kSecondsPerDay = 24 * 60 * 60;
  if (nowSeconds < 0 || nowSeconds >= kSecondsPerDay || startSeconds < 0 ||
      startSeconds >= kSecondsPerDay || endSeconds < 0 ||
      endSeconds >= kSecondsPerDay || startSeconds == endSeconds) {
    return false;
  }

  if (startSeconds < endSeconds) {
    return nowSeconds >= startSeconds && nowSeconds < endSeconds;
  }
  return nowSeconds >= startSeconds || nowSeconds < endSeconds;
}

#endif  // COMMON_H
