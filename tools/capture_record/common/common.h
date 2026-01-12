#ifndef COMMON_H
#define COMMON_H

#include <QMetaType>

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
  int crossEndSec;
};

enum RecordState {
  Recording,  // 正在录制
  AutoRecording,  // 自动录制
  Paused,  // 暂停
  Stopped  // 停止
};

#endif  // COMMON_H
