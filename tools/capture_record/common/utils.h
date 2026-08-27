#ifndef UTILS_H
#define UTILS_H

#include <sys/stat.h>

#include <atomic>

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QString>

inline bool checkDevicePermission(const QString& device) {
  QFile file(device);
  return file.exists() && file.open(QIODevice::ReadWrite);
}

inline bool isRealVideoDevice(const QString& path) {
  struct stat s;
  if (stat(path.toStdString().c_str(), &s) == 0) {
    return S_ISCHR(s.st_mode);
  }
  return false;
}

// 生成包含完整日期、毫秒和进程内序号的文件名，并拒绝覆盖现有文件。
// suffix 不需要包含前导点，例如 "mp4"、"avi"、"jpg"。
inline QString generateUniqueMediaFilePath(const QString& directory,
                                           const QString& suffix) {
  static std::atomic<quint64> sequence{0};
  QDir dir(directory);
  if (!dir.exists() && !dir.mkpath(".")) return {};

  const QString normalizedSuffix =
      suffix.startsWith('.') ? suffix.mid(1) : suffix;
  for (int attempt = 0; attempt < 1000; ++attempt) {
    const QString timestamp =
        QDateTime::currentDateTimeUtc().toString("yyyyMMdd'T'HHmmss.zzz'Z'");
    const quint64 seq = sequence.fetch_add(1, std::memory_order_relaxed);
    const QString name = QString("%1_%2.%3")
                             .arg(timestamp)
                             .arg(seq)
                             .arg(normalizedSuffix);
    const QString path = dir.filePath(name);
    if (!QFileInfo::exists(path)) return path;
  }
  return {};
}

#endif  // UTILS_H
