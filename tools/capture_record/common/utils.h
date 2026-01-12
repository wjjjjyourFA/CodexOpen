#ifndef UTILS_HH
#define UTILS_HH

#include <sys/stat.h>

#include <QFile>

inline bool checkDevicePermission(const QString& device) {
  // qDebug() << device;
  QFile file(device);
  return file.exists() && file.open(QIODevice::ReadWrite);
}

inline bool isRealVideoDevice(const QString& path) {
  struct stat s;
  if (stat(path.toStdString().c_str(), &s) == 0) {
    return S_ISCHR(s.st_mode);  // 只保留字符设备
  }
  return false;
}

#endif  // UTILS_HH