// #include "tools/capture_record/qt5/mainwindow_qt5.h"
// #include "tools/capture_record/opencv/mainwindow_cv.h"
// #include "tools/capture_record/v4l2/mainwindow_v4l2.h"
#include "tools/capture_record/ffmpeg/mainwindow_ffmpeg.h"

#include <QtWidgets>

int main(int argc, char* argv[]) {
  qRegisterMetaType<std::shared_ptr<QImage>>("std::shared_ptr<QImage>");

  // QApplication 是 Qt 提供的 GUI 应用程序核心类
  // 管理应用程序的控制流和主要设置，比如窗口系统、事件循环等
  QApplication app(argc, argv);

  MainWindow main_window;
  main_window.show();

  // app.exec() 启动 Qt 的事件循环。
  // GUI 程序必须有事件循环，否则窗口无法响应用户操作（比如鼠标点击、键盘输入）。
  return app.exec();
}
