#-------------------------------------------------
#
# Project created by QtCreator 2025-09-11T15:51:03
#
#-------------------------------------------------
TEMPLATE = app
QT += multimedia
QT += multimedia multimediawidgets
QT += widgets
CONFIG += object_parallel_to_source
TARGET = capture_record_cv

CODEX_PATH = $$PWD/../../../../CodexOpen
PREFIX = $$CODEX_PATH/modules/drivers
SELF_PATH = $$CODEX_PATH/tools/capture_record

DESTDIR = $$CODEX_PATH/install/bin/tools/capture_record

CONFIG(debug, debug|release) {
  OBJECTS_DIR = $$CODEX_PATH/build/Debug
  QMAKE_CXXFLAGS += -O1
}
CONFIG(release, debug|release) {
  OBJECTS_DIR = $$CODEX_PATH/build/Release
  QMAKE_CXXFLAGS += -O3
}

# 指定中间文件目录（moc, ui, qrc 生成文件都会放到这里）
UI_DIR  = $$SELF_PATH/pro/ui
MOC_DIR = $$CODEX_PATH/tools/capture_record/moc
RCC_DIR = $$CODEX_PATH/tools/capture_record/rcc

SOURCES += \
  $$SELF_PATH/main.cpp \
  $$SELF_PATH/imagesettings.cpp \
  $$SELF_PATH/videosettings.cpp \
  $$SELF_PATH/opencv/mainwindow_cv.cpp \
  $$SELF_PATH/opencv/camera_device.cpp \

HEADERS += \
  $$SELF_PATH/imagesettings.h \
  $$SELF_PATH/videosettings.h \
  $$SELF_PATH/opencv/mainwindow_cv.h \
  $$SELF_PATH/opencv/camera_device.h \
  $$SELF_PATH/common/common.h \
  $$SELF_PATH/common/utils.h \

INCLUDEPATH += \
  $$CODEX_PATH \

INCLUDEPATH += \
  /usr/include/opencv4 \
  /usr/local/include \

FORMS += \
  $$SELF_PATH/ui/mainwindow.ui \
  $$SELF_PATH/ui/videosettings.ui \
  $$SELF_PATH/ui/imagesettings.ui

LIBS += \
  -L/usr/local/lib \
  -L/usr/lib/aarch64-linux-gnu \
  -L/usr/lib/x86_64-linux-gnu \
  -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_imgcodecs \
  -lopencv_videoio -lopencv_video \
  # -lboost_filesystem -lboost_system -lboost_thread \
