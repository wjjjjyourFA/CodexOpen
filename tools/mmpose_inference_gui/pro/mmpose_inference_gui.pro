TEMPLATE = app
CONFIG += c++14
QT += core gui
QT += widgets multimedia multimediawidgets
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets
CONFIG += object_parallel_to_source
TARGET = mmpose_inference_gui

CODEX_PATH = $$PWD/../../../../CodexOpen
SELF_PATH = $$CODEX_PATH/tools/mmpose_inference_gui

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
# disables all the APIs deprecated before Qt 6.0.0
# DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000

DESTDIR = $$CODEX_PATH/install/bin/tools/mmpose_inference

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
MOC_DIR = $$CODEX_PATH/tools/mmpose_inference_gui/moc
RCC_DIR = $$CODEX_PATH/tools/mmpose_inference_gui/rcc

SOURCES += \
  $$SELF_PATH/main.cpp \
  $$SELF_PATH/videoplayerwidget.cpp \
  $$SELF_PATH/mainwidget.cpp

HEADERS += \
  $$SELF_PATH/videoplayerwidget.h \
  $$SELF_PATH/mainwidget.h

INCLUDEPATH += \
  $$CODEX_PATH \
  
FORMS += \
  $$SELF_PATH/ui/mainwidget.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target