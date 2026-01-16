MY_SYSTEM_INFO = 20.04.1
unix{
 MY_SYSTEM_INFO = $$system(uname -a | cut -d \~ -f 2 | cut -d \- -f 1)
 message("System Is Ubuntu" $${QT_ARCH} : $${MY_SYSTEM_INFO})
}

TEMPLATE = app
CONFIG += console c++14
CONFIG -= app_bundle
CONFIG -= qt
CONFIG += object_parallel_to_source
TARGET = image_undistortion

CODEX_PATH = $$PWD/../../../../../CodexOpen
PREFIX = $$CODEX_PATH/modules/perception/common
SELF_PATH = $$CODEX_PATH/modules/tools/image_undistortion

DESTDIR = $$CODEX_PATH/install/bin/modules/tools/image_undistortion

CONFIG(debug, debug|release) {
  OBJECTS_DIR = $$CODEX_PATH/build/Debug
  QMAKE_CXXFLAGS += -O1
}
CONFIG(release, debug|release) {
  OBJECTS_DIR = $$CODEX_PATH/build/Release
  QMAKE_CXXFLAGS += -O3
}

# 启用 OpenMP 支持
#CONFIG += openmp

SOURCES += \
  $$CODEX_PATH/cyber/binary.cc \
  $$CODEX_PATH/cyber/common/file.cc \
  $$CODEX_PATH/modules/common/config/config_file_base.cpp \
  $$PREFIX/base/camera.cc \
  $$PREFIX/base/distortion_model.cc \
  $$PREFIX/config/utils.cpp \
  $$PREFIX/camera/common/undistortion_handler_legacy.cc \
  $$PREFIX/camera/params/*.cpp \
  $$SELF_PATH/run_image_undistortion.cpp \
  $$SELF_PATH/config/*.cpp \

HEADERS += \
  $$CODEX_PATH/cyber/binary.h \
  $$CODEX_PATH/cyber/common/file.h \
  $$CODEX_PATH/modules/common/config/config_file_base.h \
  $$PREFIX/base/camera.h \
  $$PREFIX/base/distortion_model.h \
  $$PREFIX/config/utils.h \
  $$PREFIX/camera/common/undistortion_handler_legacy.h \
  $$PREFIX/camera/params/*.h \
  $$SELF_PATH/config/*.h \

INCLUDEPATH += \
  /usr/include/eigen3 \
  /usr/include/opencv4 \
  /usr/local/include/opencv4 \
  /usr/local/include \
  $$CODEX_PATH

LIBS += \
  -L/usr/local/lib \
  -L/usr/lib/aarch64-linux-gnu \
  -L/usr/lib/x86_64-linux-gnu/ \
  -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_imgcodecs -lopencv_calib3d \
  -lgomp -lglog -lprotobuf


