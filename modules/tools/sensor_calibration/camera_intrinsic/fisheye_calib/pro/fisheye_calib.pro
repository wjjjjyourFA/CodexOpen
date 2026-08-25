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
TARGET = fisheye_calib

CODEX_PATH = $$PWD/../../../../../../../CodexOpen
PREFIX = $$CODEX_PATH/modules/tools/sensor_calibration
SELF_PATH = $$CODEX_PATH/modules/tools/sensor_calibration/camera_intrinsic/fisheye_calib

DESTDIR = $$CODEX_PATH/install/bin/modules/tools/sensor_calibration

CONFIG(debug, debug|release) {
  OBJECTS_DIR = $$CODEX_PATH/build/Debug
  QMAKE_CXXFLAGS += -O1
}
CONFIG(release, debug|release) {
  OBJECTS_DIR = $$CODEX_PATH/build/Release
  QMAKE_CXXFLAGS += -O3
}

SOURCES += \
  $$CODEX_PATH/cyber/binary.cc \
  $$CODEX_PATH/cyber/common/file.cc \
  $$CODEX_PATH/modules/common/config/config_file_base.cpp \
  $$PREFIX/common/yaml_writer.cpp \
  $$PREFIX/camera_intrinsic/intrinsic_calib/intrinsic_calibration.cpp \
  $$PREFIX/camera_intrinsic/intrinsic_calib/auto_image_picker.cpp \
  $$PREFIX/camera_intrinsic/intrinsic_calib/config/runtime_config.cpp \
  $$SELF_PATH/run_fisheye_calibration.cpp \
  $$SELF_PATH/fisheye_calibration.cpp \

HEADERS += \
  $$CODEX_PATH/cyber/binary.h \
  $$CODEX_PATH/cyber/common/file.h \
  $$CODEX_PATH/modules/common/config/config_file_base.h \
  $$PREFIX/common/point.hpp \
  $$PREFIX/common/yaml_writer.h \
  $$PREFIX/camera_intrinsic/intrinsic_calib/intrinsic_calibration.hpp \
  $$PREFIX/camera_intrinsic/intrinsic_calib/auto_image_picker.hpp \
  $$PREFIX/camera_intrinsic/intrinsic_calib/config/runtime_config.h \
  $$SELF_PATH/fisheye_calibration.h

INCLUDEPATH += \
  $$CODEX_PATH \

INCLUDEPATH += \
  /usr/include/eigen3 \
  /usr/include/opencv4

## BASE
LIBS += \
  -L/usr/local/lib \
  -L/usr/lib/x86_64-linux-gnu \
  -L/usr/lib/aarch64-linux-gnu \
  -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_ml -lopencv_calib3d \
  -lopencv_video -lopencv_flann -lopencv_features2d -lopencv_imgcodecs \
  -lboost_filesystem -lboost_system -lboost_thread \
  -lgomp -lglog -lprotobuf
