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
TARGET = calib_verification

CODEX_PATH = $$PWD/../../../../../../../CodexOpen
PREFIX = $$CODEX_PATH/modules/perception
SELF_PATH = $$CODEX_PATH/modules/tools/sensor_calibration/camera_intrinsic/calib_verification

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
  $$CODEX_PATH/modules/common/config/config_file_base.cpp \
  $$CODEX_PATH/modules/common/math/vec2d.cc \
  $$CODEX_PATH/modules/common/math/math_utils.cc \
  $$CODEX_PATH/modules/common/math/math_utils_extra.cpp \
  $$CODEX_PATH/modules/common/math/unit_converter.cpp \
  $$PREFIX/common/algorithm/line_segment_detector/line_segment_detector.cpp \
  $$SELF_PATH/run_distortion_measure.cpp \
  $$SELF_PATH/calibration_harp.cpp \
  $$SELF_PATH/config/runtime_config.cpp

HEADERS += \
  $$CODEX_PATH/modules/common/config/config_file_base.h \
  $$CODEX_PATH/modules/common/math/vec2d.h \
  $$CODEX_PATH/modules/common/math/math_utils.h \
  $$CODEX_PATH/modules/common/math/math_utils_extra.h \
  $$CODEX_PATH/modules/common/math/unit_converter.h \
  $$CODEX_PATH/modules/common_struct/basic_msgs/VectorPoint.h \
  $$PREFIX/common/base/point.h \
  $$PREFIX/common/algorithm/line_segment_detector/line_segment_detector.hpp \
  $$SELF_PATH/calibration_harp.hpp \
  $$SELF_PATH/config/runtime_config.h

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
