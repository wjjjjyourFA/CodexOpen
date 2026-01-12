MY_SYSTEM_INFO = 20.04.1
unix{
  MY_SYSTEM_INFO = $$system(uname -a | cut -d \~ -f 2 | cut -d \- -f 1)
  message("System Is Ubuntu" $${QT_ARCH} : $${MY_SYSTEM_INFO})
}
MY_ROS_INFO = 1

TEMPLATE = app
QT += core
CONFIG += console c++14
CONFIG -= app_bundle
CONFIG += object_parallel_to_source
TARGET = gnss_hc_wrapper_ros1

CODEX_PATH = $$PWD/../../../../../../CodexOpen
SELF_PATH = $$CODEX_PATH/modules/drivers/gnss/hc_wrapper

DESTDIR = $$CODEX_PATH/install/bin/modules/drivers/gnss

CONFIG(debug, debug|release) {
  OBJECTS_DIR = $$CODEX_PATH/build/Debug
  QMAKE_CXXFLAGS += -O1
}
CONFIG(release, debug|release) {
  OBJECTS_DIR = $$CODEX_PATH/build/Release
  QMAKE_CXXFLAGS += -O3
}

SOURCES += \
  $$CODEX_PATH/modules/common/math/vec2d.cc \
  $$CODEX_PATH/modules/common/math/math_utils.cc \
  $$CODEX_PATH/modules/common/math/math_utils_extra.cpp \
  $$CODEX_PATH/modules/common/math/unit_converter.cpp \
  $$SELF_PATH/common/*.cpp \
  $$SELF_PATH/main.cpp \
  $$SELF_PATH/config/*.cpp \
  $$SELF_PATH/rac_gnss_receiver.cpp \
  $$SELF_PATH/ros1_convert.cpp \

HEADERS += \
  $$CODEX_PATH/modules/common/environment_conf.h \
  $$CODEX_PATH/modules/common/math/vec2d.h \
  $$CODEX_PATH/modules/common/math/math_utils.h \
  $$CODEX_PATH/modules/common/math/math_utils_extra.h \
  $$CODEX_PATH/modules/common/math/unit_converter.h \
  $$CODEX_PATH/modules/common/math/kalman_filter_1d.h \
  $$CODEX_PATH/modules/common_struct/sensor_msgs/GnssData.h \
  $$SELF_PATH/common/*.h \
  $$SELF_PATH/config/*.h \
  $$SELF_PATH/rac_gnss_receiver.h \
  $$SELF_PATH/ros1_convert.h \

INCLUDEPATH += \
  $$CODEX_PATH \

INCLUDEPATH += \
  /usr/include/eigen3 \

contains(MY_ROS_INFO, 1){
INCLUDEPATH += \
  /opt/ros/melodic/include \
  /opt/ros/noetic/include \
  $$CODEX_PATH/ros1/devel/include \
}

## BASE
LIBS += \
  -L/usr/local/lib \
  -L/usr/lib/x86_64-linux-gnu \
  -L/usr/lib/aarch64-linux-gnu \
  -lgflags -lglog -lyaml-cpp

## ROS1
contains(MY_ROS_INFO, 1){
LIBS += \
  # -L/opt/ros/$(ROS_DISTRO)/lib \
  -L/opt/ros/melodic/lib \
  -L/opt/ros/noetic/lib \
  -lroscpp -lroslib -lrosconsole -lroscpp_serialization -lrostime \
  -lcv_bridge -limage_transport \
  -lxmlrpcpp -lrosconsole_log4cxx -lrosconsole_backend_interface \
  -lmessage_filters -lclass_loader -lrospack -lcpp_common \
  -lrosbag_storage \
}
