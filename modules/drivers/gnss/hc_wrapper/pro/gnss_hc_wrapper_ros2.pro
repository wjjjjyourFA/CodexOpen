MY_SYSTEM_INFO = 20.04.1
unix{
  MY_SYSTEM_INFO = $$system(uname -a | cut -d \~ -f 2 | cut -d \- -f 1)
  message("System Is Ubuntu" $${QT_ARCH} : $${MY_SYSTEM_INFO})
}
MY_ROS_INFO = 2

TEMPLATE = app
QT += core
CONFIG += console c++14
CONFIG -= app_bundle
CONFIG += object_parallel_to_source
TARGET = gnss_hc_wrapper_ros2

CODEX_PATH = $$PWD/../../../../../../CodexOpen
SELF_PATH = $$CODEX_PATH/modules/drivers/gnss/gnss_hc_wrapper

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
  $$CODEX_PATH/modules/common/math/math_utils_extra.cpp \
  $$CODEX_PATH/modules/common/math/unit_converter.cpp \
  $$CODEX_PATH/modules/common/math/kalman_filter_1d.cpp \
  $$SELF_PATH/common/*.cpp \
  $$SELF_PATH/main.cpp \
  $$SELF_PATH/config/*.cpp \
  $$SELF_PATH/rac_gnss_receiver.cpp \
  $$SELF_PATH/ros2_convert.cpp \

HEADERS += \
  $$CODEX_PATH/modules/common/environment_conf.h \
  $$CODEX_PATH/modules/common/math/math_utils_extra.h \
  $$CODEX_PATH/modules/common/math/unit_converter.h \
  $$CODEX_PATH/modules/common/math/kalman_filter_1d.h \
  $$CODEX_PATH/modules/common_struct/sensor_msgs/GnssData.h \
  $$SELF_PATH/common/*.h \
  $$SELF_PATH/config/*.h \
  $$SELF_PATH/rac_gnss_receiver.h \
  $$SELF_PATH/ros2_convert.h \

INCLUDEPATH += \
  $$CODEX_PATH \

contains(MY_ROS_INFO, 2){
INCLUDEPATH += \
  /opt/ros/foxy/include \
  /opt/ros/humble/include \
  # $$PWD/../../ros2/install \
  $$CODEX_PATH/ros2/install \

INCLUDEPATH += \
  $$CODEX_PATH/ros2/install/basic_msgs/include \
  $$CODEX_PATH/ros2/install/custom_sensor_msgs/include \
  $$CODEX_PATH/ros2/install/monitor_msgs/include \
}

## BASE
LIBS += \
  -L/usr/local/lib \
  -L/usr/lib/x86_64-linux-gnu \
  -L/usr/lib/aarch64-linux-gnu \
  -lgflags -lglog -lyaml-cpp

## ROS2
contains(MY_ROS_INFO, 2){
LIBS += \
  # -L/opt/ros/$(ROS_DISTRO)/lib \
  -L/opt/ros/foxy/lib \
  -L/opt/ros/humble/lib \
  -lrclcpp -lrcutils -lrcl -lrmw -lyaml -lbuiltin_interfaces__rosidl_generator_c \
  -lcv_bridge -limage_transport \
  -lrmw_implementation -lrcpputils -llibstatistics_collector -ltracetools -lrcl_logging_spdlog \
  -lrcl_yaml_param_parser -lrosidl_runtime_c -lrosidl_typesupport_c -lrosidl_typesupport_cpp \
  -lament_index_cpp \

LIBS += \
  -L$$CODEX_PATH/ros2/install/basic_msgs/lib \
  -L$$CODEX_PATH/ros2/install/custom_sensor_msgs/lib \
  -L$$CODEX_PATH/ros2/install/monitor_msgs/lib \
  -lbasic_msgs__rosidl_typesupport_cpp \
  -lcustom_sensor_msgs__rosidl_typesupport_cpp \
  -lmonitor_msgs__rosidl_typesupport_cpp \
}
