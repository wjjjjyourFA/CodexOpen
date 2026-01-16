MY_SYSTEM_INFO = 20.04.1
unix{
  MY_SYSTEM_INFO = $$system(uname -a | cut -d \~ -f 2 | cut -d \- -f 1)
  message("System Is Ubuntu" $${QT_ARCH} : $${MY_SYSTEM_INFO})
}
MY_ROS_INFO = 2

TEMPLATE = app
CONFIG += console c++14
CONFIG -= app_bundle
CONFIG -= qt
CONFIG += object_parallel_to_source
TARGET = data_loader_ros2

CODEX_PATH = $$PWD/../../../../CodexOpen
PREFIX = $$CODEX_PATH/modules/perception
SELF_PATH = $$CODEX_PATH/tools/data_loader
DATA_PROCESSOR_PATH = $$CODEX_PATH/tools/data_processor

DESTDIR = $$CODEX_PATH/install/bin/tools/data_loader

CONFIG(debug, debug|release) {
  OBJECTS_DIR = $$CODEX_PATH/build/Debug
  QMAKE_CXXFLAGS += -O1
}
CONFIG(release, debug|release) {
  OBJECTS_DIR = $$CODEX_PATH/build/Release
  QMAKE_CXXFLAGS += -O3
}

QMAKE_CXXFLAGS += -fopenmp
QMAKE_LFLAGS += -fopenmp

SOURCES += \
  $$CODEX_PATH/cyber/binary.cc \
  $$CODEX_PATH/cyber/common/file.cc \
  $$CODEX_PATH/modules/common/config/config_file_base.cpp \
  $$PREFIX/common/base/camera.cc \
  $$PREFIX/common/base/distortion_model.cc \
  $$PREFIX/common/camera/common/undistortion_handler_legacy.cc \
  $$files($$PREFIX/common/config/*.cpp) \
  $$files($$PREFIX/common/camera/params/*.cpp) \
  $$files($$PREFIX/common/lidar/convert/*.cpp) \
  $$DATA_PROCESSOR_PATH/config/sensor_config.cc \
  $$SELF_PATH/run_data_loader_realtime.cpp \
  $$SELF_PATH/data_loader.cpp \
  $$SELF_PATH/data_loader_realtime.cpp \
  $$SELF_PATH/config/*.cpp \
  $$SELF_PATH/ros2_convert.cpp

HEADERS += \
  $$CODEX_PATH/cyber/binary.h \
  $$CODEX_PATH/cyber/common/log.h \
  $$CODEX_PATH/cyber/common/file.h \
  $$CODEX_PATH/cyber/common/environment_conf.h \
  $$CODEX_PATH/modules/common/config/config_file_base.h \
  $$PREFIX/common/base/camera.h \
  $$PREFIX/common/base/distortion_model.h \
  $$PREFIX/common/camera/common/undistortion_handler_legacy.h \
  $$files($$PREFIX/common/config/*.h) \
  $$files($$PREFIX/common/camera/params/*.h) \
  $$files($$PREFIX/common/lidar/convert/*.h) \
  $$files($$PREFIX/common/radar/convert/*.h) \
  $$DATA_PROCESSOR_PATH/config/sensor_config.h \
  $$SELF_PATH/data_loader.h \
  $$SELF_PATH/data_loader_realtime.h \
  $$SELF_PATH/data_container.h \
  $$SELF_PATH/data_container_ros2.h \
  $$SELF_PATH/config/*.h \
  $$SELF_PATH/ros2_convert.h

INCLUDEPATH += \
  $$CODEX_PATH \
  $$DATA_PROCESSOR_PATH/ros2_message \
  $$DATA_PROCESSOR_PATH/ros2_message/version_1.0 \

INCLUDEPATH += \
  /usr/include/eigen3 \
  /usr/include/opencv4 \
  # /opt/ros \
  /opt/ros/$(ROS_DISTRO)/include \

contains(MY_ROS_INFO, 2){
INCLUDEPATH += \
  /opt/ros/foxy/include \
  /opt/ros/humble/include \
  # $$PWD/../../ros1/devel/include \
}

# contains(MY_SYSTEM_INFO, 20.04){
INCLUDEPATH += \
  /usr/include/pcl-1.10 \
  /usr/include/vtk-7.1
LIBS += \
  -lvtkCommonCore-7.1 -lvtkCommonDataModel-7.1 -lvtkCommonMath-7.1 -lvtkFiltersCore-7.1 -lvtksys-7.1 -lvtkRenderingCore-7.1 -lvtkFiltersHybrid-7.1
# }
contains(MY_SYSTEM_INFO, 18.04){
INCLUDEPATH += \
  /usr/include/pcl-1.8 \
  /usr/include/vtk-6.3
LIBS += \
  -lvtkCommonCore-6.3 -lvtkCommonDataModel-6.3 -lvtkCommonMath-6.3 -lvtkFiltersCore-6.3 -lvtksys-6.3 -lvtkRenderingCore-6.3 -lvtkFiltersHybrid-6.3
}

## BASE
LIBS += \
  -L/usr/local/lib \
  -L/usr/lib/x86_64-linux-gnu \
  -L/usr/lib/aarch64-linux-gnu \
  -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_ml -lopencv_calib3d \
  -lopencv_video -lopencv_flann -lopencv_features2d -lopencv_imgcodecs \
  -lboost_filesystem -lboost_system -lboost_thread \
  -lpcl_common -lpcl_features -lpcl_filters -lpcl_io -lpcl_io_ply -lpcl_kdtree -lpcl_keypoints \
  -lpcl_octree -lpcl_outofcore -lpcl_people -lpcl_recognition -lpcl_registration -lpcl_sample_consensus -lpcl_search \
  -lpcl_segmentation -lpcl_surface -lpcl_tracking -lpcl_visualization \
  -lpthread -lglog -lprotobuf

## ROS2
contains(MY_ROS_INFO, 2){
LIBS += \
  # -L/opt/ros/$(ROS_DISTRO)/lib \
  -L/opt/ros/foxy/lib \
  -L/opt/ros/humble/lib \
  -lrclcpp -lrcutils -lrcl -lrmw -lyaml -lbuiltin_interfaces__rosidl_generator_c \
  -lcv_bridge -limage_transport -lcompressed_image_transport \
  -lrmw_implementation -lrcpputils -llibstatistics_collector -ltracetools -lrcl_logging_spdlog \
  -lrcl_yaml_param_parser -lrosidl_runtime_c -lrosidl_typesupport_c -lrosidl_typesupport_cpp \
  -lament_index_cpp -lrosbag2_cpp -lrosbag2_storage \

LIBS += \
  -L$$CODEX_PATH/ros2/install/rslidar_msg/lib \
  -L$$CODEX_PATH/ros2/install/sensor/lib \
  -L$$CODEX_PATH/ros2/install/self_state/lib \
  -L$$CODEX_PATH/ros2/install/ars548_interface/lib \
  -lsensor_msgs__rosidl_typesupport_cpp \
  -lstatistics_msgs__rosidl_typesupport_cpp \
  -lrslidar_msg__rosidl_typesupport_cpp \
  -lsensor__rosidl_typesupport_cpp \
  -lself_state__rosidl_typesupport_cpp \
  -lars548_interface__rosidl_typesupport_cpp \
}

