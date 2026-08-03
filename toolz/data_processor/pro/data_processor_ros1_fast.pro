MY_SYSTEM_INFO = 20.04.1
unix{
  MY_SYSTEM_INFO = $$system(uname -a | cut -d \~ -f 2 | cut -d \- -f 1)
  message("System Is Ubuntu" $${QT_ARCH} : $${MY_SYSTEM_INFO})
}
MY_ROS_INFO = 1

TEMPLATE = app
CONFIG += console c++14
CONFIG -= app_bundle
CONFIG -= qt
CONFIG += object_parallel_to_source
TARGET = data_processor_ros1_fast

CODEX_PATH = $$PWD/../../../../CodexOpen
PREFIX = $$CODEX_PATH/modules/perception
SELF_PATH = $$CODEX_PATH/tools/data_processor
BRANCH_PATH = $$CODEX_PATH/toolz/data_processor

# DESTDIR += $$PWD/bin
DESTDIR = $$CODEX_PATH/install/bin/toolz/data_processor

CONFIG(debug, debug|release) {
  # OBJECTS_DIR = $$CODEX_PATH/build/Debug/$$TARGET
  OBJECTS_DIR = $$CODEX_PATH/build/Debug
  QMAKE_CXXFLAGS += -O1
}
CONFIG(release, debug|release) {
  # OBJECTS_DIR = $$CODEX_PATH/build/Release/$$TARGET
  OBJECTS_DIR = $$CODEX_PATH/build/Release
  QMAKE_CXXFLAGS += -O3
}

CONFIG += openmp
QMAKE_CXXFLAGS += -fopenmp
QMAKE_LFLAGS += -fopenmp
win32:msvc {
  QMAKE_CXXFLAGS += /MP
}

SOURCES += \
  $$CODEX_PATH/cyber/binary.cc \
  $$CODEX_PATH/cyber/common/file.cc \
  $$CODEX_PATH/modules/common/config/config_file_json.cpp \
  $$CODEX_PATH/tools/common/utils/rosbag_utils.cpp \
  $$PREFIX/common/base/camera.cc \
  $$PREFIX/common/base/distortion_model.cc \
  $$PREFIX/common/base/fisheye_model.cc \
  $$files($$PREFIX/tools/pcl/*.cpp) \
  $$PREFIX/tools/opencv/cv_colors.cpp \
  $$PREFIX/common/camera/common/undistortion_handler.cc \
  $$files($$PREFIX/common/config/*.cpp) \
  $$files($$PREFIX/common/camera/params/*.cpp) \
  $$files($$PREFIX/common/lidar/convert/*.cpp) \
  $$SELF_PATH/config/sensor_config.cc \
  $$BRANCH_PATH/run_data_processor.cpp \
  $$BRANCH_PATH/data_processor.cpp \
  $$BRANCH_PATH/ros1_convert_fast.cpp \
  $$BRANCH_PATH/config/runtime_config.cpp \
  $$BRANCH_PATH/config/interface_config.cpp \

SOURCES -= \
  $$PREFIX/tools/pcl/pcl_viewer_test.cpp \
  $$SELF_PATH/config/runtime_config_legacy.cpp \

HEADERS += \
  $$CODEX_PATH/cyber/binary.h \
  $$CODEX_PATH/cyber/base/bounded_queue.h \
  $$CODEX_PATH/cyber/base/thread_pool.h \
  $$CODEX_PATH/cyber/common/file.h \
  $$CODEX_PATH/cyber/common/log.h \
  $$CODEX_PATH/modules/common/environment_conf.h \
  $$CODEX_PATH/modules/common/config/config_file_json.h \
  $$CODEX_PATH/tools/common/utils/rosbag_utils.h \
  $$PREFIX/common/base/camera.h \
  $$PREFIX/common/base/distortion_model.h \
  $$PREFIX/common/base/fisheye_model.h \
  $$files($$PREFIX/tools/pcl/*.h) \
  $$PREFIX/tools/opencv/cv_colors.h \
  $$PREFIX/common/camera/common/undistortion_handler.h \
  $$files($$PREFIX/common/config/*.h) \
  $$files($$PREFIX/common/camera/params/*.h) \
  $$files($$PREFIX/common/lidar/convert/*.h) \
  $$files($$PREFIX/common/radar/convert/*.h) \
  $$SELF_PATH/config/sensor_config.h \
  $$SELF_PATH/async_writer.h \
  $$SELF_PATH/block_queue.h \
  $$BRANCH_PATH/data_processor.h \
  $$BRANCH_PATH/ros1_convert_fast.h \
  $$BRANCH_PATH/config/runtime_config.h \
  $$BRANCH_PATH/config/interface_config.h \

HEADERS -= \
  $$SELF_PATH/config/runtime_config_legacy.h

INCLUDEPATH += \
  $$CODEX_PATH \
  $$SELF_PATH/ros1_message \
  $$SELF_PATH/ros1_message/version_1.0 \
  $$SELF_PATH/ros1_message/version_1.1 \

INCLUDEPATH += \
  /usr/include/eigen3 \
  /usr/include/opencv4 \
  /usr/include/nlohmann

contains(MY_ROS_INFO, 1){
INCLUDEPATH += \
  # /opt/ros \
  /opt/ros/$(ROS_DISTRO)/include \
  # /opt/ros/noetic/include \
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

## ROS1
contains(MY_ROS_INFO, 1){
LIBS += \
  # -L/opt/ros/$(ROS_DISTRO)/lib \
  -L/opt/ros/noetic/lib \
  -lroscpp -lroslib -lrosconsole -lroscpp_serialization -lrostime -lcv_bridge -limage_transport \
  -lxmlrpcpp -lrosconsole_log4cxx -lrosconsole_backend_interface \
  -lmessage_filters -lclass_loader -lrospack -lcpp_common \
  -lrosbag_storage \
}

