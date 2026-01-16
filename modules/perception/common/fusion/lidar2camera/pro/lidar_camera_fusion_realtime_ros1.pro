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
TARGET = lidar_camera_fusion_realtime_ros1

CODEX_PATH = $$PWD/../../../../../../../CodexOpen
PREFIX = $$CODEX_PATH/modules/perception
SELF_PATH = $$CODEX_PATH/modules/perception/common/fusion/lidar2camera

DESTDIR = $$CODEX_PATH/install/bin/modules/perception/common/fusion

CONFIG(debug, debug|release) {
  OBJECTS_DIR = $$CODEX_PATH/build/Debug
  QMAKE_CXXFLAGS += -O1
}
CONFIG(release, debug|release) {
  OBJECTS_DIR = $$CODEX_PATH/build/Release
  QMAKE_CXXFLAGS += -O3
}

CONFIG += openmp
QMAKE_CXXFLAGS += -fopenmp
QMAKE_LFLAGS += -fopenmp

SOURCES += \
  $$CODEX_PATH/cyber/binary.cc \
  $$CODEX_PATH/modules/common/math/math_utils_extra.cpp \
  $$PREFIX/common/base/camera.cc \
  $$PREFIX/common/base/distortion_model.cc \
  $$PREFIX/common/base/fisheye_model.cc \
  $$PREFIX/common/base/opencv_extra/cv_colors.cpp \
  $$PREFIX/common/base/opencv_extra/colors.cpp \
  $$PREFIX/common/config/utils.cpp \
  $$PREFIX/common/camera/common/undistortion_handler_legacy.cc \
  $$PREFIX/common/camera/common/undistortion_handler_cv.cc \
  $$PREFIX/common/camera/params/camera_params.cpp \
  $$PREFIX/common/lidar/convert/robosense.cpp \
  $$PREFIX/common/lidar/convert/velodyne.cpp \
  # $$PREFIX/tools/common/show_data_3d.cpp \
  $$PREFIX/tools/common/show_data_2d.cpp \
  $$SELF_PATH/lidar_camera_fusion.cpp \
  $$SELF_PATH/lidar_camera_fusion_realtime.cpp \
  $$SELF_PATH/config/runtime_config_realtime.cpp \
  $$SELF_PATH/ros1_convert.cpp \

HEADERS += \
  $$CODEX_PATH/cyber/binary.h \
  $$CODEX_PATH/cyber/common/environment_conf.h \
  $$CODEX_PATH/modules/common/math/math_utils_extra.h \
  $$PREFIX/common/base/camera.h \
  $$PREFIX/common/base/distortion_model.h \
  $$PREFIX/common/base/fisheye_model.h \
  $$PREFIX/common/base/opencv_extra/cv_colors.h \
  $$PREFIX/common/base/opencv_extra/colors.hpp \
  $$PREFIX/common/base/pcl_extra/pcl_eigen.h \
  $$PREFIX/common/config/utils.h \
  $$PREFIX/common/camera/common/undistortion_handler_legacy.h \
  $$PREFIX/common/camera/common/undistortion_handler_cv.h \
  $$PREFIX/common/camera/params/camera_params.h \
  $$PREFIX/common/lidar/convert/robosense.h \
  $$PREFIX/common/lidar/convert/velodyne.h \
  # $$PREFIX/tools/common/show_data_3d.h \
  $$PREFIX/tools/common/show_data_2d.h \
  $$PREFIX/tools/save_file/save_ply.h \
  $$SELF_PATH/lidar_camera_fusion.h \
  $$SELF_PATH/config/runtime_config_realtime.h \
  $$SELF_PATH/ros1_convert.h \

INCLUDEPATH += \
  $$CODEX_PATH \

INCLUDEPATH += \
  /usr/include/eigen3 \
  /usr/include/opencv4 \
  # /opt/ros \
  /opt/ros/$(ROS_DISTRO)/include \

contains(MY_ROS_INFO, 1){
INCLUDEPATH += \
  /opt/ros/melodic/include \
  /opt/ros/noetic/include \
  # $$CODEX_PATH/ros1/devel/include \
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
  -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_ml \
  -lopencv_imgcodecs -lopencv_calib3d \
  -lboost_filesystem -lboost_system -lboost_thread \
  -lpcl_common -lpcl_filters -lpcl_io -lpcl_io_ply \
  -lpcl_visualization \
  -lpthread -lglog -lyaml-cpp -lprotobuf

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
