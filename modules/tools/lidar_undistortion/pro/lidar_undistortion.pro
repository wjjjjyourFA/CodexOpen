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
TARGET = lidar_undistortion

CODEX_PATH = $$PWD/../../../../../CodexOpen
PREFIX = $$CODEX_PATH/modules/perception/common
SELF_PATH = $$CODEX_PATH/modules/tools/lidar_undistortion
OTHER_PATH = $$CODEX_PATH/tools/data_loader
BRANCH_PATH = $$CODEX_PATH/toolz/data_loader
DATA_PROCESSOR_PATH = $$CODEX_PATH/tools/data_processor

DESTDIR = $$CODEX_PATH/install/bin/modules/tools/lidar_undistortion

CONFIG(debug, debug|release) {
  OBJECTS_DIR = $$CODEX_PATH/build/Debug
  QMAKE_CXXFLAGS += -O1
}
CONFIG(release, debug|release) {
  OBJECTS_DIR = $$CODEX_PATH/build/Release
  QMAKE_CXXFLAGS += -O3
}

# 启用 OpenMP 支持
# CONFIG += openmp

SOURCES += \
  $$CODEX_PATH/cyber/binary.cc \
  $$CODEX_PATH/cyber/common/file.cc \
  $$CODEX_PATH/modules/common/config/config_file_base.cpp \
  $$CODEX_PATH/modules/common/math/unit_converter.cpp \
  $$CODEX_PATH/modules/common/transform/geometry/rotation_conversions.cpp \
  $$PREFIX/base/camera.cc \
  $$PREFIX/base/distortion_model.cc \
  $$PREFIX/config/utils.cpp \
  $$PREFIX/config/sensor_extrinsics.cpp \
  $$PREFIX/camera/common/undistortion_handler.cc \
  $$PREFIX/camera/params/*.cpp \
  # $$PREFIX/lidar/common/motion_ompensator_legacy.cpp \
  $$PREFIX/lidar/common/motion_ompensator.cpp \
  $$PREFIX/fusion/lidar2camera/lidar_camera_fusion.cpp \
  $$CODEX_PATH/modules/perception/tools/opencv/cv_colors.cpp \
  $$files($$CODEX_PATH/modules/perception/tools/pcl/*.cpp) \
  $$DATA_PROCESSOR_PATH/config/sensor_config.cc \
  $$OTHER_PATH/data_loader.cpp \
  $$OTHER_PATH/group_convert.cpp \
  $$OTHER_PATH/config/runtime_config_offline.cpp \
  $$BRANCH_PATH/data_loader.cpp \
  $$BRANCH_PATH/group_convert.cpp \
  $$SELF_PATH/config/runtime_config.cpp \
  $$SELF_PATH/run_lidar_undistortion.cpp \

SOURCES -= \
  $$CODEX_PATH/modules/perception/tools/pcl/pcl_viewer_test.cpp

HEADERS += \
  $$CODEX_PATH/cyber/binary.h \
  $$CODEX_PATH/cyber/common/file.h \
  $$CODEX_PATH/modules/common/config/config_file_base.h \
  $$CODEX_PATH/modules/common/math/unit_converter.h \
  $$CODEX_PATH/modules/common/transform/geometry/rotation_conversions.h \
  $$PREFIX/base/camera.h \
  $$PREFIX/base/distortion_model.h \
  $$PREFIX/config/utils.h \
  $$PREFIX/config/sensor_extrinsics.h \
  $$PREFIX/camera/common/undistortion_handler.h \
  $$PREFIX/camera/params/*.h \
  $$PREFIX/lidar/common/motion_ompensator.h \
  $$PREFIX/fusion/lidar2camera/lidar_camera_fusion.h \
  $$CODEX_PATH/modules/perception/tools/opencv/cv_colors.h \
  $$files($$CODEX_PATH/modules/perception/tools/pcl/*.h) \
  $$DATA_PROCESSOR_PATH/config/sensor_config.h \
  $$OTHER_PATH/data_loader.h \
  $$OTHER_PATH/data_container.h \
  $$OTHER_PATH/group_convert.h \
  $$OTHER_PATH/config/runtime_config_offline.h \
  $$BRANCH_PATH/data_loader.h \
  $$BRANCH_PATH/group_convert.h \
  $$SELF_PATH/config/runtime_config.h \

INCLUDEPATH += \
  $$CODEX_PATH \

INCLUDEPATH += \
  /usr/include/eigen3 \
  /usr/include/opencv4 \
  # /usr/local/include/opencv4 \
  /usr/local/include \

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

LIBS += \
  -L/usr/local/lib \
  -L/usr/lib/x86_64-linux-gnu \
  -L/usr/lib/aarch64-linux-gnu \
  -lopencv_core -lopencv_highgui -lopencv_imgproc \
  -lopencv_imgcodecs -lopencv_calib3d \
  -lboost_filesystem -lboost_system -lboost_thread \
  -lpcl_common -lpcl_filters -lpcl_io -lpcl_io_ply \
  -lpcl_visualization \
  -lgomp -lglog -lprotobuf


