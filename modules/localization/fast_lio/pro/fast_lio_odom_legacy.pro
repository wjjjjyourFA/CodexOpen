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
TARGET = odometry_fast_lio_legacy

CODEX_PATH = $$clean_path($$PWD/../../../..)
PREFIX = $$CODEX_PATH/modules/perception/common
SELF_PATH = $$CODEX_PATH/modules/localization/fast_lio
KDTREE_PATH = $$CODEX_PATH/modules/perception/common/algorithm/point_cloud_processing/ikd-Tree
SOPHUS_PATH = $$CODEX_PATH/third_party/Sophus
OTHER_PATH = $$CODEX_PATH/tools/data_loader
DATA_PROCESSOR_PATH = $$CODEX_PATH/tools/data_processor

DESTDIR = $$CODEX_PATH/install/bin/modules/localization/fast_lio

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
# add_definitions(-DMP_EN)
DEFINES += MP_EN
# add_definitions(-DMP_PROC_NUM=4)
DEFINES += MP_PROC_NUM=4

SOURCES += \
  $$CODEX_PATH/cyber/binary.cc \
  $$CODEX_PATH/cyber/common/file.cc \
  $$CODEX_PATH/modules/common/config/config_file_base.cpp \
  $$CODEX_PATH/modules/common/math/unit_converter.cpp \
  $$CODEX_PATH/modules/common/transform/geometry/rotation_conversions.cpp \
  $$PREFIX/base/camera.cc \
  $$PREFIX/base/distortion_model.cc \
  $$PREFIX/base/fisheye_model.cc \
  $$PREFIX/config/utils.cpp \
  $$PREFIX/config/sensor_extrinsics.cpp \
  $$PREFIX/config/vehicle_config.cpp \
  $$PREFIX/camera/common/undistortion_handler.cc \
  $$PREFIX/camera/params/*.cpp \
  $$DATA_PROCESSOR_PATH/config/sensor_config.cc \
  $$OTHER_PATH/data_loader.cpp \
  $$OTHER_PATH/group_convert.cpp \
  $$OTHER_PATH/config/runtime_config.cpp \
  $$OTHER_PATH/config/interface_config.cpp \
  $$SELF_PATH/config/runtime_config.cpp \
  $$SELF_PATH/src/imu_processing_legacy.cpp \
  $$SELF_PATH/ieskf/esekfom.cpp \
  $$SELF_PATH/lidar_odometry_legacy.cpp \
  $$SELF_PATH/run_lidar_odometry_legacy.cpp \
  $$KDTREE_PATH/ikd_Tree.cpp \
  $$SOPHUS_PATH/sophus/so3.cpp

HEADERS += \
  $$CODEX_PATH/cyber/binary.h \
  $$CODEX_PATH/cyber/common/file.h \
  $$CODEX_PATH/modules/common/config/config_file_base.h \
  $$CODEX_PATH/modules/common/math/unit_converter.h \
  $$CODEX_PATH/modules/common/transform/geometry/rotation_conversions.h \
  $$PREFIX/base/camera.h \
  $$PREFIX/base/distortion_model.h \
  $$PREFIX/base/fisheye_model.h \
  $$PREFIX/config/utils.h \
  $$PREFIX/config/sensor_extrinsics.h \
  $$PREFIX/config/vehicle_config.h \
  $$PREFIX/camera/common/undistortion_handler.h \
  $$PREFIX/camera/params/*.h \
  $$CODEX_PATH/modules/perception/tools/pcl/point_types.h \
  $$OTHER_PATH/data_loader.h \
  $$OTHER_PATH/data_container.h \
  $$OTHER_PATH/group_convert.h \
  $$OTHER_PATH/config/runtime_config.h \
  $$OTHER_PATH/config/interface_config.h \
  $$DATA_PROCESSOR_PATH/config/sensor_config.h \
  $$BRANCH_PATH/data_loader.h \
  $$BRANCH_PATH/group_convert.h \
  $$SELF_PATH/config/runtime_config.h \
  $$SELF_PATH/include/common_lib.h \
  $$SELF_PATH/include/so3_math.h \
  $$SELF_PATH/include/imu_processing_legacy.hpp \
  $$SELF_PATH/include/preprocess.h \
  $$SELF_PATH/ieskf/esekfom.hpp \
  $$SELF_PATH/ieskf/use-ikfom.hpp \
  $$SELF_PATH/lidar_odometry_legacy.h \
  $$SELF_PATH/utils.h \
  $$KDTREE_PATH/ikd_Tree.h \
  $$SOPHUS_PATH/sophus/so3.h

INCLUDEPATH += \
  $$CODEX_PATH \
  $$CODEX_PATH/modules/perception/common/algorithm/point_cloud_processing/ikd-Tree \

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