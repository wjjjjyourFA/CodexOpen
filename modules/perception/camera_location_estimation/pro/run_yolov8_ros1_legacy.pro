MY_SYSTEM_INFO = 20.04.1
unix{
  MY_SYSTEM_INFO = $$system(uname -a | cut -d \~ -f 2 | cut -d \- -f 1)
  message("System Is Ubuntu" $${QT_ARCH} : $${MY_SYSTEM_INFO})
}
MY_ROS_INFO = 1
IMAGE_DETECTOR = 1
IMAGE_BYTETRACK = 1

TEMPLATE = app
CONFIG += console c++14
CONFIG -= app_bundle
CONFIG -= qt
CONFIG += object_parallel_to_source
TARGET = image_locator_yolov8_ros1_legacy

CODEX_PATH = $$PWD/../../../../../CodexOpen
PREFIX = $$CODEX_PATH/modules/perception
SELF_PATH = $$CODEX_PATH/modules/perception/camera_location_estimation
DETECTOR_PATH = $$CODEX_PATH/modules/perception/camera_detection_single_stage

DESTDIR = $$CODEX_PATH/install/bin/modules/perception/camera_location_estimation

CONFIG(debug, debug|release) {
  OBJECTS_DIR = $$CODEX_PATH/build/Debug
  QMAKE_CXXFLAGS += -O1
}
CONFIG(release, debug|release) {
  OBJECTS_DIR = $$CODEX_PATH/build/Release
  QMAKE_CXXFLAGS += -O3
}

# 启用 OpenMP 支持
CONFIG += openmp
QMAKE_CXXFLAGS += -fopenmp
QMAKE_LFLAGS += -fopenmp

SOURCES += \
  $$CODEX_PATH/cyber/binary.cc \
  $$CODEX_PATH/cyber/common/file.cc \
  $$CODEX_PATH/modules/common/config/config_file_base.cpp \
  $$CODEX_PATH/modules/common/math/unit_converter.cpp \
  $$PREFIX/common/base/object.cc \
  $$PREFIX/common/base/camera.cc \
  $$PREFIX/common/base/distortion_model.cc \
  $$PREFIX/common/base/fisheye_model.cc \
  $$PREFIX/common/base/segment.cc \
  $$PREFIX/common/config/utils.cpp \
  $$PREFIX/common/config/sensor_extrinsics.cpp \
  $$PREFIX/common/camera/common/undistortion_handler.cc \
  $$files($$PREFIX/common/camera/params/*.cpp) \
  $$PREFIX/common/lidar/convert/robosense.cpp \
  $$PREFIX/common/lidar/convert/rs_sort_map.cpp \
  $$PREFIX/common/algorithm/image_processing/util/utils.cpp \
  $$PREFIX/common/algorithm/point_cloud_processing/cluster_postprocess.cpp \
  $$PREFIX/common/lidar/cluster/object_cluster.cpp \
  $$PREFIX/common/algorithm/point_cloud_processing/kdtree/kdtree.cpp \
  $$PREFIX/common/fusion/lidar2camera/lidar_camera_fusion.cpp \
  $$PREFIX/tools/opencv/colors.cpp \
  $$PREFIX/tools/opencv/cv_colors.cpp \
  # $$PREFIX/tools/common/show_data_3d.cpp \
  $$PREFIX/tools/common/show_data_2d.cpp \
  $$DETECTOR_PATH/detector/yolo_obstacle_detector.cpp \
  $$SELF_PATH/camera_location_estimation_legacy.cpp \
  $$SELF_PATH/config/runtime_config.cpp \
  $$SELF_PATH/config/interface_config.cpp \
  $$SELF_PATH/run_camera_location_estimation.cpp \
  $$SELF_PATH/ros1_convert.cpp \

HEADERS += \
  $$CODEX_PATH/cyber/binary.h \
  $$CODEX_PATH/cyber/common/file.h \
  $$CODEX_PATH/modules/common/environment_conf.h \
  $$CODEX_PATH/modules/common/config/config_file_base.h \
  $$CODEX_PATH/modules/common/math/unit_converter.h \
  $$PREFIX/common/base/box_extra.h \
  $$PREFIX/common/base/object.h \
  $$PREFIX/common/base/object_supplement.h \
  $$PREFIX/common/base/object_types.h \
  $$CODEX_PATH/modules/common/math/math_utils_extra.h \
  $$PREFIX/common/base/camera.h \
  $$PREFIX/common/base/distortion_model.h \
  $$PREFIX/common/base/fisheye_model.h \
  $$PREFIX/common/base/segment.h \
  $$PREFIX/common/config/utils.h \
  $$PREFIX/common/config/sensor_extrinsics.h \
  $$PREFIX/common/algorithm/point_cloud_processing/util/utils.h \
  $$PREFIX/common/camera/common/undistortion_handler.h \
  $$files($$PREFIX/common/camera/params/*.h) \
  $$PREFIX/common/lidar/convert/robosense.h \
  $$PREFIX/common/lidar/convert/rs_sort_map.h \
  $$PREFIX/common/algorithm/image_processing/util/utils.h \
  $$PREFIX/common/algorithm/point_cloud_processing/cluster_postprocess.h \
  $$PREFIX/common/lidar/cluster/object_cluster.h \
  $$PREFIX/common/algorithm/point_cloud_processing/kdtree/kdtree.h \
  $$PREFIX/tools/pcl/point_types.h \
  $$PREFIX/common/fusion/lidar2camera/lidar_camera_fusion.h \
  $$PREFIX/tools/opencv/colors.hpp \
  $$PREFIX/tools/opencv/cv_colors.h \
  # $$PREFIX/tools/common/show_data_3d.h \
  $$PREFIX/tools/common/show_data_2d.h \
  $$DETECTOR_PATH/detector/yolo_obstacle_detector.h \
  $$SELF_PATH/camera_location_estimation_legacy.h \
  $$SELF_PATH/config/runtime_config.h \
  $$SELF_PATH/config/interface_config.h \
  $$SELF_PATH/ros1_convert.h \

INCLUDEPATH += \
  $$CODEX_PATH \

INCLUDEPATH += \
  /usr/include/eigen3 \
  /usr/include/opencv4 \
  # /opt/ros \
  /opt/ros/$(ROS_DISTRO)/include \

contains(IMAGE_DETECTOR, 1){
SOURCES += \
  $$DETECTOR_PATH/detector/yolov8/yolov8_wrapper.cpp \
  $$DETECTOR_PATH/detector/yolov8/src/*.cpp \

HEADERS += \
  $$DETECTOR_PATH/detector/yolov8/yolov8_wrapper.h \
  $$DETECTOR_PATH/detector/yolov8/plugin/*.h \
  $$DETECTOR_PATH/detector/yolov8/include/*.h \

OTHER_FILES += \
  $$DETECTOR_PATH/detector/yolov8/plugin/yololayer.cu \
  $$DETECTOR_PATH/detector/yolov8/src/preprocess.cu \
  $$DETECTOR_PATH/detector/yolov8/src/postprocess.cu \

INCLUDEPATH += \
  /opt/TensorRT/include \
  $$DETECTOR_PATH/detector/yolov8 \
  $$DETECTOR_PATH/detector/yolov8/plugin \
  $$DETECTOR_PATH/detector/yolov8/include \
}

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
  -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_ml -lopencv_videoio \
  -lopencv_imgcodecs -lopencv_dnn -lopencv_calib3d \
  -lboost_filesystem -lboost_system -lboost_thread \
  -lpcl_common -lpcl_features -lpcl_filters -lpcl_io \
  -lpcl_visualization \
  -lpthread -lglog -lyaml-cpp -lprotobuf \

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

#### cuda ####
LIBS += \
  -L/usr/local/cuda/lib64 \
  -lcudart -lcublas -lcurand -lnvrtc \
  -lnvcaffe_parser -lnvinfer -lnvinfer_plugin -lnvparsers

contains(IMAGE_DETECTOR, 1){
# CUDA settings <-- may change depending on your system
CUDA_SOURCES += $$DETECTOR_PATH/detector/yolov8/src/preprocess.cu
CUDA_SOURCES += $$DETECTOR_PATH/detector/yolov8/src/postprocess.cu
CUDA_SOURCES += $$DETECTOR_PATH/detector/yolov8/plugin/yololayer.cu

CUDA_SDK = "/usr/local/cuda/"
CUDA_DIR = "/usr/local/cuda/"

# DO NOT EDIT BEYOND THIS UNLESS YOU KNOW WHAT YOU ARE DOING....
SYSTEM_NAME = ubuntu           # Depending on your system either 'Win32', 'x64', or 'Win64'
SYSTEM_TYPE = 64               # '32' or '64', depending on your system
# Type of CUDA architecture, for example 'compute_10', 'compute_11', 'sm_10' 'sm_50'
# CUDA_ARCH = sm_61 # 1080       
# CUDA_ARCH = sm_72 # xavier
# CUDA_ARCH = sm_75 # 2080ti
CUDA_ARCH = sm_86  # 3060
# CUDA_ARCH = sm_87 # orin
NVCC_OPTIONS = --use_fast_math

# include paths
INCLUDEPATH += $$CUDA_DIR/include

# library directories
QMAKE_LIBDIR += $$CUDA_DIR/lib64/

# Add the necessary libraries
CUDA_LIBS += -lcuda -lcudart -lcufft

# The following makes sure all path names (which often include spaces) are put between quotation marks
CUDA_INC = $$join(INCLUDEPATH,'" -I"','-I"','"')
NVCC_LIBS = $$join(CUDA_LIBS, '-l','-l','')
#LIBS += $$join(CUDA_LIBS,'.so ', '', '.so')
LIBS += $$CUDA_LIBS

# Configuration of the Cuda compiler
CONFIG(debug, debug|release) {
  # Debug mode
  cuda_d.input = CUDA_SOURCES
  cuda_d.output = $$CODEX_PATH/build/Debug/$$TARGET/${QMAKE_FILE_BASE}_cuda.o
  cuda_d.commands = $$CUDA_DIR/bin/nvcc -D_DEBUG $$NVCC_OPTIONS $$CUDA_INC $$NVCC_LIBS --machine $$SYSTEM_TYPE -arch=$$CUDA_ARCH -c -o ${QMAKE_FILE_OUT} ${QMAKE_FILE_NAME}
  cuda_d.dependency_type = TYPE_C
  QMAKE_EXTRA_COMPILERS += cuda_d
}
else {
  # Release mode
  cuda.input = CUDA_SOURCES
  cuda.output = $$CODEX_PATH/build/Release/$$TARGET/${QMAKE_FILE_BASE}_cuda.o
  cuda.commands = $$CUDA_DIR/bin/nvcc $$NVCC_OPTIONS $$CUDA_INC $$NVCC_LIBS --machine $$SYSTEM_TYPE -arch=$$CUDA_ARCH -c -o ${QMAKE_FILE_OUT} ${QMAKE_FILE_NAME}
  cuda.dependency_type = TYPE_C
  QMAKE_EXTRA_COMPILERS += cuda
}
}
