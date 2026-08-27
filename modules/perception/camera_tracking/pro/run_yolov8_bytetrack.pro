MY_SYSTEM_INFO = 20.04.1
unix{
  MY_SYSTEM_INFO = $$system(uname -a | cut -d \~ -f 2 | cut -d \- -f 1)
  message("System Is Ubuntu" $${QT_ARCH} : $${MY_SYSTEM_INFO})
}
IMAGE_DETECTOR = 1
IMAGE_BYTETRACK = 1

TEMPLATE = app
CONFIG += console c++14
CONFIG -= app_bundle
CONFIG -= qt
CONFIG += object_parallel_to_source
TARGET = image_tracking_yolov8_bytetrack

CODEX_PATH = $$clean_path($$PWD/../../../..)
PREFIX = $$CODEX_PATH/modules/perception
SELF_PATH = $$CODEX_PATH/modules/perception/camera_tracking
DETECTOR_PATH = $$CODEX_PATH/modules/perception/camera_detection_single_stage

DESTDIR = $$CODEX_PATH/install/bin/modules/perception/camera_tracking

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
  $$PREFIX/common/camera/common/undistortion_handler.cc \
  $$files($$PREFIX/common/config/*.cpp) \
  $$files($$PREFIX/common/camera/params/*.cpp) \
  $$CODEX_PATH/tools/data_loader/data_loader.cpp \
  $$CODEX_PATH/tools/data_loader/config/runtime_config.cpp \
  $$CODEX_PATH/tools/data_loader/config/interface_config.cpp \
  $$CODEX_PATH/tools/data_loader/group_convert.cpp \
  $$CODEX_PATH/tools/data_processor/config/sensor_config.cc \
  $$DETECTOR_PATH/detector/yolo_obstacle_detector.cpp \
  $$SELF_PATH/camera_tracking.cpp \
  $$SELF_PATH/config/runtime_config.cpp \
  $$SELF_PATH/run_yolo_track.cpp \

SOURCES -= \
  $$PREFIX/tools/pcl/pcl_viewer_test.cpp \
  $$SELF_PATH/config/runtime_config_legacy.cpp \
  $$PREFIX/common/config/config_core_test.cpp \
  $$PREFIX/common/camera/params/camera_params_core_test.cpp \
  $$PREFIX/common/lidar/convert/lidar_convert_core_test.cpp

HEADERS += \
  $$CODEX_PATH/cyber/binary.h \
  $$CODEX_PATH/cyber/common/file.h \
  $$CODEX_PATH/modules/common/config/config_file_base.h \
  $$CODEX_PATH/modules/common/math/unit_converter.h \
  $$PREFIX/common/base/box_extra.h \
  $$PREFIX/common/base/object.h \
  $$PREFIX/common/base/object_supplement.h \
  $$PREFIX/common/base/object_types.h \
  $$PREFIX/common/base/camera.h \
  $$PREFIX/common/base/distortion_model.h \
  $$PREFIX/common/base/fisheye_model.h \
  $$PREFIX/common/camera/common/undistortion_handler.h \
  $$files($$PREFIX/common/config/*.h) \
  $$files($$PREFIX/common/camera/params/*.h) \
  $$CODEX_PATH/tools/data_loader/data_loader.h \
  $$CODEX_PATH/tools/data_loader/config/runtime_config.h \
  $$CODEX_PATH/tools/data_loader/config/interface_config.h \
  $$CODEX_PATH/tools/data_loader/data_container.h \
  $$CODEX_PATH/tools/data_loader/group_convert.h \
  $$CODEX_PATH/tools/data_processor/config/sensor_config.h \
  $$DETECTOR_PATH/detector/yolo_obstacle_detector.h \
  $$SELF_PATH/camera_tracking.h \
  $$SELF_PATH/config/runtime_config.h \

INCLUDEPATH += \
  $$CODEX_PATH \

INCLUDEPATH += \
  /usr/include/eigen3 \
  /usr/include/opencv4 \
  # /usr/local/include/opencv4 \

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

contains(IMAGE_BYTETRACK, 1){
SOURCES += \
  $$SELF_PATH/tracking/bytetrack/bytetrack_wrapper.cpp \
  $$SELF_PATH/tracking/bytetrack/src/*.cpp \

HEADERS += \
  $$SELF_PATH/tracking/bytetrack/bytetrack_wrapper.h \
  $$SELF_PATH/tracking/bytetrack/include/*.h \
  $$SELF_PATH/tracking/bytetrack/include/*.hpp \

INCLUDEPATH += \
  /opt/TensorRT/include \
  $$SELF_PATH/tracking/bytetrack \
  $$SELF_PATH/tracking/bytetrack/include \
}

# contains(MY_SYSTEM_INFO, 20.04){
INCLUDEPATH += \
  /usr/include/pcl-1.10 \
  /usr/include/vtk-7.1
LIBS += \
  -lvtkCommonCore-7.1 -lvtkCommonDataModel-7.1 -lvtkCommonMath-7.1 -lvtkFiltersCore-7.1 -lvtksys-7.1 -lvtkRenderingCore-7.1 -lvtkFiltersHybrid-7.1
# }

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
  -lpthread -lX11 -lglog -lprotobuf \

# LIBS += `pkg-config opencv4 --libs --cflags` -lm

#### cuda ####
LIBS += \
  -L/usr/local/cuda/lib64 \
  -lcudart -lcublas -lcurand -lnvrtc \
  -lnvcaffe_parser -lnvinfer -lnvinfer_plugin -lnvparsers \
  -lnvonnxparser

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
