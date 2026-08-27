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
TARGET = camera_image_to_ros1_tztek

CODEX_PATH = $$PWD/../../../../../CodexOpen
SELF_PATH = $$CODEX_PATH/modules/drivers/camera_tztek
OTHER_PATH = $$CODEX_PATH/third_party/tztek

DESTDIR = $$CODEX_PATH/install/bin/modules/drivers/camera

CONFIG(debug, debug|release) {
  OBJECTS_DIR = $$CODEX_PATH/build/Debug
  QMAKE_CXXFLAGS += -O1
}
CONFIG(release, debug|release) {
  OBJECTS_DIR = $$CODEX_PATH/build/Release
  QMAKE_CXXFLAGS += -O3
}

SOURCES += \
  $$CODEX_PATH/cyber/binary.cc \
  # $$CODEX_PATH/cyber/state.cc \
  $$CODEX_PATH/cyber/common/file.cc \
  $$CODEX_PATH/modules/common/config/config_file_base.cpp \
  $$CODEX_PATH/modules/drivers/camera/config/*.cpp \
  # $$CODEX_PATH//modules/drivers/camera/proto/config.pb.cc \
  $$SELF_PATH/driver_wrapper.cpp \
  $$SELF_PATH/ros1_convert.cpp \
  $$OTHER_PATH/source/*.cpp \
  $$SELF_PATH/tztek_camera_demo.cpp \

HEADERS += \
  $$CODEX_PATH/cyber/binary.h \
  # $$CODEX_PATH/cyber/state.h \
  $$CODEX_PATH/cyber/common/log.h \
  $$CODEX_PATH/cyber/common/file.h \
  $$CODEX_PATH/modules/common/environment_conf.h \
  $$CODEX_PATH/modules/common/config/config_file_base.h \
  $$CODEX_PATH/modules/drivers/camera/config/*.h \
  # $$CODEX_PATH/modules/drivers/camera/proto/config.pb.h \
  $$SELF_PATH/driver_wrapper.h \
  $$SELF_PATH/ros1_convert.h \
  $$OTHER_PATH/include/*.h \
  $$OTHER_PATH/include/*.hpp \

INCLUDEPATH += \
  $$CODEX_PATH \
  $$CODEX_PATH/install/include \

LIBS += \
  $$CODEX_PATH/install/lib/modules/drivers/camera/proto/libcamera_proto.a \

# jp5
LIBS += \
  $$CODEX_PATH/third_party/tztek/lib/codec_so/jp5/libjpegenc.so \
  $$CODEX_PATH/third_party/tztek/lib/codec_so/jp5/libvideoenc.so \

INCLUDEPATH += \
  /usr/include/eigen3 \
  /usr/include/opencv4 \
  # /opt/ros \
  /opt/ros/$(ROS_DISTRO)/include \
  /usr/include/libyuv

contains(MY_ROS_INFO, 1){
INCLUDEPATH += \
  /opt/ros/melodic/include \
  /opt/ros/noetic/include \
  # $$PWD/../../ros1/devel/include \
  $$CODEX_PATH/ros1/devel/include \
}

## BASE
LIBS += \
  -L/usr/local/lib \
  -L/usr/lib/x86_64-linux-gnu \
  -L/usr/lib/aarch64-linux-gnu \
  -L/usr/lib/aarch64-linux-gnu/tegra \
  -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_ml -lopencv_calib3d \
  -lopencv_video -lopencv_flann -lopencv_features2d -lopencv_imgcodecs \
  -lboost_filesystem -lboost_system -lboost_thread \
  -lavcodec -lavutil -lavformat -lswscale \
  -lpthread -lglog -lyaml-cpp -lprotobuf \
  -lyuv -lturbojpeg \
  -lnvbufsurface \

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
