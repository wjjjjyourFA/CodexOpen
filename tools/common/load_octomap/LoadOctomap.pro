TEMPLATE = app
CONFIG += console c++14 c++11
CONFIG -= app_bundle
CONFIG -= qt
TARGET = LoadOctomap
#DESTDIR += ./../../../bin/
DESTDIR += ./bin/

QMAKE_CXXFLAGS += -DFH_DEBUG

CONFIG(debug, debug|release) {
    TARGET = octomap_Debug
    OBJECTS_DIR = ./objects
    LIBS += -L/usr/local/lib/opencv_debug
}
CONFIG(release, debug|release) {
    TARGET = octomap_Release
    OBJECTS_DIR = ./objects
    LIBS += -L/usr/local/lib/opencv_release
    QMAKE_CXXFLAGS_RELEASE += -O3 -D NDEBUG -D BOOST_DISABLE_ASSERTS -DEIGEN_NO_DEBUG #full optimization
}

HEADERS += \
    colors.hpp

SOURCES += \
    analysis_bt.cpp \
    colors.cpp

INCLUDEPATH += \
#    /home/tracer/SoftWare/octomap-1.9.0/octomap/include \
    /usr/include/pcl-1.10 \
    /usr/include/eigen3 \
    /usr/include/vtk-7.1
    /usr/include/boost \

LIBS += \
    -L/usr/local/lib \
#    /home/ricky/tools/octomap-1.9.0/octomap/lib/liboctomap.a \
#    /home/ricky/tools/octomap-1.9.0/octomap/lib/liboctomath.a \
#    /home/ricky/tools/octomap-1.9.0/dynamicEDT3D/lib/libdynamicedt3d.a \
    -L/usr/lib/x86_64-linux-gnu \
#    -L/home/tracer/SoftWare/octomap-1.9.0/lib \
    -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_ml -lopencv_calib3d -lopencv_video -lopencv_flann -lopencv_features2d \
    -lpcl_common -lpcl_features -lpcl_filters -lpcl_io -lpcl_io_ply -lpcl_kdtree -lpcl_keypoints \
    -lpcl_octree -lpcl_outofcore -lpcl_people -lpcl_recognition -lpcl_registration -lpcl_sample_consensus -lpcl_search \
    -lpcl_segmentation -lpcl_surface -lpcl_tracking -lpcl_visualization \
    -lboost_system -lboost_filesystem -lboost_thread \
    -lvtkCommonCore-7.1 -lvtkFiltersCore-7.1 -lvtksys-7.1 -lvtkRenderingCore-7.1 -lvtkFiltersHybrid-7.1 -lvtkCommonDataModel-7.1 -lvtkCommonMath-7.1 \
    -ldynamicedt3d -loctomap -loctomath
