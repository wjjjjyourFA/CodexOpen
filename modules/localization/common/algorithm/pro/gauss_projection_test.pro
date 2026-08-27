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
TARGET = gauss_projection_test

CODEX_PATH = $$clean_path($$PWD/../../../../..)
SELF_PATH = $$CODEX_PATH/modules/localization/common/algorithm

DESTDIR = $$CODEX_PATH/install/bin/modules/localization/common

CONFIG(debug, debug|release) {
  OBJECTS_DIR = $$CODEX_PATH/build/Debug
  QMAKE_CXXFLAGS += -O1
}
CONFIG(release, debug|release) {
  OBJECTS_DIR = $$CODEX_PATH/build/Release
  QMAKE_CXXFLAGS += -O3
}

SOURCES += \
  $$SELF_PATH/gauss_projection.cpp \
  $$SELF_PATH/gauss_projection_test.cpp \

HEADERS += \
  $$SELF_PATH/gauss_projection.h \

INCLUDEPATH += \
  $$CODEX_PATH \

INCLUDEPATH += \
  /usr/include/eigen3 \
  /usr/local/include \

## BASE
LIBS += \
  -L/usr/local/lib \
  -L/usr/lib/x86_64-linux-gnu \
  -L/usr/lib/aarch64-linux-gnu \