MY_SYSTEM_INFO = 20.04.1
unix{
  MY_SYSTEM_INFO = $$system(uname -a | cut -d \~ -f 2 | cut -d \- -f 1)
  message("System Is Ubuntu" $${QT_ARCH} : $${MY_SYSTEM_INFO})
}

TEMPLATE = app
CONFIG += console c++17
CONFIG -= app_bundle
CONFIG -= qt
CONFIG += object_parallel_to_source
TARGET = serial_rs232_test

CODEX_PATH = $$clean_path($$PWD/../../..)
SELF_PATH = $$CODEX_PATH/tools/capture_record/common

# DESTDIR += $$PWD/bin
DESTDIR = $$CODEX_PATH/install/bin/tools/capture_record

MOC_DIR     = $$OUT_PWD/moc
OBJECTS_DIR = $$OUT_PWD/obj
CONFIG(debug, debug|release) { QMAKE_CXXFLAGS += -O1 }
CONFIG(release, debug|release) { QMAKE_CXXFLAGS += -O3 }

SOURCES += \
  $$SELF_PATH/serial_rs232_test.cpp \
  $$SELF_PATH/serial_rs232.cpp \

HEADERS += \
  $$SELF_PATH/serial_rs232.h \

INCLUDEPATH += \
  $$CODEX_PATH \

## BASE
LIBS += \
  -L/usr/local/lib \
  # -lboost_filesystem -lboost_system -lboost_thread \
