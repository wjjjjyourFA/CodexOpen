TEMPLATE = app
CONFIG += console c++14
CONFIG -= app_bundle
CONFIG -= qt
TARGET = frame2d_transform_test

CODEX_PATH = $$clean_path($$PWD/../../../../..)
SELF_PATH = $$CODEX_PATH/modules/localization/common/transform

DESTDIR = $$CODEX_PATH/install/bin/modules/localization/common

SOURCES += \
  $$SELF_PATH/frame2d_transform.cpp \
  $$SELF_PATH/frame2d_transform_test.cpp

HEADERS += \
  $$SELF_PATH/frame2d_transform.hpp

INCLUDEPATH += $$CODEX_PATH
