TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt
QMAKE_CXXFLAGS += -std=c++11

SOURCES += main.cpp \
    navi6d_wrapper.cpp \
    param_registry.cpp

include(deployment.pri)
qtcAddDeployment()

HEADERS += \
    navi6d_wrapper.hpp \
    param_registry.hpp

