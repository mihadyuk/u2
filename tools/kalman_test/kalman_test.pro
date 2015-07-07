TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt
CONFIG += c++11

SOURCES += main.cpp \
    navi6d_wrapper.cpp

include(deployment.pri)
qtcAddDeployment()

HEADERS += \
    navi6d_wrapper.hpp

