PROJECT(USUbConsole)
FIND_PACKAGE(Qt4 REQUIRED)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

cmake_minimum_required(VERSION 2.4.6)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

rosbuild_init()

rosbuild_genmsg()

SET(subConsole_SOURCES src/main.cpp src/SubConsole.cpp src/attitude_indicator.cpp src/ClickableLabel.cpp src/filter.cpp src/qcustomplot.cpp)
SET(subConsole_FORMS src/SubConsole.ui)
SET(subConsole_HEADERS src/SubConsole.hpp src/attitude_indicator.h src/ClickableLabel.hpp src/qcustomplot.h)

QT4_WRAP_CPP(subConsole_HEADERS_MOC ${subConsole_HEADERS})
QT4_WRAP_UI(subConsole_FORMS_HEADERS ${subConsole_FORMS})

INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})

rosbuild_add_executable(USUbConsole ${subConsole_SOURCES} ${subConsole_HEADERS_MOC} ${subConsole_FORMS_HEADERS})
target_link_libraries(USUbConsole joystick++)
target_link_libraries(USUbConsole qwt)
target_link_libraries(USUbConsole ${QT_LIBRARIES})

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})
