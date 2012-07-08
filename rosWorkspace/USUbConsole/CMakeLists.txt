PROJECT(USUbConsole)
FIND_PACKAGE(Qt4 REQUIRED)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

cmake_minimum_required(VERSION 2.4.6)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

rosbuild_init()

rosbuild_genmsg()

set(QT_ROOT $ENV{QT_ROOT})
set(QT_QMAKE_EXECUTABLE ${QT_ROOT}/bin/qmake)
set(QT_MOC_EXECUTABLE ${QT_ROOT}/bin/moc)
set(QT_RCC_EXECUTABLE ${QT_ROOT}/bin/rcc)
set(QT_UIC_EXECUTABLE ${QT_ROOT}/bin/uic)
set(QT_INCLUDE_DIR ${QT_ROOT}/include)
set(QT_LIBRARY_DIR ${QT_ROOT}/lib)
set(QT_QTCORE_LIBRARY ${QT_ROOT}/lib)

SET(subConsole_SOURCES src/main.cpp src/SubConsole.cpp src/attitude_indicator.cpp src/ClickableLabel.cpp)
SET(subConsole_FORMS src/SubConsole.ui)
SET(subConsole_HEADERS src/SubConsole.hpp src/attitude_indicator.h src/ClickableLabel.hpp)

QT4_WRAP_CPP(subConsole_HEADERS_MOC ${subConsole_HEADERS})
QT4_WRAP_UI(subConsole_FORMS_HEADERS ${subConsole_FORMS})

INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})

rosbuild_add_executable(USUbConsole ${subConsole_SOURCES} ${subConsole_HEADERS_MOC} ${subConsole_FORMS_HEADERS})
target_link_libraries(USUbConsole joystick++)
target_link_libraries(USUbConsole qwt)
target_link_libraries(USUbConsole ${QT_LIBRARIES})

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})
