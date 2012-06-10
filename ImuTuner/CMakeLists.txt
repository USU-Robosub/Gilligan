PROJECT(ImuTuner)
FIND_PACKAGE(Qt4 REQUIRED)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

cmake_minimum_required(VERSION 2.4.6)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

rosbuild_init()

set(QT_ROOT $ENV{QT_ROOT})
set(QT_QMAKE_EXECUTABLE ${QT_ROOT}/bin/qmake)
set(QT_MOC_EXECUTABLE ${QT_ROOT}/bin/moc)
set(QT_RCC_EXECUTABLE ${QT_ROOT}/bin/rcc)
set(QT_UIC_EXECUTABLE ${QT_ROOT}/bin/uic)
set(QT_INCLUDE_DIR ${QT_ROOT}/include)
set(QT_LIBRARY_DIR ${QT_ROOT}/lib)
set(QT_QTCORE_LIBRARY ${QT_ROOT}/lib)

SET(imuTuner_SOURCES src/ImuTunerMain.cpp src/ImuTuner.cpp)
SET(imuTuner_FORMS src/ImuTuner.ui)
SET(imuTuner_HEADERS src/ImuTuner.hpp)

QT4_WRAP_CPP(imuTuner_HEADERS_MOC ${imuTuner_HEADERS})
QT4_WRAP_UI(imuTuner_FORMS_HEADERS ${imuTuner_FORMS})

INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})

rosbuild_add_executable(ImuTuner ${imuTuner_SOURCES} ${imuTuner_HEADERS_MOC} ${imuTuner_FORMS_HEADERS})
target_link_libraries(ImuTuner qwt)
target_link_libraries(ImuTuner ${QT_LIBRARIES})

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})
