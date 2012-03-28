cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
rosbuild_add_library(subViz src/objLoader.cpp)
rosbuild_add_library(motorFunc src/motor.cpp)
rosbuild_add_executable(subSim src/simulator.cpp)
rosbuild_add_executable(subViz src/viz.cpp)
rosbuild_add_executable(FloatMixer src/mixer.cpp)
rosbuild_add_executable(FloatRepeater src/repeater.cpp)
rosbuild_add_executable(simpleDepth src/simpleDepth.cpp)
rosbuild_add_executable(simpleHeading src/simpleHeading.cpp)
target_link_libraries(simpleDepth motorFunc)
target_link_libraries(subViz glut)
target_link_libraries(subViz GLU)
target_link_libraries(subViz GL)
target_link_libraries(subViz X11)
