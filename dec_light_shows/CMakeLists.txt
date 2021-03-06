cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

rosbuild_add_library(${PROJECT_NAME}
  src/dec_sensor_butterworth_processor.cpp
  src/dec_sensor_decay_processor.cpp
  src/dec_sensor_rise_processor.cpp
  src/dec_light_processor.cpp
  src/dec_color_processor.cpp
  src/dec_open_loop_light_generator.cpp
  src/dec_light_show_ball_creator.cpp
  src/dec_light_show_plane_creator.cpp
  src/dec_light_show_recorder.cpp
  src/dec_light_show_player.cpp
  src/dec_idle.cpp
  src/dec_audio_processor.cpp
  src/dec_brightness_processor.cpp
  src/dec_test_light_generator.cpp
)
rosbuild_add_openmp_flags(${PROJECT_NAME})

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
