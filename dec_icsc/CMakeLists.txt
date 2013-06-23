cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

# Turn off shared libs and turn on static libs, just for this package
# set(ROS_BUILD_STATIC_LIBS true)
# set(ROS_BUILD_SHARED_LIBS false)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_add_library(icsc_over_usb
  src/icsc_over_usb.cpp
)

rosbuild_add_executable(test_icsc_send_led
  test/test_icsc_send_led.cpp
)
target_link_libraries(test_icsc_send_led icsc_over_usb)

rosbuild_add_executable(test_icsc_receive_led
  test/test_icsc_receive_led.cpp
)
target_link_libraries(test_icsc_receive_led icsc_over_usb)


rosbuild_add_executable(test_icsc_dec
  test/test_icsc_dec.cpp
)
target_link_libraries(test_icsc_dec icsc_over_usb)



#common commands for building c++ executables and libraries
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
