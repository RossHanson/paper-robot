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
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/movements.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread signals)
#find_package(Boost, COMPONENTS foreach tokenizer REQUIRED)
#target_link_libraries(${PROJECT_NAME} ${Boost_FOREACH_LIBRARY} ${Boost_TOKENIZER_LIBRARY})
#rosbuild_link_boost(${PROJECT_NAME} foreach)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(prepare src/prep.cpp)
rosbuild_add_executable(simple_pick src/simplePick.cpp)
rosbuild_add_executable(shake src/move_head.cpp)
rosbuild_add_executable(move_left src/move_left_arm.cpp)
rosbuild_add_executable(test_force src/testForce.cpp)
rosbuild_add_executable(listen_server src/move_server.cpp)
rosbuild_add_executable(seg_table src/segment_table.cpp)