# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/david/rosws/paper-robot/vision_test3

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/david/rosws/paper-robot/vision_test3/build

# Include any dependencies generated for this target.
include CMakeFiles/vision_streaming.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/vision_streaming.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/vision_streaming.dir/flags.make

CMakeFiles/vision_streaming.dir/src/vision_streaming.o: CMakeFiles/vision_streaming.dir/flags.make
CMakeFiles/vision_streaming.dir/src/vision_streaming.o: ../src/vision_streaming.cpp
CMakeFiles/vision_streaming.dir/src/vision_streaming.o: ../manifest.xml
CMakeFiles/vision_streaming.dir/src/vision_streaming.o: /opt/ros/fuerte/share/roslang/manifest.xml
CMakeFiles/vision_streaming.dir/src/vision_streaming.o: /opt/ros/fuerte/share/roscpp/manifest.xml
CMakeFiles/vision_streaming.dir/src/vision_streaming.o: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
CMakeFiles/vision_streaming.dir/src/vision_streaming.o: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
CMakeFiles/vision_streaming.dir/src/vision_streaming.o: /opt/ros/fuerte/stacks/vision_opencv/opencv2/manifest.xml
CMakeFiles/vision_streaming.dir/src/vision_streaming.o: /opt/ros/fuerte/stacks/vision_opencv/cv_bridge/manifest.xml
CMakeFiles/vision_streaming.dir/src/vision_streaming.o: /opt/ros/fuerte/share/ros/core/rosbuild/manifest.xml
CMakeFiles/vision_streaming.dir/src/vision_streaming.o: /opt/ros/fuerte/share/roslib/manifest.xml
CMakeFiles/vision_streaming.dir/src/vision_streaming.o: /opt/ros/fuerte/share/rosconsole/manifest.xml
CMakeFiles/vision_streaming.dir/src/vision_streaming.o: /opt/ros/fuerte/stacks/pluginlib/manifest.xml
CMakeFiles/vision_streaming.dir/src/vision_streaming.o: /opt/ros/fuerte/share/message_filters/manifest.xml
CMakeFiles/vision_streaming.dir/src/vision_streaming.o: /opt/ros/fuerte/stacks/image_common/image_transport/manifest.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /home/david/rosws/paper-robot/vision_test3/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/vision_streaming.dir/src/vision_streaming.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/vision_streaming.dir/src/vision_streaming.o -c /home/david/rosws/paper-robot/vision_test3/src/vision_streaming.cpp

CMakeFiles/vision_streaming.dir/src/vision_streaming.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/vision_streaming.dir/src/vision_streaming.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/david/rosws/paper-robot/vision_test3/src/vision_streaming.cpp > CMakeFiles/vision_streaming.dir/src/vision_streaming.i

CMakeFiles/vision_streaming.dir/src/vision_streaming.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/vision_streaming.dir/src/vision_streaming.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/david/rosws/paper-robot/vision_test3/src/vision_streaming.cpp -o CMakeFiles/vision_streaming.dir/src/vision_streaming.s

CMakeFiles/vision_streaming.dir/src/vision_streaming.o.requires:
.PHONY : CMakeFiles/vision_streaming.dir/src/vision_streaming.o.requires

CMakeFiles/vision_streaming.dir/src/vision_streaming.o.provides: CMakeFiles/vision_streaming.dir/src/vision_streaming.o.requires
	$(MAKE) -f CMakeFiles/vision_streaming.dir/build.make CMakeFiles/vision_streaming.dir/src/vision_streaming.o.provides.build
.PHONY : CMakeFiles/vision_streaming.dir/src/vision_streaming.o.provides

CMakeFiles/vision_streaming.dir/src/vision_streaming.o.provides.build: CMakeFiles/vision_streaming.dir/src/vision_streaming.o

# Object files for target vision_streaming
vision_streaming_OBJECTS = \
"CMakeFiles/vision_streaming.dir/src/vision_streaming.o"

# External object files for target vision_streaming
vision_streaming_EXTERNAL_OBJECTS =

../bin/vision_streaming: CMakeFiles/vision_streaming.dir/src/vision_streaming.o
../bin/vision_streaming: /opt/ros/fuerte/lib/libopencv_calib3d.so
../bin/vision_streaming: /opt/ros/fuerte/lib/libopencv_contrib.so
../bin/vision_streaming: /opt/ros/fuerte/lib/libopencv_core.so
../bin/vision_streaming: /opt/ros/fuerte/lib/libopencv_features2d.so
../bin/vision_streaming: /opt/ros/fuerte/lib/libopencv_flann.so
../bin/vision_streaming: /opt/ros/fuerte/lib/libopencv_gpu.so
../bin/vision_streaming: /opt/ros/fuerte/lib/libopencv_highgui.so
../bin/vision_streaming: /opt/ros/fuerte/lib/libopencv_imgproc.so
../bin/vision_streaming: /opt/ros/fuerte/lib/libopencv_legacy.so
../bin/vision_streaming: /opt/ros/fuerte/lib/libopencv_ml.so
../bin/vision_streaming: /opt/ros/fuerte/lib/libopencv_nonfree.so
../bin/vision_streaming: /opt/ros/fuerte/lib/libopencv_objdetect.so
../bin/vision_streaming: /opt/ros/fuerte/lib/libopencv_photo.so
../bin/vision_streaming: /opt/ros/fuerte/lib/libopencv_stitching.so
../bin/vision_streaming: /opt/ros/fuerte/lib/libopencv_ts.so
../bin/vision_streaming: /opt/ros/fuerte/lib/libopencv_video.so
../bin/vision_streaming: /opt/ros/fuerte/lib/libopencv_videostab.so
../bin/vision_streaming: CMakeFiles/vision_streaming.dir/build.make
../bin/vision_streaming: CMakeFiles/vision_streaming.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/vision_streaming"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/vision_streaming.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/vision_streaming.dir/build: ../bin/vision_streaming
.PHONY : CMakeFiles/vision_streaming.dir/build

CMakeFiles/vision_streaming.dir/requires: CMakeFiles/vision_streaming.dir/src/vision_streaming.o.requires
.PHONY : CMakeFiles/vision_streaming.dir/requires

CMakeFiles/vision_streaming.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/vision_streaming.dir/cmake_clean.cmake
.PHONY : CMakeFiles/vision_streaming.dir/clean

CMakeFiles/vision_streaming.dir/depend:
	cd /home/david/rosws/paper-robot/vision_test3/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/david/rosws/paper-robot/vision_test3 /home/david/rosws/paper-robot/vision_test3 /home/david/rosws/paper-robot/vision_test3/build /home/david/rosws/paper-robot/vision_test3/build /home/david/rosws/paper-robot/vision_test3/build/CMakeFiles/vision_streaming.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/vision_streaming.dir/depend
