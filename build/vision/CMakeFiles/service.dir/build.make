# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/davidzhang/Robotics/Octo/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/davidzhang/Robotics/Octo/build

# Include any dependencies generated for this target.
include vision/CMakeFiles/service.dir/depend.make

# Include the progress variables for this target.
include vision/CMakeFiles/service.dir/progress.make

# Include the compile flags for this target's objects.
include vision/CMakeFiles/service.dir/flags.make

vision/CMakeFiles/service.dir/src/detect.cpp.o: vision/CMakeFiles/service.dir/flags.make
vision/CMakeFiles/service.dir/src/detect.cpp.o: /home/davidzhang/Robotics/Octo/src/vision/src/detect.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/davidzhang/Robotics/Octo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object vision/CMakeFiles/service.dir/src/detect.cpp.o"
	cd /home/davidzhang/Robotics/Octo/build/vision && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/service.dir/src/detect.cpp.o -c /home/davidzhang/Robotics/Octo/src/vision/src/detect.cpp

vision/CMakeFiles/service.dir/src/detect.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/service.dir/src/detect.cpp.i"
	cd /home/davidzhang/Robotics/Octo/build/vision && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/davidzhang/Robotics/Octo/src/vision/src/detect.cpp > CMakeFiles/service.dir/src/detect.cpp.i

vision/CMakeFiles/service.dir/src/detect.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/service.dir/src/detect.cpp.s"
	cd /home/davidzhang/Robotics/Octo/build/vision && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/davidzhang/Robotics/Octo/src/vision/src/detect.cpp -o CMakeFiles/service.dir/src/detect.cpp.s

vision/CMakeFiles/service.dir/src/detect.cpp.o.requires:

.PHONY : vision/CMakeFiles/service.dir/src/detect.cpp.o.requires

vision/CMakeFiles/service.dir/src/detect.cpp.o.provides: vision/CMakeFiles/service.dir/src/detect.cpp.o.requires
	$(MAKE) -f vision/CMakeFiles/service.dir/build.make vision/CMakeFiles/service.dir/src/detect.cpp.o.provides.build
.PHONY : vision/CMakeFiles/service.dir/src/detect.cpp.o.provides

vision/CMakeFiles/service.dir/src/detect.cpp.o.provides.build: vision/CMakeFiles/service.dir/src/detect.cpp.o


vision/CMakeFiles/service.dir/src/capture.cpp.o: vision/CMakeFiles/service.dir/flags.make
vision/CMakeFiles/service.dir/src/capture.cpp.o: /home/davidzhang/Robotics/Octo/src/vision/src/capture.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/davidzhang/Robotics/Octo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object vision/CMakeFiles/service.dir/src/capture.cpp.o"
	cd /home/davidzhang/Robotics/Octo/build/vision && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/service.dir/src/capture.cpp.o -c /home/davidzhang/Robotics/Octo/src/vision/src/capture.cpp

vision/CMakeFiles/service.dir/src/capture.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/service.dir/src/capture.cpp.i"
	cd /home/davidzhang/Robotics/Octo/build/vision && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/davidzhang/Robotics/Octo/src/vision/src/capture.cpp > CMakeFiles/service.dir/src/capture.cpp.i

vision/CMakeFiles/service.dir/src/capture.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/service.dir/src/capture.cpp.s"
	cd /home/davidzhang/Robotics/Octo/build/vision && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/davidzhang/Robotics/Octo/src/vision/src/capture.cpp -o CMakeFiles/service.dir/src/capture.cpp.s

vision/CMakeFiles/service.dir/src/capture.cpp.o.requires:

.PHONY : vision/CMakeFiles/service.dir/src/capture.cpp.o.requires

vision/CMakeFiles/service.dir/src/capture.cpp.o.provides: vision/CMakeFiles/service.dir/src/capture.cpp.o.requires
	$(MAKE) -f vision/CMakeFiles/service.dir/build.make vision/CMakeFiles/service.dir/src/capture.cpp.o.provides.build
.PHONY : vision/CMakeFiles/service.dir/src/capture.cpp.o.provides

vision/CMakeFiles/service.dir/src/capture.cpp.o.provides.build: vision/CMakeFiles/service.dir/src/capture.cpp.o


# Object files for target service
service_OBJECTS = \
"CMakeFiles/service.dir/src/detect.cpp.o" \
"CMakeFiles/service.dir/src/capture.cpp.o"

# External object files for target service
service_EXTERNAL_OBJECTS =

/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: vision/CMakeFiles/service.dir/src/detect.cpp.o
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: vision/CMakeFiles/service.dir/src/capture.cpp.o
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: vision/CMakeFiles/service.dir/build.make
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /opt/ros/melodic/lib/libcv_bridge.so
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /opt/ros/melodic/lib/libimage_transport.so
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /opt/ros/melodic/lib/libmessage_filters.so
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /opt/ros/melodic/lib/libclass_loader.so
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/libPocoFoundation.so
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libdl.so
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /opt/ros/melodic/lib/libroscpp.so
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /opt/ros/melodic/lib/librosconsole.so
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /opt/ros/melodic/lib/libroslib.so
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /opt/ros/melodic/lib/librospack.so
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libpython2.7.so
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /opt/ros/melodic/lib/librostime.so
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /opt/ros/melodic/lib/libcpp_common.so
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.3.2.0
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.3.2.0
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.3.2.0
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.3.2.0
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.3.2.0
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.3.2.0
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.3.2.0
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.3.2.0
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.3.2.0
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.3.2.0
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libopencv_face.so.3.2.0
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.3.2.0
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.3.2.0
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.3.2.0
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.3.2.0
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.3.2.0
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.3.2.0
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.3.2.0
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.3.2.0
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.3.2.0
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.3.2.0
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.3.2.0
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libopencv_text.so.3.2.0
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.3.2.0
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.3.2.0
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.3.2.0
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libopencv_video.so.3.2.0
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.3.2.0
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.3.2.0
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.3.2.0
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.3.2.0
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.3.2.0
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.3.2.0
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.3.2.0
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.3.2.0
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.3.2.0
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
/home/davidzhang/Robotics/Octo/devel/lib/libservice.so: vision/CMakeFiles/service.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/davidzhang/Robotics/Octo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX shared library /home/davidzhang/Robotics/Octo/devel/lib/libservice.so"
	cd /home/davidzhang/Robotics/Octo/build/vision && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/service.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
vision/CMakeFiles/service.dir/build: /home/davidzhang/Robotics/Octo/devel/lib/libservice.so

.PHONY : vision/CMakeFiles/service.dir/build

vision/CMakeFiles/service.dir/requires: vision/CMakeFiles/service.dir/src/detect.cpp.o.requires
vision/CMakeFiles/service.dir/requires: vision/CMakeFiles/service.dir/src/capture.cpp.o.requires

.PHONY : vision/CMakeFiles/service.dir/requires

vision/CMakeFiles/service.dir/clean:
	cd /home/davidzhang/Robotics/Octo/build/vision && $(CMAKE_COMMAND) -P CMakeFiles/service.dir/cmake_clean.cmake
.PHONY : vision/CMakeFiles/service.dir/clean

vision/CMakeFiles/service.dir/depend:
	cd /home/davidzhang/Robotics/Octo/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/davidzhang/Robotics/Octo/src /home/davidzhang/Robotics/Octo/src/vision /home/davidzhang/Robotics/Octo/build /home/davidzhang/Robotics/Octo/build/vision /home/davidzhang/Robotics/Octo/build/vision/CMakeFiles/service.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : vision/CMakeFiles/service.dir/depend
