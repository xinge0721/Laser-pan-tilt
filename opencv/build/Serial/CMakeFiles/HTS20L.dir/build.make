# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/ros/chenxu/Laser-pan-tilt/opencv/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ros/chenxu/Laser-pan-tilt/opencv/build

# Include any dependencies generated for this target.
include Serial/CMakeFiles/HTS20L.dir/depend.make

# Include the progress variables for this target.
include Serial/CMakeFiles/HTS20L.dir/progress.make

# Include the compile flags for this target's objects.
include Serial/CMakeFiles/HTS20L.dir/flags.make

Serial/CMakeFiles/HTS20L.dir/src/main.cpp.o: Serial/CMakeFiles/HTS20L.dir/flags.make
Serial/CMakeFiles/HTS20L.dir/src/main.cpp.o: /home/ros/chenxu/Laser-pan-tilt/opencv/src/Serial/src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ros/chenxu/Laser-pan-tilt/opencv/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object Serial/CMakeFiles/HTS20L.dir/src/main.cpp.o"
	cd /home/ros/chenxu/Laser-pan-tilt/opencv/build/Serial && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/HTS20L.dir/src/main.cpp.o -c /home/ros/chenxu/Laser-pan-tilt/opencv/src/Serial/src/main.cpp

Serial/CMakeFiles/HTS20L.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/HTS20L.dir/src/main.cpp.i"
	cd /home/ros/chenxu/Laser-pan-tilt/opencv/build/Serial && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ros/chenxu/Laser-pan-tilt/opencv/src/Serial/src/main.cpp > CMakeFiles/HTS20L.dir/src/main.cpp.i

Serial/CMakeFiles/HTS20L.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/HTS20L.dir/src/main.cpp.s"
	cd /home/ros/chenxu/Laser-pan-tilt/opencv/build/Serial && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ros/chenxu/Laser-pan-tilt/opencv/src/Serial/src/main.cpp -o CMakeFiles/HTS20L.dir/src/main.cpp.s

Serial/CMakeFiles/HTS20L.dir/src/HTS221/HTS221.cpp.o: Serial/CMakeFiles/HTS20L.dir/flags.make
Serial/CMakeFiles/HTS20L.dir/src/HTS221/HTS221.cpp.o: /home/ros/chenxu/Laser-pan-tilt/opencv/src/Serial/src/HTS221/HTS221.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ros/chenxu/Laser-pan-tilt/opencv/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object Serial/CMakeFiles/HTS20L.dir/src/HTS221/HTS221.cpp.o"
	cd /home/ros/chenxu/Laser-pan-tilt/opencv/build/Serial && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/HTS20L.dir/src/HTS221/HTS221.cpp.o -c /home/ros/chenxu/Laser-pan-tilt/opencv/src/Serial/src/HTS221/HTS221.cpp

Serial/CMakeFiles/HTS20L.dir/src/HTS221/HTS221.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/HTS20L.dir/src/HTS221/HTS221.cpp.i"
	cd /home/ros/chenxu/Laser-pan-tilt/opencv/build/Serial && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ros/chenxu/Laser-pan-tilt/opencv/src/Serial/src/HTS221/HTS221.cpp > CMakeFiles/HTS20L.dir/src/HTS221/HTS221.cpp.i

Serial/CMakeFiles/HTS20L.dir/src/HTS221/HTS221.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/HTS20L.dir/src/HTS221/HTS221.cpp.s"
	cd /home/ros/chenxu/Laser-pan-tilt/opencv/build/Serial && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ros/chenxu/Laser-pan-tilt/opencv/src/Serial/src/HTS221/HTS221.cpp -o CMakeFiles/HTS20L.dir/src/HTS221/HTS221.cpp.s

Serial/CMakeFiles/HTS20L.dir/src/Serial/Serial.cpp.o: Serial/CMakeFiles/HTS20L.dir/flags.make
Serial/CMakeFiles/HTS20L.dir/src/Serial/Serial.cpp.o: /home/ros/chenxu/Laser-pan-tilt/opencv/src/Serial/src/Serial/Serial.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ros/chenxu/Laser-pan-tilt/opencv/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object Serial/CMakeFiles/HTS20L.dir/src/Serial/Serial.cpp.o"
	cd /home/ros/chenxu/Laser-pan-tilt/opencv/build/Serial && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/HTS20L.dir/src/Serial/Serial.cpp.o -c /home/ros/chenxu/Laser-pan-tilt/opencv/src/Serial/src/Serial/Serial.cpp

Serial/CMakeFiles/HTS20L.dir/src/Serial/Serial.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/HTS20L.dir/src/Serial/Serial.cpp.i"
	cd /home/ros/chenxu/Laser-pan-tilt/opencv/build/Serial && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ros/chenxu/Laser-pan-tilt/opencv/src/Serial/src/Serial/Serial.cpp > CMakeFiles/HTS20L.dir/src/Serial/Serial.cpp.i

Serial/CMakeFiles/HTS20L.dir/src/Serial/Serial.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/HTS20L.dir/src/Serial/Serial.cpp.s"
	cd /home/ros/chenxu/Laser-pan-tilt/opencv/build/Serial && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ros/chenxu/Laser-pan-tilt/opencv/src/Serial/src/Serial/Serial.cpp -o CMakeFiles/HTS20L.dir/src/Serial/Serial.cpp.s

Serial/CMakeFiles/HTS20L.dir/src/speed/speed.cpp.o: Serial/CMakeFiles/HTS20L.dir/flags.make
Serial/CMakeFiles/HTS20L.dir/src/speed/speed.cpp.o: /home/ros/chenxu/Laser-pan-tilt/opencv/src/Serial/src/speed/speed.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ros/chenxu/Laser-pan-tilt/opencv/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object Serial/CMakeFiles/HTS20L.dir/src/speed/speed.cpp.o"
	cd /home/ros/chenxu/Laser-pan-tilt/opencv/build/Serial && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/HTS20L.dir/src/speed/speed.cpp.o -c /home/ros/chenxu/Laser-pan-tilt/opencv/src/Serial/src/speed/speed.cpp

Serial/CMakeFiles/HTS20L.dir/src/speed/speed.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/HTS20L.dir/src/speed/speed.cpp.i"
	cd /home/ros/chenxu/Laser-pan-tilt/opencv/build/Serial && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ros/chenxu/Laser-pan-tilt/opencv/src/Serial/src/speed/speed.cpp > CMakeFiles/HTS20L.dir/src/speed/speed.cpp.i

Serial/CMakeFiles/HTS20L.dir/src/speed/speed.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/HTS20L.dir/src/speed/speed.cpp.s"
	cd /home/ros/chenxu/Laser-pan-tilt/opencv/build/Serial && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ros/chenxu/Laser-pan-tilt/opencv/src/Serial/src/speed/speed.cpp -o CMakeFiles/HTS20L.dir/src/speed/speed.cpp.s

# Object files for target HTS20L
HTS20L_OBJECTS = \
"CMakeFiles/HTS20L.dir/src/main.cpp.o" \
"CMakeFiles/HTS20L.dir/src/HTS221/HTS221.cpp.o" \
"CMakeFiles/HTS20L.dir/src/Serial/Serial.cpp.o" \
"CMakeFiles/HTS20L.dir/src/speed/speed.cpp.o"

# External object files for target HTS20L
HTS20L_EXTERNAL_OBJECTS =

/home/ros/chenxu/Laser-pan-tilt/opencv/devel/lib/Serial/HTS20L: Serial/CMakeFiles/HTS20L.dir/src/main.cpp.o
/home/ros/chenxu/Laser-pan-tilt/opencv/devel/lib/Serial/HTS20L: Serial/CMakeFiles/HTS20L.dir/src/HTS221/HTS221.cpp.o
/home/ros/chenxu/Laser-pan-tilt/opencv/devel/lib/Serial/HTS20L: Serial/CMakeFiles/HTS20L.dir/src/Serial/Serial.cpp.o
/home/ros/chenxu/Laser-pan-tilt/opencv/devel/lib/Serial/HTS20L: Serial/CMakeFiles/HTS20L.dir/src/speed/speed.cpp.o
/home/ros/chenxu/Laser-pan-tilt/opencv/devel/lib/Serial/HTS20L: Serial/CMakeFiles/HTS20L.dir/build.make
/home/ros/chenxu/Laser-pan-tilt/opencv/devel/lib/Serial/HTS20L: /opt/ros/noetic/lib/libroscpp.so
/home/ros/chenxu/Laser-pan-tilt/opencv/devel/lib/Serial/HTS20L: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/ros/chenxu/Laser-pan-tilt/opencv/devel/lib/Serial/HTS20L: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/ros/chenxu/Laser-pan-tilt/opencv/devel/lib/Serial/HTS20L: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/ros/chenxu/Laser-pan-tilt/opencv/devel/lib/Serial/HTS20L: /opt/ros/noetic/lib/librosconsole.so
/home/ros/chenxu/Laser-pan-tilt/opencv/devel/lib/Serial/HTS20L: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/ros/chenxu/Laser-pan-tilt/opencv/devel/lib/Serial/HTS20L: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/ros/chenxu/Laser-pan-tilt/opencv/devel/lib/Serial/HTS20L: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/ros/chenxu/Laser-pan-tilt/opencv/devel/lib/Serial/HTS20L: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/ros/chenxu/Laser-pan-tilt/opencv/devel/lib/Serial/HTS20L: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/ros/chenxu/Laser-pan-tilt/opencv/devel/lib/Serial/HTS20L: /opt/ros/noetic/lib/libserial.so
/home/ros/chenxu/Laser-pan-tilt/opencv/devel/lib/Serial/HTS20L: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/ros/chenxu/Laser-pan-tilt/opencv/devel/lib/Serial/HTS20L: /opt/ros/noetic/lib/librostime.so
/home/ros/chenxu/Laser-pan-tilt/opencv/devel/lib/Serial/HTS20L: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/ros/chenxu/Laser-pan-tilt/opencv/devel/lib/Serial/HTS20L: /opt/ros/noetic/lib/libcpp_common.so
/home/ros/chenxu/Laser-pan-tilt/opencv/devel/lib/Serial/HTS20L: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/ros/chenxu/Laser-pan-tilt/opencv/devel/lib/Serial/HTS20L: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/ros/chenxu/Laser-pan-tilt/opencv/devel/lib/Serial/HTS20L: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/ros/chenxu/Laser-pan-tilt/opencv/devel/lib/Serial/HTS20L: /usr/lib/x86_64-linux-gnu/libserial.so
/home/ros/chenxu/Laser-pan-tilt/opencv/devel/lib/Serial/HTS20L: Serial/CMakeFiles/HTS20L.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ros/chenxu/Laser-pan-tilt/opencv/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable /home/ros/chenxu/Laser-pan-tilt/opencv/devel/lib/Serial/HTS20L"
	cd /home/ros/chenxu/Laser-pan-tilt/opencv/build/Serial && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/HTS20L.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Serial/CMakeFiles/HTS20L.dir/build: /home/ros/chenxu/Laser-pan-tilt/opencv/devel/lib/Serial/HTS20L

.PHONY : Serial/CMakeFiles/HTS20L.dir/build

Serial/CMakeFiles/HTS20L.dir/clean:
	cd /home/ros/chenxu/Laser-pan-tilt/opencv/build/Serial && $(CMAKE_COMMAND) -P CMakeFiles/HTS20L.dir/cmake_clean.cmake
.PHONY : Serial/CMakeFiles/HTS20L.dir/clean

Serial/CMakeFiles/HTS20L.dir/depend:
	cd /home/ros/chenxu/Laser-pan-tilt/opencv/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ros/chenxu/Laser-pan-tilt/opencv/src /home/ros/chenxu/Laser-pan-tilt/opencv/src/Serial /home/ros/chenxu/Laser-pan-tilt/opencv/build /home/ros/chenxu/Laser-pan-tilt/opencv/build/Serial /home/ros/chenxu/Laser-pan-tilt/opencv/build/Serial/CMakeFiles/HTS20L.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Serial/CMakeFiles/HTS20L.dir/depend

