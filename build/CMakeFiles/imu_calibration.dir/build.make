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
CMAKE_SOURCE_DIR = /home/oneai/work/git/libImuCalibraion/imu_calibration

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/oneai/work/git/libImuCalibraion/imu_calibration/build

# Include any dependencies generated for this target.
include CMakeFiles/imu_calibration.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/imu_calibration.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/imu_calibration.dir/flags.make

CMakeFiles/imu_calibration.dir/src/imu_protocol.cpp.o: CMakeFiles/imu_calibration.dir/flags.make
CMakeFiles/imu_calibration.dir/src/imu_protocol.cpp.o: ../src/imu_protocol.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/oneai/work/git/libImuCalibraion/imu_calibration/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/imu_calibration.dir/src/imu_protocol.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/imu_calibration.dir/src/imu_protocol.cpp.o -c /home/oneai/work/git/libImuCalibraion/imu_calibration/src/imu_protocol.cpp

CMakeFiles/imu_calibration.dir/src/imu_protocol.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/imu_calibration.dir/src/imu_protocol.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/oneai/work/git/libImuCalibraion/imu_calibration/src/imu_protocol.cpp > CMakeFiles/imu_calibration.dir/src/imu_protocol.cpp.i

CMakeFiles/imu_calibration.dir/src/imu_protocol.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/imu_calibration.dir/src/imu_protocol.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/oneai/work/git/libImuCalibraion/imu_calibration/src/imu_protocol.cpp -o CMakeFiles/imu_calibration.dir/src/imu_protocol.cpp.s

CMakeFiles/imu_calibration.dir/src/imu_protocol.cpp.o.requires:

.PHONY : CMakeFiles/imu_calibration.dir/src/imu_protocol.cpp.o.requires

CMakeFiles/imu_calibration.dir/src/imu_protocol.cpp.o.provides: CMakeFiles/imu_calibration.dir/src/imu_protocol.cpp.o.requires
	$(MAKE) -f CMakeFiles/imu_calibration.dir/build.make CMakeFiles/imu_calibration.dir/src/imu_protocol.cpp.o.provides.build
.PHONY : CMakeFiles/imu_calibration.dir/src/imu_protocol.cpp.o.provides

CMakeFiles/imu_calibration.dir/src/imu_protocol.cpp.o.provides.build: CMakeFiles/imu_calibration.dir/src/imu_protocol.cpp.o


CMakeFiles/imu_calibration.dir/src/serialport.cpp.o: CMakeFiles/imu_calibration.dir/flags.make
CMakeFiles/imu_calibration.dir/src/serialport.cpp.o: ../src/serialport.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/oneai/work/git/libImuCalibraion/imu_calibration/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/imu_calibration.dir/src/serialport.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/imu_calibration.dir/src/serialport.cpp.o -c /home/oneai/work/git/libImuCalibraion/imu_calibration/src/serialport.cpp

CMakeFiles/imu_calibration.dir/src/serialport.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/imu_calibration.dir/src/serialport.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/oneai/work/git/libImuCalibraion/imu_calibration/src/serialport.cpp > CMakeFiles/imu_calibration.dir/src/serialport.cpp.i

CMakeFiles/imu_calibration.dir/src/serialport.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/imu_calibration.dir/src/serialport.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/oneai/work/git/libImuCalibraion/imu_calibration/src/serialport.cpp -o CMakeFiles/imu_calibration.dir/src/serialport.cpp.s

CMakeFiles/imu_calibration.dir/src/serialport.cpp.o.requires:

.PHONY : CMakeFiles/imu_calibration.dir/src/serialport.cpp.o.requires

CMakeFiles/imu_calibration.dir/src/serialport.cpp.o.provides: CMakeFiles/imu_calibration.dir/src/serialport.cpp.o.requires
	$(MAKE) -f CMakeFiles/imu_calibration.dir/build.make CMakeFiles/imu_calibration.dir/src/serialport.cpp.o.provides.build
.PHONY : CMakeFiles/imu_calibration.dir/src/serialport.cpp.o.provides

CMakeFiles/imu_calibration.dir/src/serialport.cpp.o.provides.build: CMakeFiles/imu_calibration.dir/src/serialport.cpp.o


CMakeFiles/imu_calibration.dir/src/io_interface.cpp.o: CMakeFiles/imu_calibration.dir/flags.make
CMakeFiles/imu_calibration.dir/src/io_interface.cpp.o: ../src/io_interface.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/oneai/work/git/libImuCalibraion/imu_calibration/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/imu_calibration.dir/src/io_interface.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/imu_calibration.dir/src/io_interface.cpp.o -c /home/oneai/work/git/libImuCalibraion/imu_calibration/src/io_interface.cpp

CMakeFiles/imu_calibration.dir/src/io_interface.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/imu_calibration.dir/src/io_interface.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/oneai/work/git/libImuCalibraion/imu_calibration/src/io_interface.cpp > CMakeFiles/imu_calibration.dir/src/io_interface.cpp.i

CMakeFiles/imu_calibration.dir/src/io_interface.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/imu_calibration.dir/src/io_interface.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/oneai/work/git/libImuCalibraion/imu_calibration/src/io_interface.cpp -o CMakeFiles/imu_calibration.dir/src/io_interface.cpp.s

CMakeFiles/imu_calibration.dir/src/io_interface.cpp.o.requires:

.PHONY : CMakeFiles/imu_calibration.dir/src/io_interface.cpp.o.requires

CMakeFiles/imu_calibration.dir/src/io_interface.cpp.o.provides: CMakeFiles/imu_calibration.dir/src/io_interface.cpp.o.requires
	$(MAKE) -f CMakeFiles/imu_calibration.dir/build.make CMakeFiles/imu_calibration.dir/src/io_interface.cpp.o.provides.build
.PHONY : CMakeFiles/imu_calibration.dir/src/io_interface.cpp.o.provides

CMakeFiles/imu_calibration.dir/src/io_interface.cpp.o.provides.build: CMakeFiles/imu_calibration.dir/src/io_interface.cpp.o


CMakeFiles/imu_calibration.dir/src/imu_data_handle.cpp.o: CMakeFiles/imu_calibration.dir/flags.make
CMakeFiles/imu_calibration.dir/src/imu_data_handle.cpp.o: ../src/imu_data_handle.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/oneai/work/git/libImuCalibraion/imu_calibration/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/imu_calibration.dir/src/imu_data_handle.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/imu_calibration.dir/src/imu_data_handle.cpp.o -c /home/oneai/work/git/libImuCalibraion/imu_calibration/src/imu_data_handle.cpp

CMakeFiles/imu_calibration.dir/src/imu_data_handle.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/imu_calibration.dir/src/imu_data_handle.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/oneai/work/git/libImuCalibraion/imu_calibration/src/imu_data_handle.cpp > CMakeFiles/imu_calibration.dir/src/imu_data_handle.cpp.i

CMakeFiles/imu_calibration.dir/src/imu_data_handle.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/imu_calibration.dir/src/imu_data_handle.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/oneai/work/git/libImuCalibraion/imu_calibration/src/imu_data_handle.cpp -o CMakeFiles/imu_calibration.dir/src/imu_data_handle.cpp.s

CMakeFiles/imu_calibration.dir/src/imu_data_handle.cpp.o.requires:

.PHONY : CMakeFiles/imu_calibration.dir/src/imu_data_handle.cpp.o.requires

CMakeFiles/imu_calibration.dir/src/imu_data_handle.cpp.o.provides: CMakeFiles/imu_calibration.dir/src/imu_data_handle.cpp.o.requires
	$(MAKE) -f CMakeFiles/imu_calibration.dir/build.make CMakeFiles/imu_calibration.dir/src/imu_data_handle.cpp.o.provides.build
.PHONY : CMakeFiles/imu_calibration.dir/src/imu_data_handle.cpp.o.provides

CMakeFiles/imu_calibration.dir/src/imu_data_handle.cpp.o.provides.build: CMakeFiles/imu_calibration.dir/src/imu_data_handle.cpp.o


CMakeFiles/imu_calibration.dir/src/utils.cpp.o: CMakeFiles/imu_calibration.dir/flags.make
CMakeFiles/imu_calibration.dir/src/utils.cpp.o: ../src/utils.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/oneai/work/git/libImuCalibraion/imu_calibration/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/imu_calibration.dir/src/utils.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/imu_calibration.dir/src/utils.cpp.o -c /home/oneai/work/git/libImuCalibraion/imu_calibration/src/utils.cpp

CMakeFiles/imu_calibration.dir/src/utils.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/imu_calibration.dir/src/utils.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/oneai/work/git/libImuCalibraion/imu_calibration/src/utils.cpp > CMakeFiles/imu_calibration.dir/src/utils.cpp.i

CMakeFiles/imu_calibration.dir/src/utils.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/imu_calibration.dir/src/utils.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/oneai/work/git/libImuCalibraion/imu_calibration/src/utils.cpp -o CMakeFiles/imu_calibration.dir/src/utils.cpp.s

CMakeFiles/imu_calibration.dir/src/utils.cpp.o.requires:

.PHONY : CMakeFiles/imu_calibration.dir/src/utils.cpp.o.requires

CMakeFiles/imu_calibration.dir/src/utils.cpp.o.provides: CMakeFiles/imu_calibration.dir/src/utils.cpp.o.requires
	$(MAKE) -f CMakeFiles/imu_calibration.dir/build.make CMakeFiles/imu_calibration.dir/src/utils.cpp.o.provides.build
.PHONY : CMakeFiles/imu_calibration.dir/src/utils.cpp.o.provides

CMakeFiles/imu_calibration.dir/src/utils.cpp.o.provides.build: CMakeFiles/imu_calibration.dir/src/utils.cpp.o


# Object files for target imu_calibration
imu_calibration_OBJECTS = \
"CMakeFiles/imu_calibration.dir/src/imu_protocol.cpp.o" \
"CMakeFiles/imu_calibration.dir/src/serialport.cpp.o" \
"CMakeFiles/imu_calibration.dir/src/io_interface.cpp.o" \
"CMakeFiles/imu_calibration.dir/src/imu_data_handle.cpp.o" \
"CMakeFiles/imu_calibration.dir/src/utils.cpp.o"

# External object files for target imu_calibration
imu_calibration_EXTERNAL_OBJECTS =

libimu_calibration.so: CMakeFiles/imu_calibration.dir/src/imu_protocol.cpp.o
libimu_calibration.so: CMakeFiles/imu_calibration.dir/src/serialport.cpp.o
libimu_calibration.so: CMakeFiles/imu_calibration.dir/src/io_interface.cpp.o
libimu_calibration.so: CMakeFiles/imu_calibration.dir/src/imu_data_handle.cpp.o
libimu_calibration.so: CMakeFiles/imu_calibration.dir/src/utils.cpp.o
libimu_calibration.so: CMakeFiles/imu_calibration.dir/build.make
libimu_calibration.so: CMakeFiles/imu_calibration.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/oneai/work/git/libImuCalibraion/imu_calibration/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX shared library libimu_calibration.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/imu_calibration.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/imu_calibration.dir/build: libimu_calibration.so

.PHONY : CMakeFiles/imu_calibration.dir/build

CMakeFiles/imu_calibration.dir/requires: CMakeFiles/imu_calibration.dir/src/imu_protocol.cpp.o.requires
CMakeFiles/imu_calibration.dir/requires: CMakeFiles/imu_calibration.dir/src/serialport.cpp.o.requires
CMakeFiles/imu_calibration.dir/requires: CMakeFiles/imu_calibration.dir/src/io_interface.cpp.o.requires
CMakeFiles/imu_calibration.dir/requires: CMakeFiles/imu_calibration.dir/src/imu_data_handle.cpp.o.requires
CMakeFiles/imu_calibration.dir/requires: CMakeFiles/imu_calibration.dir/src/utils.cpp.o.requires

.PHONY : CMakeFiles/imu_calibration.dir/requires

CMakeFiles/imu_calibration.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/imu_calibration.dir/cmake_clean.cmake
.PHONY : CMakeFiles/imu_calibration.dir/clean

CMakeFiles/imu_calibration.dir/depend:
	cd /home/oneai/work/git/libImuCalibraion/imu_calibration/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/oneai/work/git/libImuCalibraion/imu_calibration /home/oneai/work/git/libImuCalibraion/imu_calibration /home/oneai/work/git/libImuCalibraion/imu_calibration/build /home/oneai/work/git/libImuCalibraion/imu_calibration/build /home/oneai/work/git/libImuCalibraion/imu_calibration/build/CMakeFiles/imu_calibration.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/imu_calibration.dir/depend

