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
CMAKE_SOURCE_DIR = /home/fujii/libmikataarm

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/fujii/libmikataarm/build

# Include any dependencies generated for this target.
include libdynamixel/src/CMakeFiles/dynamixel_test.dir/depend.make

# Include the progress variables for this target.
include libdynamixel/src/CMakeFiles/dynamixel_test.dir/progress.make

# Include the compile flags for this target's objects.
include libdynamixel/src/CMakeFiles/dynamixel_test.dir/flags.make

libdynamixel/src/CMakeFiles/dynamixel_test.dir/test.cpp.o: libdynamixel/src/CMakeFiles/dynamixel_test.dir/flags.make
libdynamixel/src/CMakeFiles/dynamixel_test.dir/test.cpp.o: ../libdynamixel/src/test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/fujii/libmikataarm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object libdynamixel/src/CMakeFiles/dynamixel_test.dir/test.cpp.o"
	cd /home/fujii/libmikataarm/build/libdynamixel/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dynamixel_test.dir/test.cpp.o -c /home/fujii/libmikataarm/libdynamixel/src/test.cpp

libdynamixel/src/CMakeFiles/dynamixel_test.dir/test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dynamixel_test.dir/test.cpp.i"
	cd /home/fujii/libmikataarm/build/libdynamixel/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/fujii/libmikataarm/libdynamixel/src/test.cpp > CMakeFiles/dynamixel_test.dir/test.cpp.i

libdynamixel/src/CMakeFiles/dynamixel_test.dir/test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dynamixel_test.dir/test.cpp.s"
	cd /home/fujii/libmikataarm/build/libdynamixel/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/fujii/libmikataarm/libdynamixel/src/test.cpp -o CMakeFiles/dynamixel_test.dir/test.cpp.s

libdynamixel/src/CMakeFiles/dynamixel_test.dir/test.cpp.o.requires:

.PHONY : libdynamixel/src/CMakeFiles/dynamixel_test.dir/test.cpp.o.requires

libdynamixel/src/CMakeFiles/dynamixel_test.dir/test.cpp.o.provides: libdynamixel/src/CMakeFiles/dynamixel_test.dir/test.cpp.o.requires
	$(MAKE) -f libdynamixel/src/CMakeFiles/dynamixel_test.dir/build.make libdynamixel/src/CMakeFiles/dynamixel_test.dir/test.cpp.o.provides.build
.PHONY : libdynamixel/src/CMakeFiles/dynamixel_test.dir/test.cpp.o.provides

libdynamixel/src/CMakeFiles/dynamixel_test.dir/test.cpp.o.provides.build: libdynamixel/src/CMakeFiles/dynamixel_test.dir/test.cpp.o


# Object files for target dynamixel_test
dynamixel_test_OBJECTS = \
"CMakeFiles/dynamixel_test.dir/test.cpp.o"

# External object files for target dynamixel_test
dynamixel_test_EXTERNAL_OBJECTS =

libdynamixel/src/dynamixel_test: libdynamixel/src/CMakeFiles/dynamixel_test.dir/test.cpp.o
libdynamixel/src/dynamixel_test: libdynamixel/src/CMakeFiles/dynamixel_test.dir/build.make
libdynamixel/src/dynamixel_test: libdynamixel/src/libdynamixel.so
libdynamixel/src/dynamixel_test: libdynamixel/src/CMakeFiles/dynamixel_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/fujii/libmikataarm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable dynamixel_test"
	cd /home/fujii/libmikataarm/build/libdynamixel/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/dynamixel_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
libdynamixel/src/CMakeFiles/dynamixel_test.dir/build: libdynamixel/src/dynamixel_test

.PHONY : libdynamixel/src/CMakeFiles/dynamixel_test.dir/build

libdynamixel/src/CMakeFiles/dynamixel_test.dir/requires: libdynamixel/src/CMakeFiles/dynamixel_test.dir/test.cpp.o.requires

.PHONY : libdynamixel/src/CMakeFiles/dynamixel_test.dir/requires

libdynamixel/src/CMakeFiles/dynamixel_test.dir/clean:
	cd /home/fujii/libmikataarm/build/libdynamixel/src && $(CMAKE_COMMAND) -P CMakeFiles/dynamixel_test.dir/cmake_clean.cmake
.PHONY : libdynamixel/src/CMakeFiles/dynamixel_test.dir/clean

libdynamixel/src/CMakeFiles/dynamixel_test.dir/depend:
	cd /home/fujii/libmikataarm/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/fujii/libmikataarm /home/fujii/libmikataarm/libdynamixel/src /home/fujii/libmikataarm/build /home/fujii/libmikataarm/build/libdynamixel/src /home/fujii/libmikataarm/build/libdynamixel/src/CMakeFiles/dynamixel_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : libdynamixel/src/CMakeFiles/dynamixel_test.dir/depend

