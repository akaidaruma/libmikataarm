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
include test/CMakeFiles/mikata_move_example01.dir/depend.make

# Include the progress variables for this target.
include test/CMakeFiles/mikata_move_example01.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/mikata_move_example01.dir/flags.make

test/CMakeFiles/mikata_move_example01.dir/mikata_move_example01.cpp.o: test/CMakeFiles/mikata_move_example01.dir/flags.make
test/CMakeFiles/mikata_move_example01.dir/mikata_move_example01.cpp.o: ../test/mikata_move_example01.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/fujii/libmikataarm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/mikata_move_example01.dir/mikata_move_example01.cpp.o"
	cd /home/fujii/libmikataarm/build/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mikata_move_example01.dir/mikata_move_example01.cpp.o -c /home/fujii/libmikataarm/test/mikata_move_example01.cpp

test/CMakeFiles/mikata_move_example01.dir/mikata_move_example01.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mikata_move_example01.dir/mikata_move_example01.cpp.i"
	cd /home/fujii/libmikataarm/build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/fujii/libmikataarm/test/mikata_move_example01.cpp > CMakeFiles/mikata_move_example01.dir/mikata_move_example01.cpp.i

test/CMakeFiles/mikata_move_example01.dir/mikata_move_example01.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mikata_move_example01.dir/mikata_move_example01.cpp.s"
	cd /home/fujii/libmikataarm/build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/fujii/libmikataarm/test/mikata_move_example01.cpp -o CMakeFiles/mikata_move_example01.dir/mikata_move_example01.cpp.s

test/CMakeFiles/mikata_move_example01.dir/mikata_move_example01.cpp.o.requires:

.PHONY : test/CMakeFiles/mikata_move_example01.dir/mikata_move_example01.cpp.o.requires

test/CMakeFiles/mikata_move_example01.dir/mikata_move_example01.cpp.o.provides: test/CMakeFiles/mikata_move_example01.dir/mikata_move_example01.cpp.o.requires
	$(MAKE) -f test/CMakeFiles/mikata_move_example01.dir/build.make test/CMakeFiles/mikata_move_example01.dir/mikata_move_example01.cpp.o.provides.build
.PHONY : test/CMakeFiles/mikata_move_example01.dir/mikata_move_example01.cpp.o.provides

test/CMakeFiles/mikata_move_example01.dir/mikata_move_example01.cpp.o.provides.build: test/CMakeFiles/mikata_move_example01.dir/mikata_move_example01.cpp.o


# Object files for target mikata_move_example01
mikata_move_example01_OBJECTS = \
"CMakeFiles/mikata_move_example01.dir/mikata_move_example01.cpp.o"

# External object files for target mikata_move_example01
mikata_move_example01_EXTERNAL_OBJECTS =

test/mikata_move_example01: test/CMakeFiles/mikata_move_example01.dir/mikata_move_example01.cpp.o
test/mikata_move_example01: test/CMakeFiles/mikata_move_example01.dir/build.make
test/mikata_move_example01: src/libmikata.so
test/mikata_move_example01: libdynamixel/src/libdynamixel.so
test/mikata_move_example01: test/CMakeFiles/mikata_move_example01.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/fujii/libmikataarm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable mikata_move_example01"
	cd /home/fujii/libmikataarm/build/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mikata_move_example01.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/mikata_move_example01.dir/build: test/mikata_move_example01

.PHONY : test/CMakeFiles/mikata_move_example01.dir/build

test/CMakeFiles/mikata_move_example01.dir/requires: test/CMakeFiles/mikata_move_example01.dir/mikata_move_example01.cpp.o.requires

.PHONY : test/CMakeFiles/mikata_move_example01.dir/requires

test/CMakeFiles/mikata_move_example01.dir/clean:
	cd /home/fujii/libmikataarm/build/test && $(CMAKE_COMMAND) -P CMakeFiles/mikata_move_example01.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/mikata_move_example01.dir/clean

test/CMakeFiles/mikata_move_example01.dir/depend:
	cd /home/fujii/libmikataarm/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/fujii/libmikataarm /home/fujii/libmikataarm/test /home/fujii/libmikataarm/build /home/fujii/libmikataarm/build/test /home/fujii/libmikataarm/build/test/CMakeFiles/mikata_move_example01.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/mikata_move_example01.dir/depend

