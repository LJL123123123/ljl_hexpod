# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/dragon/桌面/bishe/hexpod_ljl

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/dragon/桌面/bishe/hexpod_ljl/build

# Include any dependencies generated for this target.
include Part_Robotrunner/CMakeFiles/TestRobotrunner.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include Part_Robotrunner/CMakeFiles/TestRobotrunner.dir/compiler_depend.make

# Include the progress variables for this target.
include Part_Robotrunner/CMakeFiles/TestRobotrunner.dir/progress.make

# Include the compile flags for this target's objects.
include Part_Robotrunner/CMakeFiles/TestRobotrunner.dir/flags.make

Part_Robotrunner/CMakeFiles/TestRobotrunner.dir/test/test.cpp.o: Part_Robotrunner/CMakeFiles/TestRobotrunner.dir/flags.make
Part_Robotrunner/CMakeFiles/TestRobotrunner.dir/test/test.cpp.o: ../Part_Robotrunner/test/test.cpp
Part_Robotrunner/CMakeFiles/TestRobotrunner.dir/test/test.cpp.o: Part_Robotrunner/CMakeFiles/TestRobotrunner.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dragon/桌面/bishe/hexpod_ljl/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object Part_Robotrunner/CMakeFiles/TestRobotrunner.dir/test/test.cpp.o"
	cd /home/dragon/桌面/bishe/hexpod_ljl/build/Part_Robotrunner && /usr/local/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT Part_Robotrunner/CMakeFiles/TestRobotrunner.dir/test/test.cpp.o -MF CMakeFiles/TestRobotrunner.dir/test/test.cpp.o.d -o CMakeFiles/TestRobotrunner.dir/test/test.cpp.o -c /home/dragon/桌面/bishe/hexpod_ljl/Part_Robotrunner/test/test.cpp

Part_Robotrunner/CMakeFiles/TestRobotrunner.dir/test/test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/TestRobotrunner.dir/test/test.cpp.i"
	cd /home/dragon/桌面/bishe/hexpod_ljl/build/Part_Robotrunner && /usr/local/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dragon/桌面/bishe/hexpod_ljl/Part_Robotrunner/test/test.cpp > CMakeFiles/TestRobotrunner.dir/test/test.cpp.i

Part_Robotrunner/CMakeFiles/TestRobotrunner.dir/test/test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/TestRobotrunner.dir/test/test.cpp.s"
	cd /home/dragon/桌面/bishe/hexpod_ljl/build/Part_Robotrunner && /usr/local/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dragon/桌面/bishe/hexpod_ljl/Part_Robotrunner/test/test.cpp -o CMakeFiles/TestRobotrunner.dir/test/test.cpp.s

# Object files for target TestRobotrunner
TestRobotrunner_OBJECTS = \
"CMakeFiles/TestRobotrunner.dir/test/test.cpp.o"

# External object files for target TestRobotrunner
TestRobotrunner_EXTERNAL_OBJECTS =

bin/TestRobotrunner: Part_Robotrunner/CMakeFiles/TestRobotrunner.dir/test/test.cpp.o
bin/TestRobotrunner: Part_Robotrunner/CMakeFiles/TestRobotrunner.dir/build.make
bin/TestRobotrunner: ../lib/libRobotrunner.so
bin/TestRobotrunner: ../lib/libHardwareCom.so
bin/TestRobotrunner: ../lib/libPeriod.so
bin/TestRobotrunner: Part_Robotrunner/CMakeFiles/TestRobotrunner.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/dragon/桌面/bishe/hexpod_ljl/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../bin/TestRobotrunner"
	cd /home/dragon/桌面/bishe/hexpod_ljl/build/Part_Robotrunner && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/TestRobotrunner.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Part_Robotrunner/CMakeFiles/TestRobotrunner.dir/build: bin/TestRobotrunner
.PHONY : Part_Robotrunner/CMakeFiles/TestRobotrunner.dir/build

Part_Robotrunner/CMakeFiles/TestRobotrunner.dir/clean:
	cd /home/dragon/桌面/bishe/hexpod_ljl/build/Part_Robotrunner && $(CMAKE_COMMAND) -P CMakeFiles/TestRobotrunner.dir/cmake_clean.cmake
.PHONY : Part_Robotrunner/CMakeFiles/TestRobotrunner.dir/clean

Part_Robotrunner/CMakeFiles/TestRobotrunner.dir/depend:
	cd /home/dragon/桌面/bishe/hexpod_ljl/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dragon/桌面/bishe/hexpod_ljl /home/dragon/桌面/bishe/hexpod_ljl/Part_Robotrunner /home/dragon/桌面/bishe/hexpod_ljl/build /home/dragon/桌面/bishe/hexpod_ljl/build/Part_Robotrunner /home/dragon/桌面/bishe/hexpod_ljl/build/Part_Robotrunner/CMakeFiles/TestRobotrunner.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Part_Robotrunner/CMakeFiles/TestRobotrunner.dir/depend

