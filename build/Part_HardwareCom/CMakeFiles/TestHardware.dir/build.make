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
include Part_HardwareCom/CMakeFiles/TestHardware.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include Part_HardwareCom/CMakeFiles/TestHardware.dir/compiler_depend.make

# Include the progress variables for this target.
include Part_HardwareCom/CMakeFiles/TestHardware.dir/progress.make

# Include the compile flags for this target's objects.
include Part_HardwareCom/CMakeFiles/TestHardware.dir/flags.make

Part_HardwareCom/CMakeFiles/TestHardware.dir/test/test.cpp.o: Part_HardwareCom/CMakeFiles/TestHardware.dir/flags.make
Part_HardwareCom/CMakeFiles/TestHardware.dir/test/test.cpp.o: ../Part_HardwareCom/test/test.cpp
Part_HardwareCom/CMakeFiles/TestHardware.dir/test/test.cpp.o: Part_HardwareCom/CMakeFiles/TestHardware.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dragon/桌面/bishe/hexpod_ljl/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object Part_HardwareCom/CMakeFiles/TestHardware.dir/test/test.cpp.o"
	cd /home/dragon/桌面/bishe/hexpod_ljl/build/Part_HardwareCom && /usr/local/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT Part_HardwareCom/CMakeFiles/TestHardware.dir/test/test.cpp.o -MF CMakeFiles/TestHardware.dir/test/test.cpp.o.d -o CMakeFiles/TestHardware.dir/test/test.cpp.o -c /home/dragon/桌面/bishe/hexpod_ljl/Part_HardwareCom/test/test.cpp

Part_HardwareCom/CMakeFiles/TestHardware.dir/test/test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/TestHardware.dir/test/test.cpp.i"
	cd /home/dragon/桌面/bishe/hexpod_ljl/build/Part_HardwareCom && /usr/local/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dragon/桌面/bishe/hexpod_ljl/Part_HardwareCom/test/test.cpp > CMakeFiles/TestHardware.dir/test/test.cpp.i

Part_HardwareCom/CMakeFiles/TestHardware.dir/test/test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/TestHardware.dir/test/test.cpp.s"
	cd /home/dragon/桌面/bishe/hexpod_ljl/build/Part_HardwareCom && /usr/local/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dragon/桌面/bishe/hexpod_ljl/Part_HardwareCom/test/test.cpp -o CMakeFiles/TestHardware.dir/test/test.cpp.s

# Object files for target TestHardware
TestHardware_OBJECTS = \
"CMakeFiles/TestHardware.dir/test/test.cpp.o"

# External object files for target TestHardware
TestHardware_EXTERNAL_OBJECTS =

bin/TestHardware: Part_HardwareCom/CMakeFiles/TestHardware.dir/test/test.cpp.o
bin/TestHardware: Part_HardwareCom/CMakeFiles/TestHardware.dir/build.make
bin/TestHardware: ../lib/libHardwareCom.so
bin/TestHardware: ../lib/libPeriod.so
bin/TestHardware: Part_HardwareCom/CMakeFiles/TestHardware.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/dragon/桌面/bishe/hexpod_ljl/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../bin/TestHardware"
	cd /home/dragon/桌面/bishe/hexpod_ljl/build/Part_HardwareCom && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/TestHardware.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Part_HardwareCom/CMakeFiles/TestHardware.dir/build: bin/TestHardware
.PHONY : Part_HardwareCom/CMakeFiles/TestHardware.dir/build

Part_HardwareCom/CMakeFiles/TestHardware.dir/clean:
	cd /home/dragon/桌面/bishe/hexpod_ljl/build/Part_HardwareCom && $(CMAKE_COMMAND) -P CMakeFiles/TestHardware.dir/cmake_clean.cmake
.PHONY : Part_HardwareCom/CMakeFiles/TestHardware.dir/clean

Part_HardwareCom/CMakeFiles/TestHardware.dir/depend:
	cd /home/dragon/桌面/bishe/hexpod_ljl/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dragon/桌面/bishe/hexpod_ljl /home/dragon/桌面/bishe/hexpod_ljl/Part_HardwareCom /home/dragon/桌面/bishe/hexpod_ljl/build /home/dragon/桌面/bishe/hexpod_ljl/build/Part_HardwareCom /home/dragon/桌面/bishe/hexpod_ljl/build/Part_HardwareCom/CMakeFiles/TestHardware.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Part_HardwareCom/CMakeFiles/TestHardware.dir/depend

