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
CMAKE_SOURCE_DIR = /home/dragon/桌面/bishe/111/ljl_hexpod

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/dragon/桌面/bishe/111/ljl_hexpod/build

# Include any dependencies generated for this target.
include CMakeFiles/code_struct.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/code_struct.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/code_struct.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/code_struct.dir/flags.make

CMakeFiles/code_struct.dir/main.cpp.o: CMakeFiles/code_struct.dir/flags.make
CMakeFiles/code_struct.dir/main.cpp.o: ../main.cpp
CMakeFiles/code_struct.dir/main.cpp.o: CMakeFiles/code_struct.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dragon/桌面/bishe/111/ljl_hexpod/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/code_struct.dir/main.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/code_struct.dir/main.cpp.o -MF CMakeFiles/code_struct.dir/main.cpp.o.d -o CMakeFiles/code_struct.dir/main.cpp.o -c /home/dragon/桌面/bishe/111/ljl_hexpod/main.cpp

CMakeFiles/code_struct.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/code_struct.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dragon/桌面/bishe/111/ljl_hexpod/main.cpp > CMakeFiles/code_struct.dir/main.cpp.i

CMakeFiles/code_struct.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/code_struct.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dragon/桌面/bishe/111/ljl_hexpod/main.cpp -o CMakeFiles/code_struct.dir/main.cpp.s

# Object files for target code_struct
code_struct_OBJECTS = \
"CMakeFiles/code_struct.dir/main.cpp.o"

# External object files for target code_struct
code_struct_EXTERNAL_OBJECTS =

bin/code_struct: CMakeFiles/code_struct.dir/main.cpp.o
bin/code_struct: CMakeFiles/code_struct.dir/build.make
bin/code_struct: ../lib/libHardwareCom.so
bin/code_struct: ../lib/libRobotrunner.so
bin/code_struct: /home/dragon/Qt/6.2.4/gcc_64/lib/libQt6Widgets.so.6.2.4
bin/code_struct: /home/dragon/Qt/6.2.4/gcc_64/lib/cmake/Qt6PrintSupport/../../libQt6PrintSupport.so.6
bin/code_struct: ../lib/libRealtimeCurvePlot.so
bin/code_struct: ../lib/libPeriod.so
bin/code_struct: /home/dragon/Qt/6.2.4/gcc_64/lib/libQt6Gui.so.6.2.4
bin/code_struct: /home/dragon/Qt/6.2.4/gcc_64/lib/libQt6Core.so.6.2.4
bin/code_struct: /usr/lib/x86_64-linux-gnu/libGLX.so
bin/code_struct: /usr/lib/x86_64-linux-gnu/libOpenGL.so
bin/code_struct: CMakeFiles/code_struct.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/dragon/桌面/bishe/111/ljl_hexpod/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable bin/code_struct"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/code_struct.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/code_struct.dir/build: bin/code_struct
.PHONY : CMakeFiles/code_struct.dir/build

CMakeFiles/code_struct.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/code_struct.dir/cmake_clean.cmake
.PHONY : CMakeFiles/code_struct.dir/clean

CMakeFiles/code_struct.dir/depend:
	cd /home/dragon/桌面/bishe/111/ljl_hexpod/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dragon/桌面/bishe/111/ljl_hexpod /home/dragon/桌面/bishe/111/ljl_hexpod /home/dragon/桌面/bishe/111/ljl_hexpod/build /home/dragon/桌面/bishe/111/ljl_hexpod/build /home/dragon/桌面/bishe/111/ljl_hexpod/build/CMakeFiles/code_struct.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/code_struct.dir/depend

