# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.17

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
CMAKE_COMMAND = /usr/cmake-3.17.3-Linux-x86_64/bin/cmake

# The command to remove a file.
RM = /usr/cmake-3.17.3-Linux-x86_64/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/zh/orbbec/AstraSDK-v2.1.3-94bca0f52e-20210608T055717Z-Ubuntu16.04-x86_64/samples

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zh/orbbec/AstraSDK-v2.1.3-94bca0f52e-20210608T055717Z-Ubuntu16.04-x86_64/samples/build

# Include any dependencies generated for this target.
include sfml/SimpleStreamViewer-SFML/CMakeFiles/SimpleStreamViewer-SFML.dir/depend.make

# Include the progress variables for this target.
include sfml/SimpleStreamViewer-SFML/CMakeFiles/SimpleStreamViewer-SFML.dir/progress.make

# Include the compile flags for this target's objects.
include sfml/SimpleStreamViewer-SFML/CMakeFiles/SimpleStreamViewer-SFML.dir/flags.make

sfml/SimpleStreamViewer-SFML/CMakeFiles/SimpleStreamViewer-SFML.dir/main.cpp.o: sfml/SimpleStreamViewer-SFML/CMakeFiles/SimpleStreamViewer-SFML.dir/flags.make
sfml/SimpleStreamViewer-SFML/CMakeFiles/SimpleStreamViewer-SFML.dir/main.cpp.o: ../sfml/SimpleStreamViewer-SFML/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zh/orbbec/AstraSDK-v2.1.3-94bca0f52e-20210608T055717Z-Ubuntu16.04-x86_64/samples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object sfml/SimpleStreamViewer-SFML/CMakeFiles/SimpleStreamViewer-SFML.dir/main.cpp.o"
	cd /home/zh/orbbec/AstraSDK-v2.1.3-94bca0f52e-20210608T055717Z-Ubuntu16.04-x86_64/samples/build/sfml/SimpleStreamViewer-SFML && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/SimpleStreamViewer-SFML.dir/main.cpp.o -c /home/zh/orbbec/AstraSDK-v2.1.3-94bca0f52e-20210608T055717Z-Ubuntu16.04-x86_64/samples/sfml/SimpleStreamViewer-SFML/main.cpp

sfml/SimpleStreamViewer-SFML/CMakeFiles/SimpleStreamViewer-SFML.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SimpleStreamViewer-SFML.dir/main.cpp.i"
	cd /home/zh/orbbec/AstraSDK-v2.1.3-94bca0f52e-20210608T055717Z-Ubuntu16.04-x86_64/samples/build/sfml/SimpleStreamViewer-SFML && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zh/orbbec/AstraSDK-v2.1.3-94bca0f52e-20210608T055717Z-Ubuntu16.04-x86_64/samples/sfml/SimpleStreamViewer-SFML/main.cpp > CMakeFiles/SimpleStreamViewer-SFML.dir/main.cpp.i

sfml/SimpleStreamViewer-SFML/CMakeFiles/SimpleStreamViewer-SFML.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SimpleStreamViewer-SFML.dir/main.cpp.s"
	cd /home/zh/orbbec/AstraSDK-v2.1.3-94bca0f52e-20210608T055717Z-Ubuntu16.04-x86_64/samples/build/sfml/SimpleStreamViewer-SFML && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zh/orbbec/AstraSDK-v2.1.3-94bca0f52e-20210608T055717Z-Ubuntu16.04-x86_64/samples/sfml/SimpleStreamViewer-SFML/main.cpp -o CMakeFiles/SimpleStreamViewer-SFML.dir/main.cpp.s

# Object files for target SimpleStreamViewer-SFML
SimpleStreamViewer__SFML_OBJECTS = \
"CMakeFiles/SimpleStreamViewer-SFML.dir/main.cpp.o"

# External object files for target SimpleStreamViewer-SFML
SimpleStreamViewer__SFML_EXTERNAL_OBJECTS =

bin/SimpleStreamViewer-SFML: sfml/SimpleStreamViewer-SFML/CMakeFiles/SimpleStreamViewer-SFML.dir/main.cpp.o
bin/SimpleStreamViewer-SFML: sfml/SimpleStreamViewer-SFML/CMakeFiles/SimpleStreamViewer-SFML.dir/build.make
bin/SimpleStreamViewer-SFML: /home/zh/orbbec/AstraSDK-v2.1.3-94bca0f52e-20210608T055717Z-Ubuntu16.04-x86_64/lib/libastra_core.so
bin/SimpleStreamViewer-SFML: /home/zh/orbbec/AstraSDK-v2.1.3-94bca0f52e-20210608T055717Z-Ubuntu16.04-x86_64/lib/libastra_core_api.so
bin/SimpleStreamViewer-SFML: /home/zh/orbbec/AstraSDK-v2.1.3-94bca0f52e-20210608T055717Z-Ubuntu16.04-x86_64/lib/libastra.so
bin/SimpleStreamViewer-SFML: /usr/lib/x86_64-linux-gnu/libsfml-graphics.so
bin/SimpleStreamViewer-SFML: /usr/lib/x86_64-linux-gnu/libsfml-window.so
bin/SimpleStreamViewer-SFML: /usr/lib/x86_64-linux-gnu/libsfml-system.so
bin/SimpleStreamViewer-SFML: /usr/lib/x86_64-linux-gnu/libsfml-graphics.so
bin/SimpleStreamViewer-SFML: /usr/lib/x86_64-linux-gnu/libsfml-window.so
bin/SimpleStreamViewer-SFML: /usr/lib/x86_64-linux-gnu/libsfml-system.so
bin/SimpleStreamViewer-SFML: sfml/SimpleStreamViewer-SFML/CMakeFiles/SimpleStreamViewer-SFML.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zh/orbbec/AstraSDK-v2.1.3-94bca0f52e-20210608T055717Z-Ubuntu16.04-x86_64/samples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/SimpleStreamViewer-SFML"
	cd /home/zh/orbbec/AstraSDK-v2.1.3-94bca0f52e-20210608T055717Z-Ubuntu16.04-x86_64/samples/build/sfml/SimpleStreamViewer-SFML && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/SimpleStreamViewer-SFML.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
sfml/SimpleStreamViewer-SFML/CMakeFiles/SimpleStreamViewer-SFML.dir/build: bin/SimpleStreamViewer-SFML

.PHONY : sfml/SimpleStreamViewer-SFML/CMakeFiles/SimpleStreamViewer-SFML.dir/build

sfml/SimpleStreamViewer-SFML/CMakeFiles/SimpleStreamViewer-SFML.dir/clean:
	cd /home/zh/orbbec/AstraSDK-v2.1.3-94bca0f52e-20210608T055717Z-Ubuntu16.04-x86_64/samples/build/sfml/SimpleStreamViewer-SFML && $(CMAKE_COMMAND) -P CMakeFiles/SimpleStreamViewer-SFML.dir/cmake_clean.cmake
.PHONY : sfml/SimpleStreamViewer-SFML/CMakeFiles/SimpleStreamViewer-SFML.dir/clean

sfml/SimpleStreamViewer-SFML/CMakeFiles/SimpleStreamViewer-SFML.dir/depend:
	cd /home/zh/orbbec/AstraSDK-v2.1.3-94bca0f52e-20210608T055717Z-Ubuntu16.04-x86_64/samples/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zh/orbbec/AstraSDK-v2.1.3-94bca0f52e-20210608T055717Z-Ubuntu16.04-x86_64/samples /home/zh/orbbec/AstraSDK-v2.1.3-94bca0f52e-20210608T055717Z-Ubuntu16.04-x86_64/samples/sfml/SimpleStreamViewer-SFML /home/zh/orbbec/AstraSDK-v2.1.3-94bca0f52e-20210608T055717Z-Ubuntu16.04-x86_64/samples/build /home/zh/orbbec/AstraSDK-v2.1.3-94bca0f52e-20210608T055717Z-Ubuntu16.04-x86_64/samples/build/sfml/SimpleStreamViewer-SFML /home/zh/orbbec/AstraSDK-v2.1.3-94bca0f52e-20210608T055717Z-Ubuntu16.04-x86_64/samples/build/sfml/SimpleStreamViewer-SFML/CMakeFiles/SimpleStreamViewer-SFML.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : sfml/SimpleStreamViewer-SFML/CMakeFiles/SimpleStreamViewer-SFML.dir/depend

