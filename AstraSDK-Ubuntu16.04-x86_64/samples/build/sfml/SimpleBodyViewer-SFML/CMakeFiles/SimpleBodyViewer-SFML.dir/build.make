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
CMAKE_SOURCE_DIR = /home/zh/real-time-imitation/AstraSDK-Ubuntu16.04-x86_64/samples

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zh/real-time-imitation/AstraSDK-Ubuntu16.04-x86_64/samples/build

# Include any dependencies generated for this target.
include sfml/SimpleBodyViewer-SFML/CMakeFiles/SimpleBodyViewer-SFML.dir/depend.make

# Include the progress variables for this target.
include sfml/SimpleBodyViewer-SFML/CMakeFiles/SimpleBodyViewer-SFML.dir/progress.make

# Include the compile flags for this target's objects.
include sfml/SimpleBodyViewer-SFML/CMakeFiles/SimpleBodyViewer-SFML.dir/flags.make

sfml/SimpleBodyViewer-SFML/CMakeFiles/SimpleBodyViewer-SFML.dir/main.cpp.o: sfml/SimpleBodyViewer-SFML/CMakeFiles/SimpleBodyViewer-SFML.dir/flags.make
sfml/SimpleBodyViewer-SFML/CMakeFiles/SimpleBodyViewer-SFML.dir/main.cpp.o: ../sfml/SimpleBodyViewer-SFML/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zh/real-time-imitation/AstraSDK-Ubuntu16.04-x86_64/samples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object sfml/SimpleBodyViewer-SFML/CMakeFiles/SimpleBodyViewer-SFML.dir/main.cpp.o"
	cd /home/zh/real-time-imitation/AstraSDK-Ubuntu16.04-x86_64/samples/build/sfml/SimpleBodyViewer-SFML && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/SimpleBodyViewer-SFML.dir/main.cpp.o -c /home/zh/real-time-imitation/AstraSDK-Ubuntu16.04-x86_64/samples/sfml/SimpleBodyViewer-SFML/main.cpp

sfml/SimpleBodyViewer-SFML/CMakeFiles/SimpleBodyViewer-SFML.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SimpleBodyViewer-SFML.dir/main.cpp.i"
	cd /home/zh/real-time-imitation/AstraSDK-Ubuntu16.04-x86_64/samples/build/sfml/SimpleBodyViewer-SFML && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zh/real-time-imitation/AstraSDK-Ubuntu16.04-x86_64/samples/sfml/SimpleBodyViewer-SFML/main.cpp > CMakeFiles/SimpleBodyViewer-SFML.dir/main.cpp.i

sfml/SimpleBodyViewer-SFML/CMakeFiles/SimpleBodyViewer-SFML.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SimpleBodyViewer-SFML.dir/main.cpp.s"
	cd /home/zh/real-time-imitation/AstraSDK-Ubuntu16.04-x86_64/samples/build/sfml/SimpleBodyViewer-SFML && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zh/real-time-imitation/AstraSDK-Ubuntu16.04-x86_64/samples/sfml/SimpleBodyViewer-SFML/main.cpp -o CMakeFiles/SimpleBodyViewer-SFML.dir/main.cpp.s

# Object files for target SimpleBodyViewer-SFML
SimpleBodyViewer__SFML_OBJECTS = \
"CMakeFiles/SimpleBodyViewer-SFML.dir/main.cpp.o"

# External object files for target SimpleBodyViewer-SFML
SimpleBodyViewer__SFML_EXTERNAL_OBJECTS =

bin/SimpleBodyViewer-SFML: sfml/SimpleBodyViewer-SFML/CMakeFiles/SimpleBodyViewer-SFML.dir/main.cpp.o
bin/SimpleBodyViewer-SFML: sfml/SimpleBodyViewer-SFML/CMakeFiles/SimpleBodyViewer-SFML.dir/build.make
bin/SimpleBodyViewer-SFML: /home/zh/real-time-imitation/AstraSDK-Ubuntu16.04-x86_64/lib/libastra_core.so
bin/SimpleBodyViewer-SFML: /home/zh/real-time-imitation/AstraSDK-Ubuntu16.04-x86_64/lib/libastra_core_api.so
bin/SimpleBodyViewer-SFML: /home/zh/real-time-imitation/AstraSDK-Ubuntu16.04-x86_64/lib/libastra.so
bin/SimpleBodyViewer-SFML: /usr/lib/x86_64-linux-gnu/libsfml-graphics.so
bin/SimpleBodyViewer-SFML: /usr/lib/x86_64-linux-gnu/libsfml-window.so
bin/SimpleBodyViewer-SFML: /usr/lib/x86_64-linux-gnu/libsfml-system.so
bin/SimpleBodyViewer-SFML: /usr/lib/x86_64-linux-gnu/libsfml-graphics.so
bin/SimpleBodyViewer-SFML: /usr/lib/x86_64-linux-gnu/libsfml-window.so
bin/SimpleBodyViewer-SFML: /usr/lib/x86_64-linux-gnu/libsfml-system.so
bin/SimpleBodyViewer-SFML: sfml/SimpleBodyViewer-SFML/CMakeFiles/SimpleBodyViewer-SFML.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zh/real-time-imitation/AstraSDK-Ubuntu16.04-x86_64/samples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/SimpleBodyViewer-SFML"
	cd /home/zh/real-time-imitation/AstraSDK-Ubuntu16.04-x86_64/samples/build/sfml/SimpleBodyViewer-SFML && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/SimpleBodyViewer-SFML.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
sfml/SimpleBodyViewer-SFML/CMakeFiles/SimpleBodyViewer-SFML.dir/build: bin/SimpleBodyViewer-SFML

.PHONY : sfml/SimpleBodyViewer-SFML/CMakeFiles/SimpleBodyViewer-SFML.dir/build

sfml/SimpleBodyViewer-SFML/CMakeFiles/SimpleBodyViewer-SFML.dir/clean:
	cd /home/zh/real-time-imitation/AstraSDK-Ubuntu16.04-x86_64/samples/build/sfml/SimpleBodyViewer-SFML && $(CMAKE_COMMAND) -P CMakeFiles/SimpleBodyViewer-SFML.dir/cmake_clean.cmake
.PHONY : sfml/SimpleBodyViewer-SFML/CMakeFiles/SimpleBodyViewer-SFML.dir/clean

sfml/SimpleBodyViewer-SFML/CMakeFiles/SimpleBodyViewer-SFML.dir/depend:
	cd /home/zh/real-time-imitation/AstraSDK-Ubuntu16.04-x86_64/samples/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zh/real-time-imitation/AstraSDK-Ubuntu16.04-x86_64/samples /home/zh/real-time-imitation/AstraSDK-Ubuntu16.04-x86_64/samples/sfml/SimpleBodyViewer-SFML /home/zh/real-time-imitation/AstraSDK-Ubuntu16.04-x86_64/samples/build /home/zh/real-time-imitation/AstraSDK-Ubuntu16.04-x86_64/samples/build/sfml/SimpleBodyViewer-SFML /home/zh/real-time-imitation/AstraSDK-Ubuntu16.04-x86_64/samples/build/sfml/SimpleBodyViewer-SFML/CMakeFiles/SimpleBodyViewer-SFML.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : sfml/SimpleBodyViewer-SFML/CMakeFiles/SimpleBodyViewer-SFML.dir/depend

