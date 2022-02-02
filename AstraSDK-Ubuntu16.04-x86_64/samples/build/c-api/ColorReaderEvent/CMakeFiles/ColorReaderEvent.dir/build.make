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
include c-api/ColorReaderEvent/CMakeFiles/ColorReaderEvent.dir/depend.make

# Include the progress variables for this target.
include c-api/ColorReaderEvent/CMakeFiles/ColorReaderEvent.dir/progress.make

# Include the compile flags for this target's objects.
include c-api/ColorReaderEvent/CMakeFiles/ColorReaderEvent.dir/flags.make

c-api/ColorReaderEvent/CMakeFiles/ColorReaderEvent.dir/main.c.o: c-api/ColorReaderEvent/CMakeFiles/ColorReaderEvent.dir/flags.make
c-api/ColorReaderEvent/CMakeFiles/ColorReaderEvent.dir/main.c.o: ../c-api/ColorReaderEvent/main.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zh/real-time-imitation/AstraSDK-Ubuntu16.04-x86_64/samples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object c-api/ColorReaderEvent/CMakeFiles/ColorReaderEvent.dir/main.c.o"
	cd /home/zh/real-time-imitation/AstraSDK-Ubuntu16.04-x86_64/samples/build/c-api/ColorReaderEvent && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/ColorReaderEvent.dir/main.c.o   -c /home/zh/real-time-imitation/AstraSDK-Ubuntu16.04-x86_64/samples/c-api/ColorReaderEvent/main.c

c-api/ColorReaderEvent/CMakeFiles/ColorReaderEvent.dir/main.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/ColorReaderEvent.dir/main.c.i"
	cd /home/zh/real-time-imitation/AstraSDK-Ubuntu16.04-x86_64/samples/build/c-api/ColorReaderEvent && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/zh/real-time-imitation/AstraSDK-Ubuntu16.04-x86_64/samples/c-api/ColorReaderEvent/main.c > CMakeFiles/ColorReaderEvent.dir/main.c.i

c-api/ColorReaderEvent/CMakeFiles/ColorReaderEvent.dir/main.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/ColorReaderEvent.dir/main.c.s"
	cd /home/zh/real-time-imitation/AstraSDK-Ubuntu16.04-x86_64/samples/build/c-api/ColorReaderEvent && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/zh/real-time-imitation/AstraSDK-Ubuntu16.04-x86_64/samples/c-api/ColorReaderEvent/main.c -o CMakeFiles/ColorReaderEvent.dir/main.c.s

# Object files for target ColorReaderEvent
ColorReaderEvent_OBJECTS = \
"CMakeFiles/ColorReaderEvent.dir/main.c.o"

# External object files for target ColorReaderEvent
ColorReaderEvent_EXTERNAL_OBJECTS =

bin/ColorReaderEvent: c-api/ColorReaderEvent/CMakeFiles/ColorReaderEvent.dir/main.c.o
bin/ColorReaderEvent: c-api/ColorReaderEvent/CMakeFiles/ColorReaderEvent.dir/build.make
bin/ColorReaderEvent: /home/zh/real-time-imitation/AstraSDK-Ubuntu16.04-x86_64/lib/libastra_core.so
bin/ColorReaderEvent: /home/zh/real-time-imitation/AstraSDK-Ubuntu16.04-x86_64/lib/libastra_core_api.so
bin/ColorReaderEvent: /home/zh/real-time-imitation/AstraSDK-Ubuntu16.04-x86_64/lib/libastra.so
bin/ColorReaderEvent: c-api/ColorReaderEvent/CMakeFiles/ColorReaderEvent.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zh/real-time-imitation/AstraSDK-Ubuntu16.04-x86_64/samples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C executable ../../bin/ColorReaderEvent"
	cd /home/zh/real-time-imitation/AstraSDK-Ubuntu16.04-x86_64/samples/build/c-api/ColorReaderEvent && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ColorReaderEvent.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
c-api/ColorReaderEvent/CMakeFiles/ColorReaderEvent.dir/build: bin/ColorReaderEvent

.PHONY : c-api/ColorReaderEvent/CMakeFiles/ColorReaderEvent.dir/build

c-api/ColorReaderEvent/CMakeFiles/ColorReaderEvent.dir/clean:
	cd /home/zh/real-time-imitation/AstraSDK-Ubuntu16.04-x86_64/samples/build/c-api/ColorReaderEvent && $(CMAKE_COMMAND) -P CMakeFiles/ColorReaderEvent.dir/cmake_clean.cmake
.PHONY : c-api/ColorReaderEvent/CMakeFiles/ColorReaderEvent.dir/clean

c-api/ColorReaderEvent/CMakeFiles/ColorReaderEvent.dir/depend:
	cd /home/zh/real-time-imitation/AstraSDK-Ubuntu16.04-x86_64/samples/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zh/real-time-imitation/AstraSDK-Ubuntu16.04-x86_64/samples /home/zh/real-time-imitation/AstraSDK-Ubuntu16.04-x86_64/samples/c-api/ColorReaderEvent /home/zh/real-time-imitation/AstraSDK-Ubuntu16.04-x86_64/samples/build /home/zh/real-time-imitation/AstraSDK-Ubuntu16.04-x86_64/samples/build/c-api/ColorReaderEvent /home/zh/real-time-imitation/AstraSDK-Ubuntu16.04-x86_64/samples/build/c-api/ColorReaderEvent/CMakeFiles/ColorReaderEvent.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : c-api/ColorReaderEvent/CMakeFiles/ColorReaderEvent.dir/depend
