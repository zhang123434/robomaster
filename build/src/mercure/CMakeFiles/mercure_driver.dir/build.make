# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.26

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
CMAKE_COMMAND = /usr/anaconda3/lib/python3.8/site-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /usr/anaconda3/lib/python3.8/site-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /media/zzh/Data/infantry_current_6.7

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /media/zzh/Data/infantry_current_6.7/build

# Include any dependencies generated for this target.
include src/mercure/CMakeFiles/mercure_driver.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include src/mercure/CMakeFiles/mercure_driver.dir/compiler_depend.make

# Include the progress variables for this target.
include src/mercure/CMakeFiles/mercure_driver.dir/progress.make

# Include the compile flags for this target's objects.
include src/mercure/CMakeFiles/mercure_driver.dir/flags.make

src/mercure/CMakeFiles/mercure_driver.dir/mercure_driver.cpp.o: src/mercure/CMakeFiles/mercure_driver.dir/flags.make
src/mercure/CMakeFiles/mercure_driver.dir/mercure_driver.cpp.o: /media/zzh/Data/infantry_current_6.7/src/mercure/mercure_driver.cpp
src/mercure/CMakeFiles/mercure_driver.dir/mercure_driver.cpp.o: src/mercure/CMakeFiles/mercure_driver.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/zzh/Data/infantry_current_6.7/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/mercure/CMakeFiles/mercure_driver.dir/mercure_driver.cpp.o"
	cd /media/zzh/Data/infantry_current_6.7/build/src/mercure && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/mercure/CMakeFiles/mercure_driver.dir/mercure_driver.cpp.o -MF CMakeFiles/mercure_driver.dir/mercure_driver.cpp.o.d -o CMakeFiles/mercure_driver.dir/mercure_driver.cpp.o -c /media/zzh/Data/infantry_current_6.7/src/mercure/mercure_driver.cpp

src/mercure/CMakeFiles/mercure_driver.dir/mercure_driver.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mercure_driver.dir/mercure_driver.cpp.i"
	cd /media/zzh/Data/infantry_current_6.7/build/src/mercure && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/zzh/Data/infantry_current_6.7/src/mercure/mercure_driver.cpp > CMakeFiles/mercure_driver.dir/mercure_driver.cpp.i

src/mercure/CMakeFiles/mercure_driver.dir/mercure_driver.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mercure_driver.dir/mercure_driver.cpp.s"
	cd /media/zzh/Data/infantry_current_6.7/build/src/mercure && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/zzh/Data/infantry_current_6.7/src/mercure/mercure_driver.cpp -o CMakeFiles/mercure_driver.dir/mercure_driver.cpp.s

# Object files for target mercure_driver
mercure_driver_OBJECTS = \
"CMakeFiles/mercure_driver.dir/mercure_driver.cpp.o"

# External object files for target mercure_driver
mercure_driver_EXTERNAL_OBJECTS =

src/mercure/libmercure_driver.a: src/mercure/CMakeFiles/mercure_driver.dir/mercure_driver.cpp.o
src/mercure/libmercure_driver.a: src/mercure/CMakeFiles/mercure_driver.dir/build.make
src/mercure/libmercure_driver.a: src/mercure/CMakeFiles/mercure_driver.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/media/zzh/Data/infantry_current_6.7/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libmercure_driver.a"
	cd /media/zzh/Data/infantry_current_6.7/build/src/mercure && $(CMAKE_COMMAND) -P CMakeFiles/mercure_driver.dir/cmake_clean_target.cmake
	cd /media/zzh/Data/infantry_current_6.7/build/src/mercure && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mercure_driver.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/mercure/CMakeFiles/mercure_driver.dir/build: src/mercure/libmercure_driver.a
.PHONY : src/mercure/CMakeFiles/mercure_driver.dir/build

src/mercure/CMakeFiles/mercure_driver.dir/clean:
	cd /media/zzh/Data/infantry_current_6.7/build/src/mercure && $(CMAKE_COMMAND) -P CMakeFiles/mercure_driver.dir/cmake_clean.cmake
.PHONY : src/mercure/CMakeFiles/mercure_driver.dir/clean

src/mercure/CMakeFiles/mercure_driver.dir/depend:
	cd /media/zzh/Data/infantry_current_6.7/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /media/zzh/Data/infantry_current_6.7 /media/zzh/Data/infantry_current_6.7/src/mercure /media/zzh/Data/infantry_current_6.7/build /media/zzh/Data/infantry_current_6.7/build/src/mercure /media/zzh/Data/infantry_current_6.7/build/src/mercure/CMakeFiles/mercure_driver.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/mercure/CMakeFiles/mercure_driver.dir/depend

