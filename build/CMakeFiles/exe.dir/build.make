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
include CMakeFiles/exe.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/exe.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/exe.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/exe.dir/flags.make

CMakeFiles/exe.dir/src/Antitop/SpinDetector.cpp.o: CMakeFiles/exe.dir/flags.make
CMakeFiles/exe.dir/src/Antitop/SpinDetector.cpp.o: /media/zzh/Data/infantry_current_6.7/src/Antitop/SpinDetector.cpp
CMakeFiles/exe.dir/src/Antitop/SpinDetector.cpp.o: CMakeFiles/exe.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/zzh/Data/infantry_current_6.7/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/exe.dir/src/Antitop/SpinDetector.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/exe.dir/src/Antitop/SpinDetector.cpp.o -MF CMakeFiles/exe.dir/src/Antitop/SpinDetector.cpp.o.d -o CMakeFiles/exe.dir/src/Antitop/SpinDetector.cpp.o -c /media/zzh/Data/infantry_current_6.7/src/Antitop/SpinDetector.cpp

CMakeFiles/exe.dir/src/Antitop/SpinDetector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/exe.dir/src/Antitop/SpinDetector.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/zzh/Data/infantry_current_6.7/src/Antitop/SpinDetector.cpp > CMakeFiles/exe.dir/src/Antitop/SpinDetector.cpp.i

CMakeFiles/exe.dir/src/Antitop/SpinDetector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/exe.dir/src/Antitop/SpinDetector.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/zzh/Data/infantry_current_6.7/src/Antitop/SpinDetector.cpp -o CMakeFiles/exe.dir/src/Antitop/SpinDetector.cpp.s

CMakeFiles/exe.dir/src/Imageprocess/Imageprocess.cpp.o: CMakeFiles/exe.dir/flags.make
CMakeFiles/exe.dir/src/Imageprocess/Imageprocess.cpp.o: /media/zzh/Data/infantry_current_6.7/src/Imageprocess/Imageprocess.cpp
CMakeFiles/exe.dir/src/Imageprocess/Imageprocess.cpp.o: CMakeFiles/exe.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/zzh/Data/infantry_current_6.7/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/exe.dir/src/Imageprocess/Imageprocess.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/exe.dir/src/Imageprocess/Imageprocess.cpp.o -MF CMakeFiles/exe.dir/src/Imageprocess/Imageprocess.cpp.o.d -o CMakeFiles/exe.dir/src/Imageprocess/Imageprocess.cpp.o -c /media/zzh/Data/infantry_current_6.7/src/Imageprocess/Imageprocess.cpp

CMakeFiles/exe.dir/src/Imageprocess/Imageprocess.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/exe.dir/src/Imageprocess/Imageprocess.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/zzh/Data/infantry_current_6.7/src/Imageprocess/Imageprocess.cpp > CMakeFiles/exe.dir/src/Imageprocess/Imageprocess.cpp.i

CMakeFiles/exe.dir/src/Imageprocess/Imageprocess.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/exe.dir/src/Imageprocess/Imageprocess.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/zzh/Data/infantry_current_6.7/src/Imageprocess/Imageprocess.cpp -o CMakeFiles/exe.dir/src/Imageprocess/Imageprocess.cpp.s

CMakeFiles/exe.dir/src/Infantry-can/CanSerial.cpp.o: CMakeFiles/exe.dir/flags.make
CMakeFiles/exe.dir/src/Infantry-can/CanSerial.cpp.o: /media/zzh/Data/infantry_current_6.7/src/Infantry-can/CanSerial.cpp
CMakeFiles/exe.dir/src/Infantry-can/CanSerial.cpp.o: CMakeFiles/exe.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/zzh/Data/infantry_current_6.7/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/exe.dir/src/Infantry-can/CanSerial.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/exe.dir/src/Infantry-can/CanSerial.cpp.o -MF CMakeFiles/exe.dir/src/Infantry-can/CanSerial.cpp.o.d -o CMakeFiles/exe.dir/src/Infantry-can/CanSerial.cpp.o -c /media/zzh/Data/infantry_current_6.7/src/Infantry-can/CanSerial.cpp

CMakeFiles/exe.dir/src/Infantry-can/CanSerial.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/exe.dir/src/Infantry-can/CanSerial.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/zzh/Data/infantry_current_6.7/src/Infantry-can/CanSerial.cpp > CMakeFiles/exe.dir/src/Infantry-can/CanSerial.cpp.i

CMakeFiles/exe.dir/src/Infantry-can/CanSerial.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/exe.dir/src/Infantry-can/CanSerial.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/zzh/Data/infantry_current_6.7/src/Infantry-can/CanSerial.cpp -o CMakeFiles/exe.dir/src/Infantry-can/CanSerial.cpp.s

CMakeFiles/exe.dir/src/autoaim/autoaim1.cpp.o: CMakeFiles/exe.dir/flags.make
CMakeFiles/exe.dir/src/autoaim/autoaim1.cpp.o: /media/zzh/Data/infantry_current_6.7/src/autoaim/autoaim1.cpp
CMakeFiles/exe.dir/src/autoaim/autoaim1.cpp.o: CMakeFiles/exe.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/zzh/Data/infantry_current_6.7/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/exe.dir/src/autoaim/autoaim1.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/exe.dir/src/autoaim/autoaim1.cpp.o -MF CMakeFiles/exe.dir/src/autoaim/autoaim1.cpp.o.d -o CMakeFiles/exe.dir/src/autoaim/autoaim1.cpp.o -c /media/zzh/Data/infantry_current_6.7/src/autoaim/autoaim1.cpp

CMakeFiles/exe.dir/src/autoaim/autoaim1.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/exe.dir/src/autoaim/autoaim1.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/zzh/Data/infantry_current_6.7/src/autoaim/autoaim1.cpp > CMakeFiles/exe.dir/src/autoaim/autoaim1.cpp.i

CMakeFiles/exe.dir/src/autoaim/autoaim1.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/exe.dir/src/autoaim/autoaim1.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/zzh/Data/infantry_current_6.7/src/autoaim/autoaim1.cpp -o CMakeFiles/exe.dir/src/autoaim/autoaim1.cpp.s

CMakeFiles/exe.dir/src/detector/detector.cpp.o: CMakeFiles/exe.dir/flags.make
CMakeFiles/exe.dir/src/detector/detector.cpp.o: /media/zzh/Data/infantry_current_6.7/src/detector/detector.cpp
CMakeFiles/exe.dir/src/detector/detector.cpp.o: CMakeFiles/exe.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/zzh/Data/infantry_current_6.7/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/exe.dir/src/detector/detector.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/exe.dir/src/detector/detector.cpp.o -MF CMakeFiles/exe.dir/src/detector/detector.cpp.o.d -o CMakeFiles/exe.dir/src/detector/detector.cpp.o -c /media/zzh/Data/infantry_current_6.7/src/detector/detector.cpp

CMakeFiles/exe.dir/src/detector/detector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/exe.dir/src/detector/detector.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/zzh/Data/infantry_current_6.7/src/detector/detector.cpp > CMakeFiles/exe.dir/src/detector/detector.cpp.i

CMakeFiles/exe.dir/src/detector/detector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/exe.dir/src/detector/detector.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/zzh/Data/infantry_current_6.7/src/detector/detector.cpp -o CMakeFiles/exe.dir/src/detector/detector.cpp.s

CMakeFiles/exe.dir/src/digital_classifier/classfier.cpp.o: CMakeFiles/exe.dir/flags.make
CMakeFiles/exe.dir/src/digital_classifier/classfier.cpp.o: /media/zzh/Data/infantry_current_6.7/src/digital_classifier/classfier.cpp
CMakeFiles/exe.dir/src/digital_classifier/classfier.cpp.o: CMakeFiles/exe.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/zzh/Data/infantry_current_6.7/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/exe.dir/src/digital_classifier/classfier.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/exe.dir/src/digital_classifier/classfier.cpp.o -MF CMakeFiles/exe.dir/src/digital_classifier/classfier.cpp.o.d -o CMakeFiles/exe.dir/src/digital_classifier/classfier.cpp.o -c /media/zzh/Data/infantry_current_6.7/src/digital_classifier/classfier.cpp

CMakeFiles/exe.dir/src/digital_classifier/classfier.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/exe.dir/src/digital_classifier/classfier.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/zzh/Data/infantry_current_6.7/src/digital_classifier/classfier.cpp > CMakeFiles/exe.dir/src/digital_classifier/classfier.cpp.i

CMakeFiles/exe.dir/src/digital_classifier/classfier.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/exe.dir/src/digital_classifier/classfier.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/zzh/Data/infantry_current_6.7/src/digital_classifier/classfier.cpp -o CMakeFiles/exe.dir/src/digital_classifier/classfier.cpp.s

CMakeFiles/exe.dir/src/digital_classifier/classifier1.cpp.o: CMakeFiles/exe.dir/flags.make
CMakeFiles/exe.dir/src/digital_classifier/classifier1.cpp.o: /media/zzh/Data/infantry_current_6.7/src/digital_classifier/classifier1.cpp
CMakeFiles/exe.dir/src/digital_classifier/classifier1.cpp.o: CMakeFiles/exe.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/zzh/Data/infantry_current_6.7/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/exe.dir/src/digital_classifier/classifier1.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/exe.dir/src/digital_classifier/classifier1.cpp.o -MF CMakeFiles/exe.dir/src/digital_classifier/classifier1.cpp.o.d -o CMakeFiles/exe.dir/src/digital_classifier/classifier1.cpp.o -c /media/zzh/Data/infantry_current_6.7/src/digital_classifier/classifier1.cpp

CMakeFiles/exe.dir/src/digital_classifier/classifier1.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/exe.dir/src/digital_classifier/classifier1.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/zzh/Data/infantry_current_6.7/src/digital_classifier/classifier1.cpp > CMakeFiles/exe.dir/src/digital_classifier/classifier1.cpp.i

CMakeFiles/exe.dir/src/digital_classifier/classifier1.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/exe.dir/src/digital_classifier/classifier1.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/zzh/Data/infantry_current_6.7/src/digital_classifier/classifier1.cpp -o CMakeFiles/exe.dir/src/digital_classifier/classifier1.cpp.s

CMakeFiles/exe.dir/src/main.cpp.o: CMakeFiles/exe.dir/flags.make
CMakeFiles/exe.dir/src/main.cpp.o: /media/zzh/Data/infantry_current_6.7/src/main.cpp
CMakeFiles/exe.dir/src/main.cpp.o: CMakeFiles/exe.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/zzh/Data/infantry_current_6.7/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/exe.dir/src/main.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/exe.dir/src/main.cpp.o -MF CMakeFiles/exe.dir/src/main.cpp.o.d -o CMakeFiles/exe.dir/src/main.cpp.o -c /media/zzh/Data/infantry_current_6.7/src/main.cpp

CMakeFiles/exe.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/exe.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/zzh/Data/infantry_current_6.7/src/main.cpp > CMakeFiles/exe.dir/src/main.cpp.i

CMakeFiles/exe.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/exe.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/zzh/Data/infantry_current_6.7/src/main.cpp -o CMakeFiles/exe.dir/src/main.cpp.s

CMakeFiles/exe.dir/src/mercure/mercure_driver.cpp.o: CMakeFiles/exe.dir/flags.make
CMakeFiles/exe.dir/src/mercure/mercure_driver.cpp.o: /media/zzh/Data/infantry_current_6.7/src/mercure/mercure_driver.cpp
CMakeFiles/exe.dir/src/mercure/mercure_driver.cpp.o: CMakeFiles/exe.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/zzh/Data/infantry_current_6.7/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/exe.dir/src/mercure/mercure_driver.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/exe.dir/src/mercure/mercure_driver.cpp.o -MF CMakeFiles/exe.dir/src/mercure/mercure_driver.cpp.o.d -o CMakeFiles/exe.dir/src/mercure/mercure_driver.cpp.o -c /media/zzh/Data/infantry_current_6.7/src/mercure/mercure_driver.cpp

CMakeFiles/exe.dir/src/mercure/mercure_driver.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/exe.dir/src/mercure/mercure_driver.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/zzh/Data/infantry_current_6.7/src/mercure/mercure_driver.cpp > CMakeFiles/exe.dir/src/mercure/mercure_driver.cpp.i

CMakeFiles/exe.dir/src/mercure/mercure_driver.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/exe.dir/src/mercure/mercure_driver.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/zzh/Data/infantry_current_6.7/src/mercure/mercure_driver.cpp -o CMakeFiles/exe.dir/src/mercure/mercure_driver.cpp.s

# Object files for target exe
exe_OBJECTS = \
"CMakeFiles/exe.dir/src/Antitop/SpinDetector.cpp.o" \
"CMakeFiles/exe.dir/src/Imageprocess/Imageprocess.cpp.o" \
"CMakeFiles/exe.dir/src/Infantry-can/CanSerial.cpp.o" \
"CMakeFiles/exe.dir/src/autoaim/autoaim1.cpp.o" \
"CMakeFiles/exe.dir/src/detector/detector.cpp.o" \
"CMakeFiles/exe.dir/src/digital_classifier/classfier.cpp.o" \
"CMakeFiles/exe.dir/src/digital_classifier/classifier1.cpp.o" \
"CMakeFiles/exe.dir/src/main.cpp.o" \
"CMakeFiles/exe.dir/src/mercure/mercure_driver.cpp.o"

# External object files for target exe
exe_EXTERNAL_OBJECTS =

exe: CMakeFiles/exe.dir/src/Antitop/SpinDetector.cpp.o
exe: CMakeFiles/exe.dir/src/Imageprocess/Imageprocess.cpp.o
exe: CMakeFiles/exe.dir/src/Infantry-can/CanSerial.cpp.o
exe: CMakeFiles/exe.dir/src/autoaim/autoaim1.cpp.o
exe: CMakeFiles/exe.dir/src/detector/detector.cpp.o
exe: CMakeFiles/exe.dir/src/digital_classifier/classfier.cpp.o
exe: CMakeFiles/exe.dir/src/digital_classifier/classifier1.cpp.o
exe: CMakeFiles/exe.dir/src/main.cpp.o
exe: CMakeFiles/exe.dir/src/mercure/mercure_driver.cpp.o
exe: CMakeFiles/exe.dir/build.make
exe: src/mercure/libmercure_driver.a
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_img_hash.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: /usr/local/lib/libopencv_world.so.4.4.0
exe: CMakeFiles/exe.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/media/zzh/Data/infantry_current_6.7/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Linking CXX executable exe"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/exe.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/exe.dir/build: exe
.PHONY : CMakeFiles/exe.dir/build

CMakeFiles/exe.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/exe.dir/cmake_clean.cmake
.PHONY : CMakeFiles/exe.dir/clean

CMakeFiles/exe.dir/depend:
	cd /media/zzh/Data/infantry_current_6.7/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /media/zzh/Data/infantry_current_6.7 /media/zzh/Data/infantry_current_6.7 /media/zzh/Data/infantry_current_6.7/build /media/zzh/Data/infantry_current_6.7/build /media/zzh/Data/infantry_current_6.7/build/CMakeFiles/exe.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/exe.dir/depend

