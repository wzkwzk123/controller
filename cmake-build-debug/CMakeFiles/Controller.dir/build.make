# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.15

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
CMAKE_SOURCE_DIR = /home/cjh/wzk/masterThesis/Controller

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cjh/wzk/masterThesis/Controller/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/Controller.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Controller.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Controller.dir/flags.make

CMakeFiles/Controller.dir/main.cpp.o: CMakeFiles/Controller.dir/flags.make
CMakeFiles/Controller.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cjh/wzk/masterThesis/Controller/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Controller.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Controller.dir/main.cpp.o -c /home/cjh/wzk/masterThesis/Controller/main.cpp

CMakeFiles/Controller.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Controller.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cjh/wzk/masterThesis/Controller/main.cpp > CMakeFiles/Controller.dir/main.cpp.i

CMakeFiles/Controller.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Controller.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cjh/wzk/masterThesis/Controller/main.cpp -o CMakeFiles/Controller.dir/main.cpp.s

CMakeFiles/Controller.dir/src/mpc_controller.cpp.o: CMakeFiles/Controller.dir/flags.make
CMakeFiles/Controller.dir/src/mpc_controller.cpp.o: ../src/mpc_controller.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cjh/wzk/masterThesis/Controller/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/Controller.dir/src/mpc_controller.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Controller.dir/src/mpc_controller.cpp.o -c /home/cjh/wzk/masterThesis/Controller/src/mpc_controller.cpp

CMakeFiles/Controller.dir/src/mpc_controller.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Controller.dir/src/mpc_controller.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cjh/wzk/masterThesis/Controller/src/mpc_controller.cpp > CMakeFiles/Controller.dir/src/mpc_controller.cpp.i

CMakeFiles/Controller.dir/src/mpc_controller.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Controller.dir/src/mpc_controller.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cjh/wzk/masterThesis/Controller/src/mpc_controller.cpp -o CMakeFiles/Controller.dir/src/mpc_controller.cpp.s

CMakeFiles/Controller.dir/src/controller.cpp.o: CMakeFiles/Controller.dir/flags.make
CMakeFiles/Controller.dir/src/controller.cpp.o: ../src/controller.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cjh/wzk/masterThesis/Controller/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/Controller.dir/src/controller.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Controller.dir/src/controller.cpp.o -c /home/cjh/wzk/masterThesis/Controller/src/controller.cpp

CMakeFiles/Controller.dir/src/controller.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Controller.dir/src/controller.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cjh/wzk/masterThesis/Controller/src/controller.cpp > CMakeFiles/Controller.dir/src/controller.cpp.i

CMakeFiles/Controller.dir/src/controller.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Controller.dir/src/controller.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cjh/wzk/masterThesis/Controller/src/controller.cpp -o CMakeFiles/Controller.dir/src/controller.cpp.s

CMakeFiles/Controller.dir/test/test1.cpp.o: CMakeFiles/Controller.dir/flags.make
CMakeFiles/Controller.dir/test/test1.cpp.o: ../test/test1.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cjh/wzk/masterThesis/Controller/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/Controller.dir/test/test1.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Controller.dir/test/test1.cpp.o -c /home/cjh/wzk/masterThesis/Controller/test/test1.cpp

CMakeFiles/Controller.dir/test/test1.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Controller.dir/test/test1.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cjh/wzk/masterThesis/Controller/test/test1.cpp > CMakeFiles/Controller.dir/test/test1.cpp.i

CMakeFiles/Controller.dir/test/test1.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Controller.dir/test/test1.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cjh/wzk/masterThesis/Controller/test/test1.cpp -o CMakeFiles/Controller.dir/test/test1.cpp.s

CMakeFiles/Controller.dir/src/mpc_solver.cpp.o: CMakeFiles/Controller.dir/flags.make
CMakeFiles/Controller.dir/src/mpc_solver.cpp.o: ../src/mpc_solver.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cjh/wzk/masterThesis/Controller/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/Controller.dir/src/mpc_solver.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Controller.dir/src/mpc_solver.cpp.o -c /home/cjh/wzk/masterThesis/Controller/src/mpc_solver.cpp

CMakeFiles/Controller.dir/src/mpc_solver.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Controller.dir/src/mpc_solver.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cjh/wzk/masterThesis/Controller/src/mpc_solver.cpp > CMakeFiles/Controller.dir/src/mpc_solver.cpp.i

CMakeFiles/Controller.dir/src/mpc_solver.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Controller.dir/src/mpc_solver.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cjh/wzk/masterThesis/Controller/src/mpc_solver.cpp -o CMakeFiles/Controller.dir/src/mpc_solver.cpp.s

CMakeFiles/Controller.dir/src/qp_solver/qp_solver.cpp.o: CMakeFiles/Controller.dir/flags.make
CMakeFiles/Controller.dir/src/qp_solver/qp_solver.cpp.o: ../src/qp_solver/qp_solver.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cjh/wzk/masterThesis/Controller/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/Controller.dir/src/qp_solver/qp_solver.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Controller.dir/src/qp_solver/qp_solver.cpp.o -c /home/cjh/wzk/masterThesis/Controller/src/qp_solver/qp_solver.cpp

CMakeFiles/Controller.dir/src/qp_solver/qp_solver.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Controller.dir/src/qp_solver/qp_solver.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cjh/wzk/masterThesis/Controller/src/qp_solver/qp_solver.cpp > CMakeFiles/Controller.dir/src/qp_solver/qp_solver.cpp.i

CMakeFiles/Controller.dir/src/qp_solver/qp_solver.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Controller.dir/src/qp_solver/qp_solver.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cjh/wzk/masterThesis/Controller/src/qp_solver/qp_solver.cpp -o CMakeFiles/Controller.dir/src/qp_solver/qp_solver.cpp.s

CMakeFiles/Controller.dir/src/qp_solver/active_set_qp_solver.cpp.o: CMakeFiles/Controller.dir/flags.make
CMakeFiles/Controller.dir/src/qp_solver/active_set_qp_solver.cpp.o: ../src/qp_solver/active_set_qp_solver.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cjh/wzk/masterThesis/Controller/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/Controller.dir/src/qp_solver/active_set_qp_solver.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Controller.dir/src/qp_solver/active_set_qp_solver.cpp.o -c /home/cjh/wzk/masterThesis/Controller/src/qp_solver/active_set_qp_solver.cpp

CMakeFiles/Controller.dir/src/qp_solver/active_set_qp_solver.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Controller.dir/src/qp_solver/active_set_qp_solver.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cjh/wzk/masterThesis/Controller/src/qp_solver/active_set_qp_solver.cpp > CMakeFiles/Controller.dir/src/qp_solver/active_set_qp_solver.cpp.i

CMakeFiles/Controller.dir/src/qp_solver/active_set_qp_solver.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Controller.dir/src/qp_solver/active_set_qp_solver.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cjh/wzk/masterThesis/Controller/src/qp_solver/active_set_qp_solver.cpp -o CMakeFiles/Controller.dir/src/qp_solver/active_set_qp_solver.cpp.s

CMakeFiles/Controller.dir/src/qp_solver/qp_solver_gflags.cpp.o: CMakeFiles/Controller.dir/flags.make
CMakeFiles/Controller.dir/src/qp_solver/qp_solver_gflags.cpp.o: ../src/qp_solver/qp_solver_gflags.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cjh/wzk/masterThesis/Controller/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/Controller.dir/src/qp_solver/qp_solver_gflags.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Controller.dir/src/qp_solver/qp_solver_gflags.cpp.o -c /home/cjh/wzk/masterThesis/Controller/src/qp_solver/qp_solver_gflags.cpp

CMakeFiles/Controller.dir/src/qp_solver/qp_solver_gflags.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Controller.dir/src/qp_solver/qp_solver_gflags.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cjh/wzk/masterThesis/Controller/src/qp_solver/qp_solver_gflags.cpp > CMakeFiles/Controller.dir/src/qp_solver/qp_solver_gflags.cpp.i

CMakeFiles/Controller.dir/src/qp_solver/qp_solver_gflags.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Controller.dir/src/qp_solver/qp_solver_gflags.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cjh/wzk/masterThesis/Controller/src/qp_solver/qp_solver_gflags.cpp -o CMakeFiles/Controller.dir/src/qp_solver/qp_solver_gflags.cpp.s

# Object files for target Controller
Controller_OBJECTS = \
"CMakeFiles/Controller.dir/main.cpp.o" \
"CMakeFiles/Controller.dir/src/mpc_controller.cpp.o" \
"CMakeFiles/Controller.dir/src/controller.cpp.o" \
"CMakeFiles/Controller.dir/test/test1.cpp.o" \
"CMakeFiles/Controller.dir/src/mpc_solver.cpp.o" \
"CMakeFiles/Controller.dir/src/qp_solver/qp_solver.cpp.o" \
"CMakeFiles/Controller.dir/src/qp_solver/active_set_qp_solver.cpp.o" \
"CMakeFiles/Controller.dir/src/qp_solver/qp_solver_gflags.cpp.o"

# External object files for target Controller
Controller_EXTERNAL_OBJECTS =

Controller: CMakeFiles/Controller.dir/main.cpp.o
Controller: CMakeFiles/Controller.dir/src/mpc_controller.cpp.o
Controller: CMakeFiles/Controller.dir/src/controller.cpp.o
Controller: CMakeFiles/Controller.dir/test/test1.cpp.o
Controller: CMakeFiles/Controller.dir/src/mpc_solver.cpp.o
Controller: CMakeFiles/Controller.dir/src/qp_solver/qp_solver.cpp.o
Controller: CMakeFiles/Controller.dir/src/qp_solver/active_set_qp_solver.cpp.o
Controller: CMakeFiles/Controller.dir/src/qp_solver/qp_solver_gflags.cpp.o
Controller: CMakeFiles/Controller.dir/build.make
Controller: CMakeFiles/Controller.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cjh/wzk/masterThesis/Controller/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Linking CXX executable Controller"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Controller.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Controller.dir/build: Controller

.PHONY : CMakeFiles/Controller.dir/build

CMakeFiles/Controller.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Controller.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Controller.dir/clean

CMakeFiles/Controller.dir/depend:
	cd /home/cjh/wzk/masterThesis/Controller/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cjh/wzk/masterThesis/Controller /home/cjh/wzk/masterThesis/Controller /home/cjh/wzk/masterThesis/Controller/cmake-build-debug /home/cjh/wzk/masterThesis/Controller/cmake-build-debug /home/cjh/wzk/masterThesis/Controller/cmake-build-debug/CMakeFiles/Controller.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Controller.dir/depend
