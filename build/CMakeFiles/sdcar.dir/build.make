# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/jom/Documents/prgs/projects/sdcar

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jom/Documents/prgs/projects/sdcar/build

# Include any dependencies generated for this target.
include CMakeFiles/sdcar.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/sdcar.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/sdcar.dir/flags.make

CMakeFiles/sdcar.dir/main.cpp.o: CMakeFiles/sdcar.dir/flags.make
CMakeFiles/sdcar.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jom/Documents/prgs/projects/sdcar/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/sdcar.dir/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sdcar.dir/main.cpp.o -c /home/jom/Documents/prgs/projects/sdcar/main.cpp

CMakeFiles/sdcar.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sdcar.dir/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jom/Documents/prgs/projects/sdcar/main.cpp > CMakeFiles/sdcar.dir/main.cpp.i

CMakeFiles/sdcar.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sdcar.dir/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jom/Documents/prgs/projects/sdcar/main.cpp -o CMakeFiles/sdcar.dir/main.cpp.s

CMakeFiles/sdcar.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/sdcar.dir/main.cpp.o.requires

CMakeFiles/sdcar.dir/main.cpp.o.provides: CMakeFiles/sdcar.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/sdcar.dir/build.make CMakeFiles/sdcar.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/sdcar.dir/main.cpp.o.provides

CMakeFiles/sdcar.dir/main.cpp.o.provides.build: CMakeFiles/sdcar.dir/main.cpp.o


CMakeFiles/sdcar.dir/mpc/mpc.cpp.o: CMakeFiles/sdcar.dir/flags.make
CMakeFiles/sdcar.dir/mpc/mpc.cpp.o: ../mpc/mpc.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jom/Documents/prgs/projects/sdcar/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/sdcar.dir/mpc/mpc.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sdcar.dir/mpc/mpc.cpp.o -c /home/jom/Documents/prgs/projects/sdcar/mpc/mpc.cpp

CMakeFiles/sdcar.dir/mpc/mpc.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sdcar.dir/mpc/mpc.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jom/Documents/prgs/projects/sdcar/mpc/mpc.cpp > CMakeFiles/sdcar.dir/mpc/mpc.cpp.i

CMakeFiles/sdcar.dir/mpc/mpc.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sdcar.dir/mpc/mpc.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jom/Documents/prgs/projects/sdcar/mpc/mpc.cpp -o CMakeFiles/sdcar.dir/mpc/mpc.cpp.s

CMakeFiles/sdcar.dir/mpc/mpc.cpp.o.requires:

.PHONY : CMakeFiles/sdcar.dir/mpc/mpc.cpp.o.requires

CMakeFiles/sdcar.dir/mpc/mpc.cpp.o.provides: CMakeFiles/sdcar.dir/mpc/mpc.cpp.o.requires
	$(MAKE) -f CMakeFiles/sdcar.dir/build.make CMakeFiles/sdcar.dir/mpc/mpc.cpp.o.provides.build
.PHONY : CMakeFiles/sdcar.dir/mpc/mpc.cpp.o.provides

CMakeFiles/sdcar.dir/mpc/mpc.cpp.o.provides.build: CMakeFiles/sdcar.dir/mpc/mpc.cpp.o


# Object files for target sdcar
sdcar_OBJECTS = \
"CMakeFiles/sdcar.dir/main.cpp.o" \
"CMakeFiles/sdcar.dir/mpc/mpc.cpp.o"

# External object files for target sdcar
sdcar_EXTERNAL_OBJECTS =

sdcar: CMakeFiles/sdcar.dir/main.cpp.o
sdcar: CMakeFiles/sdcar.dir/mpc/mpc.cpp.o
sdcar: CMakeFiles/sdcar.dir/build.make
sdcar: CMakeFiles/sdcar.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jom/Documents/prgs/projects/sdcar/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable sdcar"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sdcar.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/sdcar.dir/build: sdcar

.PHONY : CMakeFiles/sdcar.dir/build

CMakeFiles/sdcar.dir/requires: CMakeFiles/sdcar.dir/main.cpp.o.requires
CMakeFiles/sdcar.dir/requires: CMakeFiles/sdcar.dir/mpc/mpc.cpp.o.requires

.PHONY : CMakeFiles/sdcar.dir/requires

CMakeFiles/sdcar.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/sdcar.dir/cmake_clean.cmake
.PHONY : CMakeFiles/sdcar.dir/clean

CMakeFiles/sdcar.dir/depend:
	cd /home/jom/Documents/prgs/projects/sdcar/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jom/Documents/prgs/projects/sdcar /home/jom/Documents/prgs/projects/sdcar /home/jom/Documents/prgs/projects/sdcar/build /home/jom/Documents/prgs/projects/sdcar/build /home/jom/Documents/prgs/projects/sdcar/build/CMakeFiles/sdcar.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/sdcar.dir/depend

