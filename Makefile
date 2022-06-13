# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Default target executed when no arguments are given to make.
default_target: all

.PHONY : default_target

# Allow only one "make -f Makefile2" at a time, but pass parallelism.
.NOTPARALLEL:


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
CMAKE_SOURCE_DIR = /home/c/umouse/codetest/myProject

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/c/umouse/codetest/myProject

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "No interactive CMake dialog available..."
	/usr/bin/cmake -E echo No\ interactive\ CMake\ dialog\ available.
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache

.PHONY : edit_cache/fast

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/bin/cmake -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache

.PHONY : rebuild_cache/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /home/c/umouse/codetest/myProject/CMakeFiles /home/c/umouse/codetest/myProject/CMakeFiles/progress.marks
	$(MAKE) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /home/c/umouse/codetest/myProject/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean

.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named proj

# Build rule for target.
proj: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 proj
.PHONY : proj

# fast build rule for target.
proj/fast:
	$(MAKE) -f CMakeFiles/proj.dir/build.make CMakeFiles/proj.dir/build
.PHONY : proj/fast

src/alongWall.o: src/alongWall.cpp.o

.PHONY : src/alongWall.o

# target to build an object file
src/alongWall.cpp.o:
	$(MAKE) -f CMakeFiles/proj.dir/build.make CMakeFiles/proj.dir/src/alongWall.cpp.o
.PHONY : src/alongWall.cpp.o

src/alongWall.i: src/alongWall.cpp.i

.PHONY : src/alongWall.i

# target to preprocess a source file
src/alongWall.cpp.i:
	$(MAKE) -f CMakeFiles/proj.dir/build.make CMakeFiles/proj.dir/src/alongWall.cpp.i
.PHONY : src/alongWall.cpp.i

src/alongWall.s: src/alongWall.cpp.s

.PHONY : src/alongWall.s

# target to generate assembly for a file
src/alongWall.cpp.s:
	$(MAKE) -f CMakeFiles/proj.dir/build.make CMakeFiles/proj.dir/src/alongWall.cpp.s
.PHONY : src/alongWall.cpp.s

src/main.o: src/main.cpp.o

.PHONY : src/main.o

# target to build an object file
src/main.cpp.o:
	$(MAKE) -f CMakeFiles/proj.dir/build.make CMakeFiles/proj.dir/src/main.cpp.o
.PHONY : src/main.cpp.o

src/main.i: src/main.cpp.i

.PHONY : src/main.i

# target to preprocess a source file
src/main.cpp.i:
	$(MAKE) -f CMakeFiles/proj.dir/build.make CMakeFiles/proj.dir/src/main.cpp.i
.PHONY : src/main.cpp.i

src/main.s: src/main.cpp.s

.PHONY : src/main.s

# target to generate assembly for a file
src/main.cpp.s:
	$(MAKE) -f CMakeFiles/proj.dir/build.make CMakeFiles/proj.dir/src/main.cpp.s
.PHONY : src/main.cpp.s

src/manager.o: src/manager.cpp.o

.PHONY : src/manager.o

# target to build an object file
src/manager.cpp.o:
	$(MAKE) -f CMakeFiles/proj.dir/build.make CMakeFiles/proj.dir/src/manager.cpp.o
.PHONY : src/manager.cpp.o

src/manager.i: src/manager.cpp.i

.PHONY : src/manager.i

# target to preprocess a source file
src/manager.cpp.i:
	$(MAKE) -f CMakeFiles/proj.dir/build.make CMakeFiles/proj.dir/src/manager.cpp.i
.PHONY : src/manager.cpp.i

src/manager.s: src/manager.cpp.s

.PHONY : src/manager.s

# target to generate assembly for a file
src/manager.cpp.s:
	$(MAKE) -f CMakeFiles/proj.dir/build.make CMakeFiles/proj.dir/src/manager.cpp.s
.PHONY : src/manager.cpp.s

src/singlyList.o: src/singlyList.cpp.o

.PHONY : src/singlyList.o

# target to build an object file
src/singlyList.cpp.o:
	$(MAKE) -f CMakeFiles/proj.dir/build.make CMakeFiles/proj.dir/src/singlyList.cpp.o
.PHONY : src/singlyList.cpp.o

src/singlyList.i: src/singlyList.cpp.i

.PHONY : src/singlyList.i

# target to preprocess a source file
src/singlyList.cpp.i:
	$(MAKE) -f CMakeFiles/proj.dir/build.make CMakeFiles/proj.dir/src/singlyList.cpp.i
.PHONY : src/singlyList.cpp.i

src/singlyList.s: src/singlyList.cpp.s

.PHONY : src/singlyList.s

# target to generate assembly for a file
src/singlyList.cpp.s:
	$(MAKE) -f CMakeFiles/proj.dir/build.make CMakeFiles/proj.dir/src/singlyList.cpp.s
.PHONY : src/singlyList.cpp.s

src/skeletonService.o: src/skeletonService.cpp.o

.PHONY : src/skeletonService.o

# target to build an object file
src/skeletonService.cpp.o:
	$(MAKE) -f CMakeFiles/proj.dir/build.make CMakeFiles/proj.dir/src/skeletonService.cpp.o
.PHONY : src/skeletonService.cpp.o

src/skeletonService.i: src/skeletonService.cpp.i

.PHONY : src/skeletonService.i

# target to preprocess a source file
src/skeletonService.cpp.i:
	$(MAKE) -f CMakeFiles/proj.dir/build.make CMakeFiles/proj.dir/src/skeletonService.cpp.i
.PHONY : src/skeletonService.cpp.i

src/skeletonService.s: src/skeletonService.cpp.s

.PHONY : src/skeletonService.s

# target to generate assembly for a file
src/skeletonService.cpp.s:
	$(MAKE) -f CMakeFiles/proj.dir/build.make CMakeFiles/proj.dir/src/skeletonService.cpp.s
.PHONY : src/skeletonService.cpp.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... edit_cache"
	@echo "... rebuild_cache"
	@echo "... proj"
	@echo "... src/alongWall.o"
	@echo "... src/alongWall.i"
	@echo "... src/alongWall.s"
	@echo "... src/main.o"
	@echo "... src/main.i"
	@echo "... src/main.s"
	@echo "... src/manager.o"
	@echo "... src/manager.i"
	@echo "... src/manager.s"
	@echo "... src/singlyList.o"
	@echo "... src/singlyList.i"
	@echo "... src/singlyList.s"
	@echo "... src/skeletonService.o"
	@echo "... src/skeletonService.i"
	@echo "... src/skeletonService.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system
