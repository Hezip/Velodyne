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
CMAKE_SOURCE_DIR = /home/he/ROS_Code/dua_velodye/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/he/ROS_Code/dua_velodye/build

# Utility rule file for run_tests_velodyne_pointcloud_roslaunch-check_launch.

# Include the progress variables for this target.
include velodyne/velodyne_pointcloud/CMakeFiles/run_tests_velodyne_pointcloud_roslaunch-check_launch.dir/progress.make

velodyne/velodyne_pointcloud/CMakeFiles/run_tests_velodyne_pointcloud_roslaunch-check_launch:
	cd /home/he/ROS_Code/dua_velodye/build/velodyne/velodyne_pointcloud && ../../catkin_generated/env_cached.sh /usr/bin/python /opt/ros/kinetic/share/catkin/cmake/test/run_tests.py /home/he/ROS_Code/dua_velodye/build/test_results/velodyne_pointcloud/roslaunch-check_launch.xml /usr/bin/cmake\ -E\ make_directory\ /home/he/ROS_Code/dua_velodye/build/test_results/velodyne_pointcloud /opt/ros/kinetic/share/roslaunch/cmake/../scripts/roslaunch-check\ -o\ '/home/he/ROS_Code/dua_velodye/build/test_results/velodyne_pointcloud/roslaunch-check_launch.xml'\ '/home/he/ROS_Code/dua_velodye/src/velodyne/velodyne_pointcloud/launch'\ 

run_tests_velodyne_pointcloud_roslaunch-check_launch: velodyne/velodyne_pointcloud/CMakeFiles/run_tests_velodyne_pointcloud_roslaunch-check_launch
run_tests_velodyne_pointcloud_roslaunch-check_launch: velodyne/velodyne_pointcloud/CMakeFiles/run_tests_velodyne_pointcloud_roslaunch-check_launch.dir/build.make

.PHONY : run_tests_velodyne_pointcloud_roslaunch-check_launch

# Rule to build all files generated by this target.
velodyne/velodyne_pointcloud/CMakeFiles/run_tests_velodyne_pointcloud_roslaunch-check_launch.dir/build: run_tests_velodyne_pointcloud_roslaunch-check_launch

.PHONY : velodyne/velodyne_pointcloud/CMakeFiles/run_tests_velodyne_pointcloud_roslaunch-check_launch.dir/build

velodyne/velodyne_pointcloud/CMakeFiles/run_tests_velodyne_pointcloud_roslaunch-check_launch.dir/clean:
	cd /home/he/ROS_Code/dua_velodye/build/velodyne/velodyne_pointcloud && $(CMAKE_COMMAND) -P CMakeFiles/run_tests_velodyne_pointcloud_roslaunch-check_launch.dir/cmake_clean.cmake
.PHONY : velodyne/velodyne_pointcloud/CMakeFiles/run_tests_velodyne_pointcloud_roslaunch-check_launch.dir/clean

velodyne/velodyne_pointcloud/CMakeFiles/run_tests_velodyne_pointcloud_roslaunch-check_launch.dir/depend:
	cd /home/he/ROS_Code/dua_velodye/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/he/ROS_Code/dua_velodye/src /home/he/ROS_Code/dua_velodye/src/velodyne/velodyne_pointcloud /home/he/ROS_Code/dua_velodye/build /home/he/ROS_Code/dua_velodye/build/velodyne/velodyne_pointcloud /home/he/ROS_Code/dua_velodye/build/velodyne/velodyne_pointcloud/CMakeFiles/run_tests_velodyne_pointcloud_roslaunch-check_launch.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : velodyne/velodyne_pointcloud/CMakeFiles/run_tests_velodyne_pointcloud_roslaunch-check_launch.dir/depend

