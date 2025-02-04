# CMake generated Testfile for 
# Source directory: /catkin_ws/src/pepper_bringup
# Build directory: /catkin_ws/build/pepper_bringup
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_pepper_bringup_roslaunch-check_launch "/catkin_ws/build/pepper_bringup/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/kinetic/share/catkin/cmake/test/run_tests.py" "/catkin_ws/build/pepper_bringup/test_results/pepper_bringup/roslaunch-check_launch.xml" "--return-code" "/usr/bin/cmake -E make_directory /catkin_ws/build/pepper_bringup/test_results/pepper_bringup" "/opt/ros/kinetic/share/roslaunch/cmake/../scripts/roslaunch-check -o '/catkin_ws/build/pepper_bringup/test_results/pepper_bringup/roslaunch-check_launch.xml' '/catkin_ws/src/pepper_bringup/launch' ")
subdirs(gtest)
