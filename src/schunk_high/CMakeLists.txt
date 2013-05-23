cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(schunk_high src/quaternion/dualquaternion.c src/quaternion/quaternion.c)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(schunk_high src/schunk_high.cpp src/quaternion/dualquaternion.c src/quaternion/quaternion.c src/kinematics/link.c src/kinematics/robot.c src/matrix/gmatrix.c src/matrix/gmatrix_linalg.c src/matrix/gmatrix_plus.c)
#rosbuild_add_executable(schunk_cartesian_velocity_controller src/cartesian_velocity_controller.cpp  src/cartesian_controller.cpp src/quaternion/dualquaternion.c src/quaternion/quaternion.c src/kinematics/link.c src/kinematics/robot.c src/matrix/gmatrix.c src/matrix/gmatrix_linalg.c src/matrix/gmatrix_plus.c)
rosbuild_add_executable(schunk_cartesian_position_controller src/cartesian_pose_controller.cpp src/cartesian_controller.cpp src/quaternion/dualquaternion.c src/quaternion/quaternion.c src/kinematics/link.c src/kinematics/robot.c src/matrix/gmatrix.c src/matrix/gmatrix_linalg.c src/matrix/gmatrix_plus.c)
#target_link_libraries(example ${PROJECT_NAME})
