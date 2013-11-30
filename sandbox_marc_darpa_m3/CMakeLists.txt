cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE Release)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_genmsg()
rosbuild_gensrv()

include_directories(src/haptic_controllers/cody src/haptic_controllers/sim_three_link)
#CMAKE_C_FLAGS=-Os
#add_definitions(-O3)
#add_definitions(-Os)
#add_definitions(-CMAKE_C_FLAGS='"-Os"')
#rosbuild_add_compile_flags(Os)
# rosbuild_add_library(${PROJECT_NAME}
#                      src/haptic_controllers/solver.c 
# 		     src/haptic_controllers/matrix_support.c   
# 		     src/haptic_controllers/ldl.c 
# 		     src/haptic_controllers/util.c)

# rosbuild_add_executable(cody_example src/haptic_controllers/cody/testsolver.c 
#                         src/haptic_controllers/cody/solver.c 
# 			src/haptic_controllers/cody/matrix_support.c 
# 			src/haptic_controllers/cody/ldl.c 
# 			src/haptic_controllers/cody/util.c)
# rosbuild_add_compile_flags(cody_example -g -O2)


#rosbuild_add_executable(three_link_example src/haptic_controllers/sim_three_link/testsolver.c        
rosbuild_add_executable(three_link_example src/haptic_controllers/sim_three_link/test_mpc_controller.cpp
                        src/haptic_controllers/sim_three_link/solver.cpp 
			src/haptic_controllers/sim_three_link/matrix_support.cpp
			src/haptic_controllers/sim_three_link/ldl.cpp 
			src/haptic_controllers/sim_three_link/util.cpp)
rosbuild_add_compile_flags(three_link_example -g -O2)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} library)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

#target_link_libraries(example ${PROJECT_NAME})
