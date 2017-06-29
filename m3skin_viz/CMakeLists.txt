cmake_minimum_required(VERSION 2.8.3)
project(m3skin_viz)

find_package(catkin REQUIRED COMPONENTS visualization_msgs tf std_msgs roscpp m3skin_ros hrl_haptic_manipulation_in_clutter_msgs visualization_msgs)

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

catkin_package(
    DEPENDS 
    CATKIN_DEPENDS visualization_msgs tf std_msgs roscpp m3skin_ros hrl_haptic_manipulation_in_clutter_msgs visualization_msgs
    INCLUDE_DIRS 
    LIBRARIES 
)

add_executable(m3skin_viz src/skin.cpp 
              src/RosPressureVisualizer.h
              src/RosPressureVisualizer.cpp)

add_dependencies(m3skin_viz ${catkin_EXPORTED_TARGETS})

target_link_libraries(m3skin_viz ${catkin_LIBRARIES})

install(TARGETS m3skin_viz
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

